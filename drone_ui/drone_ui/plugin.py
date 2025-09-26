import sys
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from std_srvs.srv import SetBool
from geometry_msgs.msg import Point

from qt_gui.plugin import Plugin
from python_qt_binding.QtCore import QTimer
from python_qt_binding.QtGui import QDoubleValidator
from python_qt_binding.QtWidgets import (
    QWidget, QVBoxLayout, QPushButton, QHBoxLayout, QLabel, QComboBox, QLineEdit, QFrame
)


class DroneControlPlugin(Plugin):
    """
    rqt plugin:
      - E-STOP -> /drone/mission (SetBool){data:false} + publishes /emergency_stop=True
      - Takeoff -> /drone/mission (SetBool){data:true}
      - Search mode selector -> 'Automatic search' | 'Manual search'
          When 'Manual search' is selected, user must enter X and Y coordinates.
          On Takeoff in Manual mode, publish geometry_msgs/Point to /userGoal.
    """

    def __init__(self, context):
        super().__init__(context)
        self.setObjectName('DroneControlPlugin')

        # rclpy init (safe if called multiple times)
        try:
            rclpy.init(args=None)
        except Exception:
            pass

        self.node: Node = rclpy.create_node('drone_ui')

        # ---------------- ROS I/O ----------------
        # E-stop state (single source of truth topic)
        self.auto_pub = self.node.create_publisher(Bool, '/explore/resume', 10)

        self._hold_explore_paused = True               # we want explore paused initially
        self._resume_timer = self.node.create_timer(0.2, self._keep_explore_paused)

        # Manual user goal publisher (geometry_msgs/Point)
        self.user_goal_pub = self.node.create_publisher(Point, '/userGoal', 10)

        # Mission service for start/stop (Nav2 wrapper on your side)
        self.mission_client = self.node.create_client(SetBool, '/drone/mission')
        self._pending_future = None

        # ---------------- UI ----------------
        self._widget = QWidget()
        self._widget.setWindowTitle('Drone Controls')
        layout = QVBoxLayout(self._widget)

        self.btn_estop = QPushButton('🚨 E-STOP')
        self.btn_takeoff = QPushButton('⬆️ Takeoff')
        self.btn_estop.setStyleSheet('font-weight: bold; padding:12px; background:#ff6666;')
        self.btn_takeoff.setStyleSheet('padding:10px;')
        layout.addWidget(self.btn_estop)
        layout.addWidget(self.btn_takeoff)

        mode_row = QHBoxLayout()
        mode_row.addWidget(QLabel('Search mode:'))
        self.mode_combo = QComboBox()
        self.mode_combo.addItems(['Automatic search', 'Manual search'])
        self.mode_combo.currentIndexChanged.connect(self._on_mode_changed)
        mode_row.addWidget(self.mode_combo)
        layout.addLayout(mode_row)

        # Manual coordinate inputs
        coords_box = QHBoxLayout()
        self.edit_x = QLineEdit(); self.edit_y = QLineEdit()
        self.edit_x.setPlaceholderText('X'); self.edit_y.setPlaceholderText('Y')
        dv = QDoubleValidator(bottom=-1e12, top=1e12, decimals=6)
        self.edit_x.setValidator(dv); self.edit_y.setValidator(dv)
        coords_box.addWidget(QLabel('X:')); coords_box.addWidget(self.edit_x)
        coords_box.addSpacing(12)
        coords_box.addWidget(QLabel('Y:')); coords_box.addWidget(self.edit_y)
        self.coords_frame = QFrame(); self.coords_frame.setLayout(coords_box)
        layout.addWidget(self.coords_frame)

        self.status_lbl = QLabel('Waiting for /drone/mission...')
        layout.addWidget(self.status_lbl)

        # Signals
        self.btn_estop.clicked.connect(self._on_estop)
        self.btn_takeoff.clicked.connect(self._on_takeoff)
        self.edit_x.textChanged.connect(self._validate_manual_inputs)
        self.edit_y.textChanged.connect(self._validate_manual_inputs)

        # Initial state
        self._on_mode_changed(self.mode_combo.currentIndex())

        # Spin rclpy in Qt loop & poll service
        self.timer = QTimer(self._widget)
        self.timer.timeout.connect(self._spin_once)
        self.timer.start(20)

        # dock in rqt
        if context is not None:
            context.add_widget(self._widget)

        # Initial service status
        self._update_service_status(first=True)

    # ---------- UI callbacks ----------

    def _on_estop(self):
        # Latch and broadcast E-STOP
        self._call_mission(False)  # stop mission / cancel nav2
        self._publish_explore(False) # stop autonomous nav
        self.status_lbl.setText('E-STOP engaged. Stopped explore  and mission')
        self.node.get_logger().warn('E-STOP engaged')

    def _on_takeoff(self):
        # Clear E-STOP (so both stacks can run again)

        manual = (self.mode_combo.currentText() == 'Manual search')
        if manual:
            valid, x, y = self._get_manual_xy()
            if not valid:
                self.status_lbl.setText('Enter valid X and Y for Manual search.')
                self.node.get_logger().warn('Manual search requires valid X and Y.')
                return
            # Stop automatic navigation
            self._publish_explore(False)
            # Publish user goal immediately on /userGoal (geometry_msgs/Point)
            self._publish_user_goal(x, y)
            # Start mission (SetBool true) 
            self._call_mission(True)
        
        else:
            # Auto: stop manual mission first, then resume explore
            # So it doesnt start the auon nav until the manual one is cancelled
            def start_auto(resp):
                if resp and resp.success:
                    #if hasattr(self, '_hold_explore_paused'):
                     #   self._hold_explore_paused = False  # stop any pause-guard
                    self._publish_explore(True)
                    self.status_lbl.setText('Auto exploration resumed')
                else:
                    self.node.get_logger().warn('Stop mission failed; not resuming explore')
            self._call_mission(False, on_done=start_auto)

    def _on_mode_changed(self, _idx: int):
        manual = (self.mode_combo.currentText() == 'Manual search')
        self.coords_frame.setVisible(manual)
        self._validate_manual_inputs()

    # ---------- Helpers ----------

    def _publish_user_goal(self, x: float, y: float):
        pt = Point(); pt.x = float(x); pt.y = float(y); pt.z = 0.0
        self.user_goal_pub.publish(pt)
        self.node.get_logger().info(f'Published /userGoal: x={pt.x:.3f}, y={pt.y:.3f}, z={pt.z:.3f}')
        self.status_lbl.setText(f'Sent /userGoal to (x={pt.x:.3f}, y={pt.y:.3f})')

    def _get_manual_xy(self):
        x_text = self.edit_x.text().strip(); y_text = self.edit_y.text().strip()
        try:
            x = float(x_text); y = float(y_text)
            return True, x, y
        except ValueError:
            return False, None, None

    def _validate_manual_inputs(self):
        manual = (self.mode_combo.currentText() == 'Manual search')
        if manual:
            valid, _, _ = self._get_manual_xy()
            self.btn_takeoff.setEnabled(self.mission_client.service_is_ready() and valid)
        else:
            self.btn_takeoff.setEnabled(self.mission_client.service_is_ready())

    def _call_mission(self, start: bool, on_done=None):
        if not self.mission_client.service_is_ready():
            self.node.get_logger().warn('/drone/mission not available yet')
            self._update_service_status()
            return
        if self._pending_future is not None:
            self.node.get_logger().info('Mission call already in progress...')
            return

        req = SetBool.Request(); req.data = bool(start)
        self._disable_ui(True)
        self.status_lbl.setText(f'Calling /drone/mission: {"start" if start else "stop"}...')
        self._pending_future = self.mission_client.call_async(req)

        def _done_callback(fut):
            try:
                resp = fut.result()
                if resp.success:
                    self.node.get_logger().info(f'/drone/mission success: {resp.message}')
                else:
                    self.node.get_logger().warn(f'/drone/mission failed: {resp.message}')
            except Exception as e:
                self.node.get_logger().error(f'/drone/mission exception: {e}')
                resp = None
            finally:
                self._pending_future = None
                self._disable_ui(False)
                self._validate_manual_inputs()
                if on_done:
                    try:
                        on_done(resp)
                    except Exception as e:
                        self.node.get_logger().error(f'on_done callback error: {e}')

        self._pending_future.add_done_callback(_done_callback)

    def _publish_explore(self, resume: bool):
        self.auto_pub.publish(Bool(data=bool(resume)))
        self.node.get_logger().info(f'Published /explore/resume: {resume}')

    def _update_service_status(self, first: bool = False):
        ready = self.mission_client.service_is_ready()
        subCount = self.auto_pub.get_subscription_count()
        if ready and (subCount > 0):
            self.status_lbl.setText('Nodes running')
            self._disable_ui(False)
            self._validate_manual_inputs()
            if first:
                self.node.get_logger().info('Connected to /drone/mission and /explore/resume')
        else:
            self.status_lbl.setText('Waiting for /drone/mission and /explore/resume...')
            self._disable_ui(True)

    def _disable_ui(self, disable: bool):
        self.btn_estop.setEnabled(not disable and self.mission_client.service_is_ready())
        # Takeoff enable is managed by _validate_manual_inputs
        self.mode_combo.setEnabled(not disable)
        self.edit_x.setEnabled(not disable)
        self.edit_y.setEnabled(not disable)

    def _keep_explore_paused(self):
        # Keep asserting False until user starts, or until a subscriber is present
        if self._hold_explore_paused:
            self.auto_pub.publish(Bool(data=False))
            
            # Optional: stop once someone is listening
            if self.auto_pub.get_subscription_count() > 0:
                self.node.get_logger().info('explore_lite subscribed; keeping /explore/resume=False')
                self._hold_explore_paused = False

    # ---------- rclpy integration ----------

    def _spin_once(self):
        rclpy.spin_once(self.node, timeout_sec=0.0)
        self._update_service_status()

        if self._pending_future and self._pending_future.done():
            try:
                resp = self._pending_future.result()
                if resp.success:
                    self.status_lbl.setText(f'/drone/mission ok: {resp.message}')
                    self.node.get_logger().info(f'/drone/mission success: {resp.message}')
                else:
                    self.status_lbl.setText(f'/drone/mission failed: {resp.message}')
                    self.node.get_logger().warn(f'/drone/mission failed: {resp.message}')
                    # added
                if getattr(self, "_pending_on_done", None):
                    cb = self._pending_on_done
                    self._pending_on_done = None
                    try:
                        cb()
                    except Exception as e:
                        self.node.get_logger().error(f'on_done callback error: {e}')
                        #added
            except Exception as e:
                self.node.get_logger().error(f'on_done callback error: {e}')
            except Exception as e:
                self.status_lbl.setText(f'/drone/mission error: {e}')
                self.node.get_logger().error(f'/drone/mission exception: {e}')
            finally:
                self._pending_future = None
                self._disable_ui(False)
                self._validate_manual_inputs()

    # ---------- rqt lifecycle ----------

    def shutdown_plugin(self):
        try:
            self.timer.stop()
        except Exception:
            pass
        if self.node is not None:
            self.node.destroy_node()


def standalone_main():
    from python_qt_binding.QtWidgets import QApplication, QWidget, QVBoxLayout, QLabel
    app = QApplication(sys.argv)
    w = QWidget(); w.setWindowTitle('Drone UI (standalone)')
    lay = QVBoxLayout(w); lay.addWidget(QLabel('Open rqt to load DroneControlPlugin'))
    w.show()
    return app.exec()


# ---------------------------
# Example snippet for your MANUAL controller to honor E-STOP:
# (Put this in your manual motion node; it should also stop any timers/loops)
#
# class ManualController(Node):
#     def __init__(self):
#         super().__init__('manual_controller')
#         self._estop = False
#         self.create_subscription(Bool, '/emergency_stop', self._on_estop, 10)
#         # ... your other pubs/subs here ...
#
#     def _on_estop(self, msg: Bool):
#         self._estop = bool(msg.data)
#         if self._estop:
#             self.get_logger().warn('E-STOP active -> halting manual control')
#             # stop motors, cancel timers, clear goal followers here
#         else:
#             self.get_logger().info('E-STOP cleared -> manual control allowed')
#
#     def drive_loop(self):
#         if self._estop:
#             return  # do not publish motion if e-stop is on
#         # ... publish your manual cmd_vel or trajectory here ...
