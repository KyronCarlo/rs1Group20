import sys
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from std_srvs.srv import SetBool

from qt_gui.plugin import Plugin
from python_qt_binding.QtCore import QTimer
from python_qt_binding.QtGui import QDoubleValidator
from python_qt_binding.QtWidgets import (
    QWidget, QVBoxLayout, QPushButton, QHBoxLayout, QLabel, QComboBox, QLineEdit, QFrame
)


class DroneControlPlugin(Plugin):
    """
    rqt plugin:
      - 🚨 E-STOP -> /drone/mission (SetBool){data:false}   (+ publishes /emergency_stop=True)
      - ⬆️ Takeoff -> /drone/mission (SetBool){data:true}
      - Search mode selector -> 'Automatic search' | 'Manual search'
          When 'Manual search' is selected, user must enter X and Y coordinates
          which will be sent to the drone mission (see commented-out ROS block).
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

        # Publishers
        self.estop_pub = self.node.create_publisher(Bool, '/emergency_stop', 10)

        # Service client for start/stop mission
        self.mission_client = self.node.create_client(SetBool, '/drone/mission')
        self._pending_future = None

        # ---- UI ----
        self._widget = QWidget()
        self._widget.setWindowTitle('Drone Controls')
        layout = QVBoxLayout(self._widget)

        # Buttons (no Land button)
        self.btn_estop = QPushButton('🚨 E-STOP')
        self.btn_takeoff = QPushButton('⬆️ Takeoff')

        # styling
        self.btn_estop.setStyleSheet('font-weight: bold; padding:12px; background:#ff6666;')
        self.btn_takeoff.setStyleSheet('padding:10px;')

        layout.addWidget(self.btn_estop)
        layout.addWidget(self.btn_takeoff)

        # --- Search mode selector (replaces speed selector) ---
        mode_row = QHBoxLayout()
        mode_row.addWidget(QLabel('Search mode:'))
        self.mode_combo = QComboBox()
        self.mode_combo.addItems(['Automatic search', 'Manual search'])
        self.mode_combo.currentIndexChanged.connect(self._on_mode_changed)
        mode_row.addWidget(self.mode_combo)
        layout.addLayout(mode_row)

        # --- Manual coordinate inputs (shown only when Manual search is selected) ---
        coords_box = QHBoxLayout()
        self.edit_x = QLineEdit(); self.edit_y = QLineEdit()
        self.edit_x.setPlaceholderText('X'); self.edit_y.setPlaceholderText('Y')

        # Only allow valid floats
        dv = QDoubleValidator(bottom=-1e12, top=1e12, decimals=6)
        self.edit_x.setValidator(dv)
        self.edit_y.setValidator(dv)

        coords_box.addWidget(QLabel('X:'))
        coords_box.addWidget(self.edit_x)
        coords_box.addSpacing(12)
        coords_box.addWidget(QLabel('Y:'))
        coords_box.addWidget(self.edit_y)

        # put the coords row in a small frame so we can hide/show cleanly
        self.coords_frame = QFrame()
        self.coords_frame.setLayout(coords_box)
        layout.addWidget(self.coords_frame)

        # Status line
        self.status_lbl = QLabel('Waiting for /drone/mission...')
        layout.addWidget(self.status_lbl)

        # Wire up signals
        self.btn_estop.clicked.connect(self._on_estop)
        self.btn_takeoff.clicked.connect(self._on_takeoff)
        self.edit_x.textChanged.connect(self._validate_manual_inputs)
        self.edit_y.textChanged.connect(self._validate_manual_inputs)

        # Initial mode state
        self._on_mode_changed(self.mode_combo.currentIndex())

        # Spin rclpy in Qt loop & poll service
        self.timer = QTimer(self._widget)
        self.timer.timeout.connect(self._spin_once)
        self.timer.start(20)

        # dock in rqt
        if context is not None:
            context.add_widget(self._widget)

        # Initial service status check
        self._update_service_status(first=True)

        # -------------------------------
        # COMMENTED-OUT NODE COMMUNICATION FOR MANUAL COORDINATES
        #
        # Example A) Publish a manual goal on a topic (uncomment & adapt as needed):
        #
        # from geometry_msgs.msg import PointStamped
        # self.manual_goal_pub = self.node.create_publisher(PointStamped, '/manual_search_goal', 10)
        #
        # def _send_manual_goal(self, x: float, y: float):
        #     msg = PointStamped()
        #     msg.header.frame_id = 'map'
        #     msg.point.x = x
        #     msg.point.y = y
        #     msg.point.z = 0.0
        #     self.manual_goal_pub.publish(msg)
        #     self.node.get_logger().info(f'Published manual goal to /manual_search_goal: x={x}, y={y}')
        #
        # Example B) Call a hypothetical service that starts a mission with XY (replace srv type):
        #
        # from custom_msgs.srv import StartManualMission  # <-- replace with your actual package/type
        # self.manual_mission_client = self.node.create_client(StartManualMission, '/drone/mission_manual')
        #
        # def _call_manual_mission(self, x: float, y: float):
        #     if not self.manual_mission_client.service_is_ready():
        #         self.node.get_logger().warn('/drone/mission_manual not available yet')
        #         return
        #     req = StartManualMission.Request()
        #     req.x = x
        #     req.y = y
        #     self._pending_future = self.manual_mission_client.call_async(req)
        # -------------------------------

    # ---------- UI callbacks ----------

    def _on_estop(self):
        # Publish a Bool so any safety mux can latch zero velocity
        self.estop_pub.publish(Bool(data=True))
        self._call_mission(False)  # stop mission

    def _on_takeoff(self):
        # If manual mode, validate and (in a real setup) send coordinates along with mission start
        manual = (self.mode_combo.currentText() == 'Manual search')
        if manual:
            valid, x, y = self._get_manual_xy()
            if not valid:
                self.status_lbl.setText('Enter valid X and Y for Manual search.')
                self.node.get_logger().warn('Manual search requires valid X and Y.')
                return

            # -- Commented out: where you would send the coordinates to your stack --
            # self._send_manual_goal(x, y)        # topic-based approach (see commented section)
            # self._call_manual_mission(x, y)     # service-based approach (see commented section)
            self.node.get_logger().info(f'(stub) Would send manual goal: x={x}, y={y}')

        # Start mission (SetBool true) regardless of mode
        self._call_mission(True)

    def _on_mode_changed(self, _idx: int):
        manual = (self.mode_combo.currentText() == 'Manual search')
        self.coords_frame.setVisible(manual)
        self._validate_manual_inputs()

    # ---------- Helpers ----------

    def _get_manual_xy(self):
        """Return (is_valid, x, y) from the text fields."""
        x_text = self.edit_x.text().strip()
        y_text = self.edit_y.text().strip()
        try:
            x = float(x_text)
            y = float(y_text)
            return True, x, y
        except ValueError:
            return False, None, None

    def _validate_manual_inputs(self):
        """Enable Takeoff only if inputs are valid in Manual mode; otherwise follow service status."""
        manual = (self.mode_combo.currentText() == 'Manual search')
        if manual:
            valid, _, _ = self._get_manual_xy()
            # Only enable Takeoff if service is ready AND inputs valid
            self.btn_takeoff.setEnabled(self.mission_client.service_is_ready() and valid)
        else:
            # In automatic mode, service readiness solely controls Takeoff enabled state
            self.btn_takeoff.setEnabled(self.mission_client.service_is_ready())

    def _call_mission(self, start: bool):
        if not self.mission_client.service_is_ready():
            self.node.get_logger().warn('/drone/mission not available yet')
            self._update_service_status()
            return
        if self._pending_future is not None:
            self.node.get_logger().info('Mission call already in progress...')
            return

        req = SetBool.Request()
        req.data = bool(start)
        self._disable_ui(True)
        self.status_lbl.setText(f'Calling /drone/mission: {"start" if start else "stop"}...')
        self._pending_future = self.mission_client.call_async(req)

    def _update_service_status(self, first: bool = False):
        ready = self.mission_client.service_is_ready()
        if ready:
            self.status_lbl.setText('Service ready: /drone/mission (SetBool)')
            self._disable_ui(False)
            self._validate_manual_inputs()  # re-check Takeoff enable for manual inputs
            if first:
                self.node.get_logger().info('Connected to /drone/mission')
        else:
            self.status_lbl.setText('Waiting for /drone/mission...')
            self._disable_ui(True)

    def _disable_ui(self, disable: bool):
        self.btn_estop.setEnabled(not disable and self.mission_client.service_is_ready())
        # Takeoff enable is handled by _validate_manual_inputs so we don’t override it blindly here
        self.mode_combo.setEnabled(not disable)
        self.edit_x.setEnabled(not disable)
        self.edit_y.setEnabled(not disable)

    # ---------- rclpy integration ----------

    def _spin_once(self):
        # spin callbacks
        rclpy.spin_once(self.node, timeout_sec=0.0)

        # poll service availability
        self._update_service_status()

        # handle async service result
        if self._pending_future and self._pending_future.done():
            try:
                resp = self._pending_future.result()
                if resp.success:
                    self.status_lbl.setText(f'/drone/mission ok: {resp.message}')
                    self.node.get_logger().info(f'/drone/mission success: {resp.message}')
                else:
                    self.status_lbl.setText(f'/drone/mission failed: {resp.message}')
                    self.node.get_logger().warn(f'/drone/mission failed: {resp.message}')
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

