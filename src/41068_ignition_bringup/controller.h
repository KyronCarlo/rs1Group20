#ifndef CONTROLLER_H
#define CONTROLLER_H

#include "controllerinterface.h"
#include <cmath>
#include <thread>
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"

#include "std_srvs/srv/set_bool.hpp"
#include "std_msgs/msg/bool.hpp"
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "std_msgs/msg/float64.hpp"
#include <deque>

/*!
 *  \brief     Controller Class
 *  \details
 *  This is the base class for all the platforms, which includes the functions implemented that are the same for all the platforms.
 *  \author    Robert Attard
 *  \date      2025-04-30
 */

class Controller : public ControllerInterface, public rclcpp::Node
{
  public:

  /*!
  * @brief Default constructor sets all attributes to a default value and gets the threads ready.
  */
  Controller();

  /*!
  * @brief Destructor joins all the threads and stops them.
  */
 virtual ~Controller();
 
 /*!
  * @brief Setter for goals
  @param goals
  @return all goal reachable, in order supplied
  */
 void setGoal(const geometry_msgs::msg::PoseArray& msg);
 
 /*!
  * @brief Set tolerance when reaching goal
  @return tolerance accepted [m]
  */
 bool setTolerance(double tolerance);
 
 /*!
  * @brief getter for total distance travelled by platform
  @return total distance travelled since execution @sa run called with goals supplied
  */
 double distanceTravelled(void);
 
 /*!
  * @brief getter for total time in motion by platform
  @return total time in motion since execution @sa run called with goals supplied
  */
 double timeTravelled(void);

 /*!
  * @brief getter for current odometry information
  @return odometry - current odometry
  */
  geometry_msgs::msg::Pose getOdometry(void);
 
 /*!
  * @brief Getter for distance to be travelled to reach current goal
  @return distance to be travlled to reach current goal [m]
  */
 double distanceToGoal(void);
 
 /*!
  * @brief Getter for time to reach current goal
  @return time to travel to current goal [s]
  */
 double timeToGoal(void);

 /*!
  * @brief Updates the odometry of the vehicle every time it is published to the topic
  */
 void odoCallback(const nav_msgs::msg::Odometry::SharedPtr msg);

 /*!
  * @brief When the service is called, this runs the control logic for reaching the goal
  */
 void runMission(const std::shared_ptr<std_srvs::srv::SetBool::Request> req,std::shared_ptr<std_srvs::srv::SetBool::Response> res);

  /*!
  * @brief This includes the logic for updating the current goal for the platform to reach
  */
 void updateCurrentGoal();

  /*!
  * @brief Getter for the current goal
  @return the position of the current goal
  */
 geometry_msgs::msg::Point getCurrentGoal();

  /*!
  * @brief Callback for communicating in with 'Trial' to access if the goal is within the centre of the road
  */
 void boolCallback(const std_msgs::msg::Bool::SharedPtr msg);

 /*!
  * @brief Calculates the total distance to reach all the goals that are stored
  */
 void getTotalDistance();

 /*!
  * @brief Calculates the percentage complete of the total distance for the task
  */
 void percentComplete();

  /*!
  * @brief Converts yaw to quaternion data
  @return the quaternion values
  */
 geometry_msgs::msg::Quaternion yawToQuaternion(double yaw);


 //! Information about the goal for the platform
 struct GoalStats {
    //! location of goal
    geometry_msgs::msg::Point location;

    //! distance to goal
    double distance;
    //! time to goal
    double time;
};

protected:
 std::deque<geometry_msgs::msg::Point> goals_; //!< Stores the positions of all the goals
 double tolerance_;//!< Stores the set tolerance value
 double distanceTravelled_; //!< The total distance travelled by the platform
 double timeInMotion_; //!< The total time the platform has been in motion
 double InitialDistance; //!< stores the initial distance the car is from the goal
 bool goalReachable; //!< a check if the goal is reacheable from the current position
 bool goalReached; //!< a check if the goal has been reached 
 bool SetInitialDistance; //!< a check if the initial distance has been stores
 geometry_msgs::msg::Pose odom_; //!< The current position of the car
 geometry_msgs::msg::Twist speed_; //!< The current speed of the car
 geometry_msgs::msg::Pose estimatedGoalPose_; //!< stores the estimated goal pose
 std::chrono::steady_clock::time_point startTime; // start time clock variable
 std::chrono::steady_clock::time_point endTime; // end time clock variable
 std::chrono::duration<double> totalTimeTaken; //!< variable to store the total time the vehicle has been moving 
 bool TotalTimeSet; // bool for if the total time of a route has been calculated
 double totalDistanceToAllGoalsValue; //!< Stores the total distance to all the goals

 std::mutex mtxStart_; //!< mutex used to keep data used secure
 std::condition_variable cvStart_;
 std::mutex mtxStart_mission;

 std::vector<std::thread> threads_; //!< We add threads onto a vector here to be able to terminate then in destructor

 bool goalSet_; //!< Flag indicating if a goal has been set

 // subscribers
 rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometrySub_; //!< Subscription to odometry
 rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr goalsSub_; //!< Subscription to goals
 rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr missionService_; //!< Service for mission

 rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr boolSub_; //!< Service for mission

 rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr goalPub_;//!< Publisher for the goal

 double percentCompleted;//!< Stores the percentage value of completed tasks
 geometry_msgs::msg::Point previousPos;//!< Odom value used for calculating distance travelled
 bool firstOdomReceived;//!< Check if the first odom value has been set

 GoalStats goal_;//!< object of Goalstats
 bool isDriving_;//!< check if the vehicle is driving

private:
 std::mutex poseMtx_;
 
 std::mutex goalMtx_; //!< Mutex for controlling access to goal

 bool isGoalInCentre;//!< check if the goal is in the centre of the track

};

#endif // CONTROLLER_H