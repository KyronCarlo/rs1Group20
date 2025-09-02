#ifndef QUADCOPTER_H
#define QUADCOPTER_H

#include "controller.h"
#include <tf2/utils.h>
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/empty.hpp"

/*!
 *  \brief     Quadcopter Class
 *  \details
 *  This is the specefic class for the quadcopter drone, which includes the control logic.
 *  \author    Robert Attard
 *  \date      2025-04-30
 */

// control system states for quadcopter
namespace controlQuad {
  enum State {
      IDLE, /*!< IDLE state of control, when the drone is not moving */
      MOVEUP, /*!< MOVEUP state of control, when the drone needs to move up to the z goal */
      TURN, /*!< TURN state of control, when the drone needs to turn to face the direction of the goal */
      FORWARD, /*!< FORWARD state of control, when the drone is moving towards the target goal */
      ADJUST, /*!< ADJUST state of control, when the drone is moving towards the goal, and needs to readjust to face the goal */
      STOPPING, /*!< STOPPING state of control, when the drone is close to the goal and is stopping */
      MOVEDOWN /*!< MOVEDOWN state of control, when the drone is moving too high and needs to move down to the z goal */
  };
}

class Quadcopter: public Controller
{
public:
  /*! 
  * @brief Default constructor should sets all attributes to a default value.
  */
  Quadcopter();

  /*! 
   * @brief Control logic for the Quadcopter to reach goals
   * This is the control logic for the quadcopter to reach the goals. It is a non blocking call and utilises the states to move to the goals. It start by calling the takeoff command, then moving the quad forward in the direction of the goal while moving it it up to the correct z height. It continues moving forward while constantly readjusting its orientation by turning to face the goal, then comes to a stop when it is close to the goal based on the set tolerance. Once coming to a complete stop, it then starts the control again to reach the next goal, when all goals are reach it exits the loop. 
   */
  void run(void) override;

  /*!
  * @brief Checks whether the quadcopter can travel between its origin and destination
  Checks whether the quadcopter can travel between its origin and destination
  @param[in] origin The origin pose, specified as odometry for the platform
  @param[in] destination The destination point for the platform
  @param[in|out] distance The distance [m] the platform will need to travel between origin and destination. If destination unreachable distance = -1
  @param[in|out] time The time [s] the platform will need to travel between origin and destination, If destination unreachable time = -1
  @param[in|out] estimatedGoalPose The estimated goal pose when reaching goal
  @return bool indicating the platform can reach the destination from origin supplied
  */
  bool checkOriginToDestination(geometry_msgs::msg::Pose origin,
                                        geometry_msgs::msg::Point goal,
                                        double& distance,
                                        double& time,
                                        geometry_msgs::msg::Pose& estimatedGoalPose) override;
 
  /*! 
   * function for normalising angles that is used in checkOriginToDestination function
  */
  double normaliseAngle(double theta);


  private:
 
    void command(double move_forward_back, double move_left_right, double turn_left_right, double move_up_down);
  
    double angleDifference; /*!< variable storee the angle difference between the current orientation and goal orientation*/
    controlQuad::State state; /*!< creating object of the control quadcopter enum*/
    bool inZposition; /*!< a bool check if the drone is in the z position*/

    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr move_f_bPub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr move_l_rPub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr move_u_dPub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr turn_l_rPub_;

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr command_Pub;
    rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr takeOff_Pub; 

    std_msgs::msg::Float64 move_f_b; 
    std_msgs::msg::Float64 turn_l_r;
    std_msgs::msg::Float64 move_u_d; 
    std_msgs::msg::Float64 move_l_r; 

};

#endif // QUADCOPTER_H