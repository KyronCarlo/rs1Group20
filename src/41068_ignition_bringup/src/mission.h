#ifndef MISSION_H
#define MISSION_H

#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "nav_msgs/msg/odometry.hpp"

class Mission : public rclcpp::Node
{
public:
    Mission();

private:
    // Add a new goal (position)
    void sendGoals();
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr goal_pub_;
    geometry_msgs::msg::PoseArray goal_array;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_Sub_; //!< Subscription to odometry
    geometry_msgs::msg::Point odom_;
    bool have_pose_;

};

#endif // MISSION_H