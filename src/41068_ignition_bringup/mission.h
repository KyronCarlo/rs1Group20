#ifndef MISSION_H
#define MISSION_H

#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/pose.hpp"

class Mission : public rclcpp::Node
{
public:
    Mission();

private:
    // Add a new goal (position)
    void sendGoals();
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr goal_pub_;
    geometry_msgs::msg::PoseArray goal_array;
};

#endif // MISSION_H
