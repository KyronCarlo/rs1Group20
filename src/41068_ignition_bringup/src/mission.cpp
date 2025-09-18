#include "mission.h"
#include "tsp_helper.h"

Mission::Mission() : Node("navigation")
{
    have_pose_ = false;
    
    goal_pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>("/mission/goals", 10);
    odom_Sub_ = this->create_subscription<nav_msgs::msg::Odometry>("/odometry", 10, std::bind(&Mission::odomCallback, this, std::placeholders::_1));

    // Wait 200 ms, then call sendGoals()
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(200),
        std::bind(&Mission::sendGoals, this));
}

void Mission::sendGoals()
{
    if (goal_pub_->get_subscription_count() == 0 || !have_pose_) {
        std::cout<<"Waiting for subscribers..."<<std::endl;
        return; 
    }

    std::vector<V3> goals;
    
    goals.push_back({9.0, 9.0, 2.0});
    goals.push_back({9.0, -9.0, 2.0});
    goals.push_back({-9.0, 9.0, 2.0});
    goals.push_back({-9.0, -9.0, 2.0});
    goals.push_back({9.0, 0.0, 2.0});
    goals.push_back({-9.0, 0.0, 2.0});
    goals.push_back({0.0, 9.0, 2.0});
    goals.push_back({0.0, -9.0, 2.0});
    goals.push_back({4.0, 4.0, 2.0});
    goals.push_back({-4.0, 4.0, 2.0});
    goals.push_back({4.0, -4.0, 2.0});
    goals.push_back({-4.0, -4.0, 2.0});

    V3 start{odom_.x, odom_.y, odom_.z};
    auto route = tsp_order_open(start, goals);

    for (const auto &p : route) {
        geometry_msgs::msg::Pose pose;
        pose.position.x = p.x;
        pose.position.y = p.y;
        pose.position.z = p.z;
        goal_array.poses.push_back(pose);
    } 

    goal_pub_->publish(goal_array);
    std::cout<<"NAV GOALS SET"<<std::endl;

    timer_->cancel();
}

void Mission::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg){
    have_pose_ = true;
    odom_= msg->pose.pose.position;
}