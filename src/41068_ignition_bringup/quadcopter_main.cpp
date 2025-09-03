#include "rclcpp/rclcpp.hpp"
#include "quadcopter.h"


int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    
    rclcpp::spin(std::make_shared<Quadcopter>());
    rclcpp::shutdown();
    
    return 0;
}