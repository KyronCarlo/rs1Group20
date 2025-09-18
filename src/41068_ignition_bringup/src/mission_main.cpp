#include "rclcpp/rclcpp.hpp"
#include "mission.h"

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Mission>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}