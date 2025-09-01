cmake_minimum_required(VERSION 3.8)
project(search_and_rescue)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
# add any other packages as we go
find_package(ament_cmake REQUIRED)
find_package(ros_ign_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_srvs REQUIRED)  
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED) 
find_package(geometry_msgs REQUIRED)

# Controller library
add_library(controller_lib controller.cpp)
ament_target_dependencies(controller_lib rclcpp std_msgs nav_msgs std_srvs tf2 tf2_ros geometry_msgs)

# Controller node
add_executable(controller_node quadcopter.cpp)
ament_target_dependencies(controller_node rclcpp std_msgs nav_msgs std_srvs tf2 tf2_ros geometry_msgs)

target_link_libraries(controller_node controller_lib)

install(TARGETS
  controller_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY config launch models urdf worlds
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
