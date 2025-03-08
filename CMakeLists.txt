cmake_minimum_required(VERSION 3.8)
project(ur_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(control_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(std_srvs REQUIRED)

add_executable(ur_control
        src/ur_control.cpp
        include/ur_control/ur_controller.h
)
ament_target_dependencies(ur_control
        rclcpp
        rclcpp_action
        control_msgs
        trajectory_msgs
        std_srvs
)
add_executable(ur_moveit
        src/ur_moveit.cpp
        include/ur_control/ur_controller.h
)
ament_target_dependencies(ur_moveit
        rclcpp
        moveit_ros_planning_interface
)
install(TARGETS
        ur_control
        DESTINATION lib/${PROJECT_NAME})


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
