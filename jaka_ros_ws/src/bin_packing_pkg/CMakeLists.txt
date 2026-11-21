cmake_minimum_required(VERSION 3.8)
project(bin_packing_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(control_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(geometric_shapes REQUIRED)
find_package(std_srvs REQUIRED)

set(executable_list
  bin_picking_node_v7_to_control_jaka_s12
  bin_picking_node_v5_to_control_jaka_s12
  collision_monitor_node
)

foreach(loop_var IN LISTS executable_list)
  add_executable(${loop_var} src/${loop_var}.cpp)
  ament_target_dependencies(${loop_var}
    rclcpp
    geometry_msgs
    sensor_msgs
    std_msgs
    tf2_msgs
    moveit_core
    moveit_ros_planning
    moveit_ros_planning_interface
    rclcpp_action
    control_msgs
    moveit_msgs
    shape_msgs
    geometric_shapes
    std_srvs
  )

  install(TARGETS
    ${loop_var}
    DESTINATION lib/${PROJECT_NAME}
  )
endforeach()

install(DIRECTORY
        launch
        meshes
  DESTINATION share/${PROJECT_NAME}
)

# Simulation helper for the tightening task (gives a nut its thread, see the script docstring).
install(PROGRAMS
  scripts/bolt_fixture_manager.py
  scripts/bolt_detector.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
