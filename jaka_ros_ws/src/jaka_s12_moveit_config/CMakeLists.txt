cmake_minimum_required(VERSION 3.22)
project(jaka_s12_moveit_config)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED COMPONENTS common filters) # Added COMPONENTS here

ament_package()

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/launch")
  install(
    DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}
    PATTERN "setup_assistant.launch" EXCLUDE)
endif()

add_executable(statistical_filter_node src/statistical_filter_node.cpp)

# 3. Use the ${PCL_LIBRARIES} variable instead of PCL::common
target_link_libraries(statistical_filter_node 
  ${PCL_LIBRARIES}
)


# 4. Standard ROS 2 dependency handling
target_include_directories(statistical_filter_node PUBLIC ${PCL_INCLUDE_DIRS})
ament_target_dependencies(statistical_filter_node 
  rclcpp 
  sensor_msgs 
  pcl_conversions
)

install(TARGETS statistical_filter_node
  DESTINATION lib/${PROJECT_NAME})


install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(DIRECTORY src DESTINATION share/${PROJECT_NAME})

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/worlds")
  install(
    DIRECTORY worlds
    DESTINATION share/${PROJECT_NAME}
  )


endif()
install(FILES .setup_assistant DESTINATION share/${PROJECT_NAME})