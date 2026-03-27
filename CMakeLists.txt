cmake_minimum_required(VERSION 3.10)
project(pointcloud2_inspector)

option(BUILD_ROS1 "Build pointcloud2_inspector for ROS1 (Noetic)" OFF)
option(BUILD_ROS2 "Build pointcloud2_inspector for ROS2 (Foxy/Humble)" OFF)

if(BUILD_ROS1 AND BUILD_ROS2)
  message(FATAL_ERROR "Invalid configuration: BUILD_ROS1 and BUILD_ROS2 cannot both be ON.")
elseif(NOT BUILD_ROS1 AND NOT BUILD_ROS2)
  message(FATAL_ERROR "Invalid configuration: exactly one of BUILD_ROS1 or BUILD_ROS2 must be ON.")
endif()

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_library(pointcloud2_inspector_core
  src/pointcloud2_inspector.cpp
)

target_include_directories(pointcloud2_inspector_core
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

if(BUILD_ROS1)
  find_package(catkin REQUIRED COMPONENTS
    roscpp
    sensor_msgs
    std_msgs
  )

  target_compile_definitions(pointcloud2_inspector_core PUBLIC POINTCLOUD2_INSPECTOR_ROS1)

  catkin_package(
    INCLUDE_DIRS include
    LIBRARIES pointcloud2_inspector_core
    CATKIN_DEPENDS roscpp sensor_msgs std_msgs
  )

  target_include_directories(pointcloud2_inspector_core PUBLIC ${catkin_INCLUDE_DIRS})
  target_link_libraries(pointcloud2_inspector_core PUBLIC ${catkin_LIBRARIES})

  add_executable(pointcloud2_inspector_node src/main_ros1.cpp)
  target_compile_definitions(pointcloud2_inspector_node PRIVATE POINTCLOUD2_INSPECTOR_ROS1)
  target_link_libraries(pointcloud2_inspector_node PRIVATE pointcloud2_inspector_core ${catkin_LIBRARIES})
  target_include_directories(pointcloud2_inspector_node PRIVATE ${catkin_INCLUDE_DIRS})

  install(TARGETS pointcloud2_inspector_core pointcloud2_inspector_node
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )

  install(DIRECTORY include/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  )

  install(DIRECTORY launch/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  )

elseif(BUILD_ROS2)
  find_package(ament_cmake REQUIRED)
  find_package(rclcpp REQUIRED)
  find_package(sensor_msgs REQUIRED)
  find_package(std_msgs REQUIRED)

  target_compile_definitions(pointcloud2_inspector_core PUBLIC POINTCLOUD2_INSPECTOR_ROS2)
  ament_target_dependencies(pointcloud2_inspector_core rclcpp sensor_msgs std_msgs)

  add_executable(pointcloud2_inspector_node src/main_ros2.cpp)
  target_compile_definitions(pointcloud2_inspector_node PRIVATE POINTCLOUD2_INSPECTOR_ROS2)
  target_link_libraries(pointcloud2_inspector_node PRIVATE pointcloud2_inspector_core)
  ament_target_dependencies(pointcloud2_inspector_node rclcpp sensor_msgs std_msgs)

  install(TARGETS
    pointcloud2_inspector_core
    pointcloud2_inspector_node
    DESTINATION lib/${PROJECT_NAME}
  )

  install(DIRECTORY include/
    DESTINATION include
  )

  install(DIRECTORY launch/
    DESTINATION share/${PROJECT_NAME}/launch
  )

  ament_export_include_directories(include)
  ament_export_libraries(pointcloud2_inspector_core)
  ament_export_dependencies(rclcpp sensor_msgs std_msgs)

  ament_package()
endif()
