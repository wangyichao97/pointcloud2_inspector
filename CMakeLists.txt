cmake_minimum_required(VERSION 3.10)
project(lidar_tools)

option(BUILD_ROS1 "Build lidar_tools for ROS1 (Noetic)" OFF)
option(BUILD_ROS2 "Build lidar_tools for ROS2 (Foxy/Humble)" OFF)

if(BUILD_ROS1 AND BUILD_ROS2)
  message(FATAL_ERROR "Invalid configuration: BUILD_ROS1 and BUILD_ROS2 cannot both be ON.")
elseif(NOT BUILD_ROS1 AND NOT BUILD_ROS2)
  message(FATAL_ERROR "Invalid configuration: exactly one of BUILD_ROS1 or BUILD_ROS2 must be ON.")
endif()

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(PCL REQUIRED COMPONENTS common io)

add_library(lidar_tools_common
  src/common/pointcloud_utils.cpp
  src/common/preprocess_utils.cpp
)
target_include_directories(lidar_tools_common PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${PCL_INCLUDE_DIRS}
)
target_link_libraries(lidar_tools_common PUBLIC ${PCL_LIBRARIES})

add_library(lidar_tools_inspector
  src/pointcloud2_inspector/pointcloud2_inspector.cpp
)
target_include_directories(lidar_tools_inspector PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(lidar_tools_inspector PUBLIC lidar_tools_common)

add_library(lidar_tools_preprocess
  src/preprocess/lidar_preprocess_tester.cpp
)
target_include_directories(lidar_tools_preprocess PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${PCL_INCLUDE_DIRS}
)
target_link_libraries(lidar_tools_preprocess PUBLIC lidar_tools_common ${PCL_LIBRARIES})

if(BUILD_ROS1)
  find_package(catkin REQUIRED COMPONENTS roscpp sensor_msgs std_msgs pcl_ros pcl_conversions)

  target_compile_definitions(lidar_tools_common PUBLIC LIDAR_TOOLS_ROS1)
  target_compile_definitions(lidar_tools_inspector PUBLIC LIDAR_TOOLS_ROS1)
  target_compile_definitions(lidar_tools_preprocess PUBLIC LIDAR_TOOLS_ROS1)

  catkin_package(
    INCLUDE_DIRS include
    LIBRARIES lidar_tools_common lidar_tools_inspector lidar_tools_preprocess
    CATKIN_DEPENDS roscpp sensor_msgs std_msgs pcl_ros pcl_conversions
  )

  target_include_directories(lidar_tools_common PUBLIC ${catkin_INCLUDE_DIRS})
  target_include_directories(lidar_tools_inspector PUBLIC ${catkin_INCLUDE_DIRS})
  target_include_directories(lidar_tools_preprocess PUBLIC ${catkin_INCLUDE_DIRS})

  target_link_libraries(lidar_tools_common PUBLIC ${catkin_LIBRARIES})
  target_link_libraries(lidar_tools_inspector PUBLIC ${catkin_LIBRARIES})
  target_link_libraries(lidar_tools_preprocess PUBLIC ${catkin_LIBRARIES})

  add_executable(pointcloud2_inspector_node src/pointcloud2_inspector/node_ros1.cpp)
  add_executable(lidar_preprocess_tester_node src/preprocess/node_ros1.cpp)
  target_link_libraries(pointcloud2_inspector_node PRIVATE lidar_tools_inspector)
  target_link_libraries(lidar_preprocess_tester_node PRIVATE lidar_tools_preprocess)

  install(TARGETS
    lidar_tools_common lidar_tools_inspector lidar_tools_preprocess
    pointcloud2_inspector_node lidar_preprocess_tester_node
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )

  install(DIRECTORY include/ DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
  install(DIRECTORY launch/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)
  install(DIRECTORY config/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config)
  install(DIRECTORY docs/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/docs)

elseif(BUILD_ROS2)
  find_package(ament_cmake REQUIRED)
  find_package(rclcpp REQUIRED)
  find_package(sensor_msgs REQUIRED)
  find_package(std_msgs REQUIRED)
  find_package(pcl_conversions REQUIRED)
  find_package(pcl_msgs REQUIRED)

  target_compile_definitions(lidar_tools_common PUBLIC LIDAR_TOOLS_ROS2)
  target_compile_definitions(lidar_tools_inspector PUBLIC LIDAR_TOOLS_ROS2)
  target_compile_definitions(lidar_tools_preprocess PUBLIC LIDAR_TOOLS_ROS2)

  ament_target_dependencies(lidar_tools_common rclcpp sensor_msgs std_msgs pcl_conversions pcl_msgs)
  ament_target_dependencies(lidar_tools_inspector rclcpp sensor_msgs std_msgs pcl_conversions pcl_msgs)
  ament_target_dependencies(lidar_tools_preprocess rclcpp sensor_msgs std_msgs pcl_conversions pcl_msgs)

  add_executable(pointcloud2_inspector_node src/pointcloud2_inspector/node_ros2.cpp)
  add_executable(lidar_preprocess_tester_node src/preprocess/node_ros2.cpp)
  target_link_libraries(pointcloud2_inspector_node PRIVATE lidar_tools_inspector)
  target_link_libraries(lidar_preprocess_tester_node PRIVATE lidar_tools_preprocess)
  ament_target_dependencies(pointcloud2_inspector_node rclcpp sensor_msgs std_msgs pcl_conversions pcl_msgs)
  ament_target_dependencies(lidar_preprocess_tester_node rclcpp sensor_msgs std_msgs pcl_conversions pcl_msgs)

  install(TARGETS
    lidar_tools_common lidar_tools_inspector lidar_tools_preprocess
    pointcloud2_inspector_node lidar_preprocess_tester_node
    DESTINATION lib/${PROJECT_NAME}
  )

  install(DIRECTORY include/ DESTINATION include)
  install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch)
  install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config)
  install(DIRECTORY docs/ DESTINATION share/${PROJECT_NAME}/docs)

  ament_export_include_directories(include)
  ament_export_libraries(lidar_tools_common lidar_tools_inspector lidar_tools_preprocess)
  ament_export_dependencies(rclcpp sensor_msgs std_msgs pcl_conversions pcl_msgs)
  ament_package()
endif()
