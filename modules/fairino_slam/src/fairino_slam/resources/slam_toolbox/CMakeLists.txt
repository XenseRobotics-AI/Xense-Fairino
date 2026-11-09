cmake_minimum_required(VERSION 3.16)
project(fairino_mapping_runtime LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
set(CMAKE_INSTALL_RPATH "$ORIGIN/../lib;$ORIGIN;/opt/ros/humble/lib")
find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED COMPONENTS system serialization filesystem thread)
set(dependencies rclcpp message_filters nav_msgs sensor_msgs geometry_msgs tf2 tf2_ros
  visualization_msgs pluginlib tf2_geometry_msgs tf2_sensor_msgs std_msgs std_srvs
  builtin_interfaces interactive_markers rclcpp_components)
foreach(dependency IN LISTS dependencies)
  find_package(${dependency} REQUIRED)
endforeach()
# Reuse the installed, version-checked 2.6.10 SDK and interfaces. Only our
# mapping executable links these privately named frontend libraries.
find_path(SLAM_HEADERS slam_toolbox/slam_toolbox_common.hpp
  PATHS /opt/ros/humble/include REQUIRED NO_DEFAULT_PATH)
find_path(SLAM_MESSAGES slam_toolbox/srv/pause.hpp
  PATHS /opt/ros/humble/include/slam_toolbox /opt/ros/humble/include REQUIRED NO_DEFAULT_PATH)
find_library(KARTO_LIBRARY kartoSlamToolbox PATHS /opt/ros/humble/lib REQUIRED NO_DEFAULT_PATH)
find_library(SLAM_TYPESUPPORT slam_toolbox__rosidl_typesupport_cpp
  PATHS /opt/ros/humble/lib REQUIRED NO_DEFAULT_PATH)
include_directories(${SLAM_HEADERS} ${SLAM_MESSAGES} ${EIGEN3_INCLUDE_DIRS})
add_library(fairino_toolbox_common SHARED src/slam_toolbox_common.cpp src/map_saver.cpp
  src/loop_closure_assistant.cpp src/laser_utils.cpp src/slam_mapper.cpp)
ament_target_dependencies(fairino_toolbox_common ${dependencies})
target_link_libraries(fairino_toolbox_common ${KARTO_LIBRARY} ${Boost_LIBRARIES} ${SLAM_TYPESUPPORT})
add_library(fairino_sync_slam_toolbox SHARED src/slam_toolbox_sync.cpp)
ament_target_dependencies(fairino_sync_slam_toolbox ${dependencies})
target_link_libraries(fairino_sync_slam_toolbox fairino_toolbox_common ${KARTO_LIBRARY}
  ${Boost_LIBRARIES} ${SLAM_TYPESUPPORT})
add_executable(sync_slam_toolbox_node src/slam_toolbox_sync_node.cpp)
ament_target_dependencies(sync_slam_toolbox_node ${dependencies})
target_link_libraries(sync_slam_toolbox_node fairino_sync_slam_toolbox fairino_toolbox_common)
install(TARGETS fairino_toolbox_common fairino_sync_slam_toolbox sync_slam_toolbox_node
  LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
install(FILES LICENSE DESTINATION share/fairino_mapping_runtime)
