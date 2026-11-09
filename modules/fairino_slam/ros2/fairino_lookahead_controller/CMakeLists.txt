cmake_minimum_required(VERSION 3.16)
project(fairino_lookahead_controller)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
find_package(ament_cmake REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
set(dependencies nav2_core nav2_costmap_2d nav2_util rclcpp rclcpp_lifecycle
  pluginlib geometry_msgs nav_msgs diagnostic_msgs tf2 tf2_ros tf2_geometry_msgs)
foreach(dependency IN LISTS dependencies)
  find_package(${dependency} REQUIRED)
endforeach()

add_library(${PROJECT_NAME} SHARED src/controller.cpp)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(${PROJECT_NAME} ${dependencies})
target_compile_options(${PROJECT_NAME} PRIVATE -Wall -Wextra -Wpedantic)
pluginlib_export_plugin_description_file(nav2_core plugin.xml)

add_library(fairino_replan_bt_node SHARED src/replan_bt.cpp)
target_include_directories(fairino_replan_bt_node PRIVATE include)
ament_target_dependencies(fairino_replan_bt_node behaviortree_cpp_v3 rclcpp diagnostic_msgs geometry_msgs nav2_util tf2_ros)
target_compile_definitions(fairino_replan_bt_node PRIVATE BT_PLUGIN_EXPORT)
install(TARGETS fairino_replan_bt_node LIBRARY DESTINATION lib)

add_executable(plugin_check src/plugin_check.cpp)
ament_target_dependencies(plugin_check nav2_core pluginlib behaviortree_cpp_v3)
install(TARGETS plugin_check DESTINATION lib/${PROJECT_NAME})

install(TARGETS ${PROJECT_NAME} EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)

if(BUILD_TESTING)
  enable_testing()
  foreach(test_name core collision coordination)
    add_executable(test_${test_name} test/test_${test_name}.cpp)
    target_include_directories(test_${test_name} PRIVATE include)
    add_test(NAME ${test_name} COMMAND test_${test_name})
  endforeach()
  add_executable(test_planning_retry ../../tests/navigation/test_planning_retry.cpp)
  target_include_directories(test_planning_retry PRIVATE include)
  ament_target_dependencies(test_planning_retry behaviortree_cpp_v3)
  add_test(NAME planning_retry COMMAND test_planning_retry
    ${CMAKE_CURRENT_SOURCE_DIR}/../../config/behavior_trees/navigate_to_pose.xml
    ${CMAKE_CURRENT_SOURCE_DIR}/../../config/behavior_trees/navigate_through_poses.xml
    ${CMAKE_CURRENT_SOURCE_DIR}/../../config/behavior_trees/navigate_to_saved_pose.xml)
  add_executable(test_saved_pose_bt ../../tests/navigation/test_saved_pose_bt.cpp)
  ament_target_dependencies(test_saved_pose_bt behaviortree_cpp_v3 rclcpp geometry_msgs tf2_ros)
  add_test(NAME saved_pose_bt COMMAND test_saved_pose_bt
    $<TARGET_FILE:fairino_replan_bt_node>
    ${CMAKE_CURRENT_SOURCE_DIR}/../../config/behavior_trees/navigate_to_saved_pose.xml)
  set_tests_properties(saved_pose_bt PROPERTIES ENVIRONMENT "ROS_DOMAIN_ID=87;ROS_LOCALHOST_ONLY=1")
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_plugin test/test_plugin.cpp)
  if(TARGET test_plugin)
    target_link_libraries(test_plugin ${PROJECT_NAME})
    ament_target_dependencies(test_plugin ${dependencies})
  endif()
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${dependencies})
ament_package()
