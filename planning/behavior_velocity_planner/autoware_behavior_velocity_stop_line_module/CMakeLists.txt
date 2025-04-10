cmake_minimum_required(VERSION 3.14)
project(autoware_behavior_velocity_stop_line_module)

find_package(autoware_cmake REQUIRED)
autoware_package()
pluginlib_export_plugin_description_file(autoware_behavior_velocity_planner plugins.xml)

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/debug.cpp
  src/manager.cpp
  src/scene.cpp
)

if(BUILD_TESTING)
  ament_add_ros_isolated_gtest(test_${PROJECT_NAME}
    test/test_scene.cpp
    test/test_node_interface.cpp
  )
  target_link_libraries(test_${PROJECT_NAME}
    gtest_main
    ${PROJECT_NAME}
  )
  target_include_directories(test_${PROJECT_NAME} PRIVATE src)
endif()

ament_auto_package(INSTALL_TO_SHARE config)
