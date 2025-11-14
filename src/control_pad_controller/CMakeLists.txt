cmake_minimum_required(VERSION 3.8)
project(control_pad_controller)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -fPIC)
endif()

# find dependencies
set(THIS_PACKAGE_INCLUDE_DEPENDS
  hardware_interface
  controller_interface
  pluginlib
  rclcpp
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
  rclcpp_lifecycle
  controller_manager
  controller_manager_msgs
  std_msgs
  trajectory_msgs
  sensor_msgs
  trac_ik
  ament_cmake
  backward_ros
  geometry_msgs
  magic_enum
  pinocchio
)


foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()
find_package(pinocchio REQUIRED)
find_package(Qt6 COMPONENTS Widgets Charts REQUIRED)


# ros2_control plugin library
add_library(${PROJECT_NAME}
  SHARED
  ${CMAKE_CURRENT_SOURCE_DIR}/src/robot_handle/robot_handle.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/database/database.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/control_pad_controller.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/move_executor/move_executor.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/controller_switcher/controller_switcher.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/kinematics_plugin/kinematics_plugin.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/dynamic_plugin/dynamic_plugin.cpp
)


target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/src/move_executor>
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/src/controller_switcher>
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/src/kinematics_plugin>
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/src/dynamic_plugin>
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/src/robot_handle>
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/src/database>
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/src/utils>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
ament_target_dependencies( ${PROJECT_NAME}  ${THIS_PACKAGE_INCLUDE_DEPENDS})

target_link_libraries(${PROJECT_NAME}
  magic_enum::magic_enum
  Qt6::Widgets
  Qt6::Charts
)

# Export hardware plugins
pluginlib_export_plugin_description_file(controller_interface control_pad.xml)

install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

file(GLOB_RECURSE HEADER_FILES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS *.h *.hpp)

install(
  FILES ${HEADER_FILES}
  DESTINATION include/${PROJECT_NAME}
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
