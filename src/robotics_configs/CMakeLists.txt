cmake_minimum_required(VERSION 3.8)
project(robotics_configs)
set(CMAKE_CXX_STANDARD 20)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
set(THIS_PACKAGE_INCLUDE_DEPENDS
  rclcpp
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
  controller_manager
  controller_manager_msgs
  std_msgs
)

# find dependencies
find_package(ament_cmake REQUIRED)

foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

add_library(${PROJECT_NAME}
  SHARED
  ${PROJECT_SOURCE_DIR}/move_executor/move_executor.cpp
  ${PROJECT_SOURCE_DIR}/controller_switcher/controller_switcher.cpp
  ${PROJECT_SOURCE_DIR}/mode_changer/mode_changer.cpp
)

ament_target_dependencies( ${PROJECT_NAME} ${THIS_PACKAGE_INCLUDE_DEPENDS})
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/move_executor>
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/controller_switcher>
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/mode_changer>
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/robot_description>
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/utils>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)

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
