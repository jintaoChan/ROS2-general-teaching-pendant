cmake_minimum_required(VERSION 3.5)
project(auto_store_ui)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)

# Note: Compilation flag -fPIC is necessary. Otherwise, you'll receive error "You must build your code with position independent code if Qt was built with -reduce-relocations."
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wbuiltin-macro-redefined -Wpedantic -fPIC)
endif()

set(THIS_PACKAGE_INCLUDE_DEPENDS
  ament_cmake
  rclcpp
  magic_enum
  control_pad_controller
  controller_manager
  controller_manager_msgs
  trac_ik_lib
  tf2
  tf2_ros
  pinocchio
)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()
find_package(JKQTPlotter6 REQUIRED)
find_package(Qt6 COMPONENTS Widgets Charts OpenGLWidgets REQUIRED)
set(CMAKE_INCLUDE_CURRENT_DIR ON)
file(GLOB_RECURSE PRO_FORM_DIR RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} *.ui)
file(GLOB_RECURSE PRO_RESOURCES_DIR RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} resource*.qrc)
file(GLOB_RECURSE PRO_INCLUDE_DIR RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS *.hpp *.h )
file(GLOB_RECURSE PRO_SOURCES_DIR RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS *.cpp)
include_directories(
  plot_page/
  plot_page/plot_widget/
  setting_panel/
  setting_page/
  setting_page/param_identification/
  setting_page/param_identification/simulate_external_force_bar/
  setting_page/tcp/
  setting_page/tcp/tool_frame_widget/
  control_pad/move_button/
  control_pad/joint_widget/
  control_pad/joint_group_widget/
  control_pad/cartesian_dof_row/
  control_pad/cartesian_pad/
  control_pad/control_pad/
  programmer/group_info_dialog/
  programmer/task_widget/
  programmer/point_table/
  programmer/point_pool/
  programmer/task_list/
  programmer/task_detail/
  programmer/treeview_with_key_event/
  programmer/task_widget/
  programmer/utils/
  main_window/
  ${CMAKE_CURRENT_BINARY_DIR}
)


add_executable(${PROJECT_NAME}
  main.cpp
  ${PRO_SOURCES_DIR}
  ${QT_RESOURCES_CPP}
  ${PRO_INCLUDE_DIR}
  ${QT_MOC_HPP}
  ${QT_UI_HPP}
)
ament_target_dependencies(${PROJECT_NAME} ${THIS_PACKAGE_INCLUDE_DEPENDS})
target_include_directories(${PROJECT_NAME}
  PRIVATE
  cereal
)
target_link_libraries(${PROJECT_NAME}
  magic_enum::magic_enum
  Qt6::Widgets
  Qt6::Charts
  Qt6::OpenGLWidgets
  JKQTPlotter6::JKQTPlotter6
)

install(TARGETS ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
