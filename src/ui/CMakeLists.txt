cmake_minimum_required(VERSION 3.5)
project(auto_store_ui)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++20
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif()

# Note: Compilation flag -fPIC is necessary. Otherwise, you'll receive error "You must build your code with position independent code if Qt was built with -reduce-relocations."
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -fPIC)
endif()

set(THIS_PACKAGE_INCLUDE_DEPENDS
  ament_cmake
  rclcpp
  robotics_configs
  controller_manager
  controller_manager_msgs
)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

find_package(Qt5 COMPONENTS Widgets REQUIRED)
# Note that ament_target_dependencies() in algo cannot export Boost::thread to downstream. As a result, qt_example_node, as a downstream of libalgo.so, needs to explicitly find_package() for Boost again so that qt_example_node is able to link to Boost::thread.
find_package(Boost 1.71.0 REQUIRED COMPONENTS thread)
set(CMAKE_INCLUDE_CURRENT_DIR ON)
file(GLOB_RECURSE PRO_FORM_DIR RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} *.ui)
file(GLOB_RECURSE PRO_RESOURCES_DIR RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} resource*.qrc)
file(GLOB_RECURSE PRO_INCLUDE_DIR RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS *.hpp *.h )
file(GLOB_RECURSE PRO_SOURCES_DIR RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS *.cpp)
#搜索头文件的位置
include_directories(
  control_pad/move_button/
  control_pad/joint_widget/
  control_pad/joint_group_widget/
  control_pad/setting_panel/
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
#qt头文件moc文件resource文件处理
qt5_wrap_ui(QT_UI_HPP ${PRO_FORM_DIR})
qt5_wrap_cpp(QT_MOC_HPP ${PRO_INCLUDE_DIR})
qt5_add_resources(QT_RESOURCES_CPP ${PRO_RESOURCES_DIR})
add_executable(${PROJECT_NAME}
  main.cpp
  ${PRO_SOURCES_DIR}
  ${QT_RESOURCES_CPP}
  ${PRO_INCLUDE_DIR}
  ${QT_MOC_HPP}
  ${PRO_FORM_DIR}
)
# Note that ament_target_dependencies() in algo cannot export Boost::thread to downstream. As a result, qt_example_node, as a downstream of libalgo.so, needs to explicitly ament_target_dependencies() for Boost again so that qt_example_node is able to link to Boost::thread.
ament_target_dependencies(${PROJECT_NAME} ${THIS_PACKAGE_INCLUDE_DEPENDS} Boost)

target_link_libraries(${PROJECT_NAME}
  Qt5::Widgets
)

install(TARGETS ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
