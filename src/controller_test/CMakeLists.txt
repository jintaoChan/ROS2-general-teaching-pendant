cmake_minimum_required(VERSION 3.8)
project(controller_test)
set(CMAKE_CXX_STANDARD 20)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
set(THIS_PACKAGE_INCLUDE_DEPENDS
  rclcpp
  control_pad_controller
  trac_ik_lib
  pinocchio
)


foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()
find_package(Qt6 COMPONENTS Widgets Charts REQUIRED)
find_package(pinocchio REQUIRED)


# ros2_control plugin library
add_executable(${PROJECT_NAME}
  main.cpp
)
ament_target_dependencies(${PROJECT_NAME} ${THIS_PACKAGE_INCLUDE_DEPENDS})

target_link_libraries(${PROJECT_NAME}
  magic_enum::magic_enum
  Qt6::Widgets
  Qt6::Charts
)

install(TARGETS ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
)
ament_package()
