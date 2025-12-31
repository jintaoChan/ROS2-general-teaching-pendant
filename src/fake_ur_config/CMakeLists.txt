cmake_minimum_required(VERSION 3.22)
project(fake_ur_config)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ethercat_driver REQUIRED)
find_package(ethercat_interface REQUIRED)
find_package(ethercat_wait_needed_cia402_drive REQUIRED)
ament_package()

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/launch")
  install(
    DIRECTORY launch driver_config meshes config worlds
    DESTINATION share/${PROJECT_NAME}
    PATTERN "setup_assistant.launch" EXCLUDE)
endif()


