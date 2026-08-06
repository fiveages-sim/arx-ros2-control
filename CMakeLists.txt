cmake_minimum_required(VERSION 3.16)
project(arx_ros2_control LANGUAGES CXX)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra -Werror=return-type)
endif()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(std_msgs REQUIRED)

find_package(Eigen3 REQUIRED)
find_package(orocos_kdl REQUIRED)
find_package(kdl_parser REQUIRED)
find_package(spdlog REQUIRED)

set(ARX5_SDK_DIR ${CMAKE_CURRENT_SOURCE_DIR}/external/arx5-sdk)
set(ARX_LIFT_DIR ${CMAKE_CURRENT_SOURCE_DIR}/external/arx_lift_src)
add_compile_definitions(SDK_ROOT="${ARX5_SDK_DIR}")

if(CMAKE_SYSTEM_PROCESSOR MATCHES "arm.*|arm|aarch64")
  set(ARX_ARCH_DIR aarch64)
else()
  set(ARX_ARCH_DIR x86_64)
endif()
set(ARX5_LIB_DIR ${ARX5_SDK_DIR}/lib/${ARX_ARCH_DIR})
set(ARX_LIFT_LIB ${ARX_LIFT_DIR}/lib/${ARX_ARCH_DIR}/libarx_lift_src.so)

include_directories(
  include
  ${ARX5_SDK_DIR}/include
  ${ARX_LIFT_DIR}/include
  ${EIGEN3_INCLUDE_DIRS}
  ${orocos_kdl_INCLUDE_DIRS}
  /opt/ros/$ENV{ROS_DISTRO}/include/kdl_parser
)

# Prefer existing ArxJointController; else build from SDK joint source.
set(ARX5_JOINT_CTRL "")
if(EXISTS "${ARX5_SDK_DIR}/build/libArxJointController.so")
  set(ARX5_JOINT_CTRL "${ARX5_SDK_DIR}/build/libArxJointController.so")
elseif(EXISTS "${CMAKE_CURRENT_BINARY_DIR}/libArxJointController.so")
  set(ARX5_JOINT_CTRL "${CMAKE_CURRENT_BINARY_DIR}/libArxJointController.so")
endif()

if(NOT ARX5_JOINT_CTRL)
  add_library(ArxJointController SHARED
    ${ARX5_SDK_DIR}/src/app/joint_controller.cpp
  )
  target_include_directories(ArxJointController PUBLIC
    ${ARX5_SDK_DIR}/include
    ${EIGEN3_INCLUDE_DIRS}
  )
  target_link_libraries(ArxJointController PUBLIC
    ${ARX5_LIB_DIR}/libhardware.so
    ${ARX5_LIB_DIR}/libsolver.so
  )
  set(ARX5_JOINT_CTRL ArxJointController)
endif()

add_library(${PROJECT_NAME} SHARED
  src/arx_x5_hardware.cpp
  src/arx_lift_hardware.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)

ament_target_dependencies(${PROJECT_NAME} PUBLIC
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  std_msgs
)

target_link_libraries(${PROJECT_NAME} PUBLIC
  ${ARX5_JOINT_CTRL}
  ${ARX5_LIB_DIR}/libhardware.so
  ${ARX5_LIB_DIR}/libsolver.so
  ${ARX_LIFT_LIB}
)

pluginlib_export_plugin_description_file(hardware_interface arx_ros2_control.xml)

set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)
set(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib")

install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  FILES
    ${ARX5_LIB_DIR}/libhardware.so
    ${ARX5_LIB_DIR}/libsolver.so
    ${ARX_LIFT_LIB}
  DESTINATION lib
)

if(TARGET ArxJointController)
  install(TARGETS ArxJointController LIBRARY DESTINATION lib)
elseif(EXISTS "${ARX5_JOINT_CTRL}")
  install(FILES ${ARX5_JOINT_CTRL} DESTINATION lib)
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  std_msgs
)

ament_package()
