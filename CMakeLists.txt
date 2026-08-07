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

# arx_lift_src ships headers + prebuilt .so only (no sources). x86_64 is vendored;
# aarch64 is not in tree history — skip Lift HI when the arch .so is missing so
# arm builds can still produce ArxX5Hardware.
set(ARX_HAS_LIFT OFF)
if(EXISTS "${ARX_LIFT_LIB}")
  set(ARX_HAS_LIFT ON)
  message(STATUS "ArxLiftHardware enabled: ${ARX_LIFT_LIB}")
else()
  message(WARNING
    "libarx_lift_src.so not found for ${ARX_ARCH_DIR} "
    "(expected ${ARX_LIFT_LIB}). Skipping ArxLiftHardware compile/link/install; "
    "arm/x86 arm-only builds get ArxX5Hardware only. Place the arch .so under "
    "external/arx_lift_src/lib/${ARX_ARCH_DIR}/ to enable Lift2S.")
endif()

include_directories(
  include
  ${ARX5_SDK_DIR}/include
  ${ARX_LIFT_DIR}/include
  ${EIGEN3_INCLUDE_DIRS}
  ${orocos_kdl_INCLUDE_DIRS}
  /opt/ros/$ENV{ROS_DISTRO}/include/kdl_parser
)

# Prefer existing ArxJointController; else build from SDK joint source.
# Must not PUBLIC-link a non-exported CMake target into export_${PROJECT_NAME}.
set(ARX5_JOINT_CTRL "")
if(EXISTS "${ARX5_SDK_DIR}/build/libArxJointController.so")
  set(ARX5_JOINT_CTRL "${ARX5_SDK_DIR}/build/libArxJointController.so")
elseif(EXISTS "${CMAKE_CURRENT_BINARY_DIR}/libArxJointController.so")
  set(ARX5_JOINT_CTRL "${CMAKE_CURRENT_BINARY_DIR}/libArxJointController.so")
endif()

if(NOT ARX5_JOINT_CTRL)
  # joint_controller depends on controller_base + utils (not in prebuilt libhardware.so).
  # Building only joint_controller.cpp yields exit 127 at HI load:
  #   undefined symbol: arx::Arx5ControllerBase::Arx5ControllerBase(...)
  add_library(ArxJointController SHARED
    ${ARX5_SDK_DIR}/src/app/joint_controller.cpp
    ${ARX5_SDK_DIR}/src/app/controller_base.cpp
    ${ARX5_SDK_DIR}/src/utils.cpp
  )
  target_include_directories(ArxJointController PRIVATE
    ${ARX5_SDK_DIR}/include
    ${EIGEN3_INCLUDE_DIRS}
  )
  target_link_libraries(ArxJointController PRIVATE
    ${ARX5_LIB_DIR}/libhardware.so
    ${ARX5_LIB_DIR}/libsolver.so
    spdlog::spdlog
  )
  # Vendored Stanford SDK is not -Werror clean (return-type / reorder / unused).
  target_compile_options(ArxJointController PRIVATE
    -Wno-error=return-type
    -Wno-unused-variable
    -Wno-reorder
    -Wno-sign-compare
  )
  set_target_properties(ArxJointController PROPERTIES
    BUILD_WITH_INSTALL_RPATH FALSE
    INSTALL_RPATH "$ORIGIN"
  )
  set(ARX5_JOINT_CTRL ArxJointController)
endif()

set(ARX_HI_SOURCES src/arx_x5_hardware.cpp)
if(ARX_HAS_LIFT)
  list(APPEND ARX_HI_SOURCES src/arx_lift_hardware.cpp)
endif()

add_library(${PROJECT_NAME} SHARED ${ARX_HI_SOURCES})

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

# PRIVATE: HI is loaded via pluginlib; do not re-export SDK / joint .so into ament export.
set(ARX_HI_LINK_LIBS
  ${ARX5_JOINT_CTRL}
  ${ARX5_LIB_DIR}/libhardware.so
  ${ARX5_LIB_DIR}/libsolver.so
)
if(ARX_HAS_LIFT)
  list(APPEND ARX_HI_LINK_LIBS ${ARX_LIFT_LIB})
endif()
target_link_libraries(${PROJECT_NAME} PRIVATE ${ARX_HI_LINK_LIBS})

if(ARX_HAS_LIFT)
  pluginlib_export_plugin_description_file(hardware_interface arx_ros2_control.xml)
else()
  pluginlib_export_plugin_description_file(hardware_interface arx_ros2_control_no_lift.xml)
endif()

# Stanford libhardware.so needs SOEM 1.4.x at runtime (conda-forge::soem=1.4.0).
# Prebuilt RPATH points at a foreign conda env; install a local copy into lib/.
set(ARX5_COMPATIBLE_SOEM_LIB "")
set(ARX5_SOEM_CANDIDATES
  "$ENV{CONDA_PREFIX}/lib/libsoem.so"
  "$ENV{HOME}/miniconda3/envs/arx-py312/lib/libsoem.so"
  "$ENV{HOME}/mambaforge/envs/arx-py312/lib/libsoem.so"
  "$ENV{HOME}/miniforge3/envs/arx-py312/lib/libsoem.so"
  "${CMAKE_CURRENT_SOURCE_DIR}/external/SOEM/lib/${ARX_ARCH_DIR}/libsoem.so"
)
foreach(candidate IN LISTS ARX5_SOEM_CANDIDATES)
  if(candidate AND EXISTS "${candidate}")
    set(ARX5_COMPATIBLE_SOEM_LIB "${candidate}")
    break()
  endif()
endforeach()
if(ARX5_COMPATIBLE_SOEM_LIB)
  message(STATUS "Using compatible SOEM runtime library: ${ARX5_COMPATIBLE_SOEM_LIB}")
elseif(ARX_ARCH_DIR STREQUAL "aarch64")
  # Vendored aarch64 libhardware.so statically embeds SOEM; no separate libsoem needed.
  message(STATUS "Skipping SOEM for aarch64 (statically linked into libhardware.so)")
else()
  message(WARNING
    "Compatible libsoem.so not found. Real hardware will fail to load "
    "(dlopen: libsoem.so). Install: conda install -n arx-py312 conda-forge::soem=1.4.0 "
    "or place libsoem.so under external/SOEM/lib/${ARX_ARCH_DIR}/")
endif()

# Ensure plugin + SDK libs resolve next to each other at runtime
set_target_properties(${PROJECT_NAME} PROPERTIES
  BUILD_WITH_INSTALL_RPATH FALSE
  INSTALL_RPATH "$ORIGIN"
  INSTALL_RPATH_USE_LINK_PATH TRUE
)

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

set(ARX_INSTALL_VENDOR_LIBS
  ${ARX5_LIB_DIR}/libhardware.so
  ${ARX5_LIB_DIR}/libsolver.so
)
if(ARX_HAS_LIFT)
  list(APPEND ARX_INSTALL_VENDOR_LIBS ${ARX_LIFT_LIB})
endif()
install(
  FILES ${ARX_INSTALL_VENDOR_LIBS}
  DESTINATION lib
)

if(TARGET ArxJointController)
  # Same export set as ${PROJECT_NAME}: required when linked (even PRIVATE under ament).
  install(
    TARGETS ArxJointController
    EXPORT export_${PROJECT_NAME}
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
  )
elseif(EXISTS "${ARX5_JOINT_CTRL}")
  install(FILES ${ARX5_JOINT_CTRL} DESTINATION lib)
endif()

if(ARX5_COMPATIBLE_SOEM_LIB)
  install(
    FILES ${ARX5_COMPATIBLE_SOEM_LIB}
    DESTINATION lib
  )
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
