# Copyright 2025 vlai.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

cmake_minimum_required(VERSION 3.22)
project(xarm_hardware)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(kdl_parser REQUIRED)
find_package(urdf REQUIRED)

set(XARM_SDK_ROOT "$ENV{XARM_SDK_ROOT}" CACHE PATH
    "Path to SDK package root (contains include/ and lib/)")
if(NOT XARM_SDK_ROOT)
  get_filename_component(_pkg_dir "${CMAKE_CURRENT_SOURCE_DIR}" ABSOLUTE)
  set(XARM_SDK_ROOT "${_pkg_dir}/../../../../xarm_can/package")
endif()

set(XARM_HARDWARE_CORE_ROOT "$ENV{XARM_HARDWARE_CORE_ROOT}" CACHE PATH
    "Optional protected xarm_hardware core root (contains include/ and src/)")
if(NOT XARM_HARDWARE_CORE_ROOT)
  set(_default_protected_core
      "${CMAKE_CURRENT_SOURCE_DIR}/../../../../../internal_protected/xarm_ros2/xarm_hardware")
  if(EXISTS "${_default_protected_core}/src/v10_simple_hardware.cpp")
    set(XARM_HARDWARE_CORE_ROOT "${_default_protected_core}")
  endif()
endif()

set(XARM_SDK_INCLUDE_DIR "${XARM_SDK_ROOT}/include")
set(XARM_SDK_LIBRARY "${XARM_SDK_ROOT}/lib/${CMAKE_SYSTEM_PROCESSOR}/libxarm_can_sdk.so")

if(EXISTS "${XARM_HARDWARE_CORE_ROOT}/src/v10_simple_hardware.cpp")
  set(XARM_HARDWARE_CORE_SRC "${XARM_HARDWARE_CORE_ROOT}/src/v10_simple_hardware.cpp")
  set(XARM_HARDWARE_CORE_INCLUDE_DIR "${XARM_HARDWARE_CORE_ROOT}/include")
  message(STATUS "xarm_hardware: using protected core at ${XARM_HARDWARE_CORE_ROOT}")
else()
  set(XARM_HARDWARE_CORE_SRC "${CMAKE_CURRENT_SOURCE_DIR}/src/v10_simple_hardware.cpp")
  set(XARM_HARDWARE_CORE_INCLUDE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/include")
  message(STATUS "xarm_hardware: using local source core")
endif()

if(NOT EXISTS "${XARM_HARDWARE_CORE_SRC}")
  message(FATAL_ERROR "xarm_hardware core source not found: ${XARM_HARDWARE_CORE_SRC}")
endif()

if(NOT EXISTS "${XARM_SDK_INCLUDE_DIR}/xarm_can_sdk.h")
  message(FATAL_ERROR "xarm_can_sdk.h not found under XARM_SDK_ROOT: ${XARM_SDK_INCLUDE_DIR}")
endif()

if(NOT EXISTS "${XARM_SDK_LIBRARY}")
  message(FATAL_ERROR "SDK shared library not found: ${XARM_SDK_LIBRARY}")
endif()

add_library(${PROJECT_NAME} SHARED
  ${XARM_HARDWARE_CORE_SRC}
)

target_include_directories(
  ${PROJECT_NAME}
  PRIVATE
  include
  ${XARM_HARDWARE_CORE_INCLUDE_DIR}
  ${XARM_SDK_INCLUDE_DIR}
)

target_link_libraries(${PROJECT_NAME}
  ${XARM_SDK_LIBRARY}
)

ament_target_dependencies(${PROJECT_NAME}
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  kdl_parser
  urdf
)

pluginlib_export_plugin_description_file(hardware_interface xarm_hardware.xml)

install(
  TARGETS ${PROJECT_NAME}
  DESTINATION lib
  )

install(DIRECTORY include/
DESTINATION include
)

ament_export_include_directories(
  include
)

ament_export_libraries(
  ${PROJECT_NAME}
)
ament_export_dependencies(
  hardware_interface
  pluginlib
  rclcpp
)

if(BUILD_TESTING)
  enable_testing()
  add_executable(command_mode_switch_test test/command_mode_switch_test.cpp)
  target_include_directories(command_mode_switch_test PRIVATE include)
  add_test(NAME command_mode_switch_test COMMAND command_mode_switch_test)
  set_tests_properties(command_mode_switch_test PROPERTIES TIMEOUT 15)
  foreach(test_name IN ITEMS feedback_timing_test startup_hold_test urdf_position_limits_test)
    add_executable(${test_name} test/${test_name}.cpp)
    target_include_directories(${test_name} PRIVATE include ${XARM_SDK_INCLUDE_DIR})
    target_link_libraries(${test_name} ${PROJECT_NAME})
    ament_target_dependencies(${test_name} hardware_interface rclcpp rclcpp_lifecycle kdl_parser)
    # Interpose SDK calls with test stubs; no CAN socket or motor access.
    set_target_properties(${test_name} PROPERTIES ENABLE_EXPORTS ON)
    # The URDF test takes a rendered product model as its command-line argument.
    if(NOT test_name STREQUAL "urdf_position_limits_test")
      add_test(NAME ${test_name} COMMAND ${test_name})
      set_tests_properties(${test_name} PROPERTIES TIMEOUT 15)
    endif()
  endforeach()
endif()

ament_package()
