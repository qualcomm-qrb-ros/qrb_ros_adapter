# -----------------------------------------------------------------------------
# qrb_ros_adapter (ROS IPC Bridge)
#
# The ament package: one generic ROS Flow implementation whose typed factories
# are generated from the shared Contract mapping list.
#
# The shared, ROS-free Adapter code lives one directory up and is pulled in with
# add_subdirectory() so there is exactly one copy of it. Business Contract
# schemas come from the independent qrb_contracts repository. Their generated
# C++ is built by the Adapter's CMake integration. The shared directories also
# build standalone with plain CMake and no ROS at all -- see ../CMakeLists.txt.
#
# The implementation has two cohesive submodules: Bridge Manager and ROS Flow
# Bridge. Message converters are discovered automatically and linked into the
# common Flow implementation.
#
#   qrb_ros_adapter_ros_flow_bridge  Per-Flow server, typed ROS endpoint,
#                                    conversion and payload ownership.
# -----------------------------------------------------------------------------
cmake_minimum_required(VERSION 3.16)
project(qrb_ros_adapter VERSION 0.2.0 LANGUAGES CXX)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
set(QRB_ROS_TRANSPORT_DEPENDENCIES
  qrb_ros_transport_point_cloud2_type)
foreach(_transport_dependency IN LISTS QRB_ROS_TRANSPORT_DEPENDENCIES)
  find_package(${_transport_dependency} REQUIRED)
endforeach()

# --- Shared, ROS-free layers -------------------------------------------------

# REALPATH first: colcon workspaces routinely reach a package through a symlink
# (`src/<pkg>` -> the real directory), and resolving ".." on the symlinked path
# would land outside the repository entirely.
get_filename_component(_ros_ipc_bridge_dir "${CMAKE_CURRENT_SOURCE_DIR}" REALPATH)
get_filename_component(_adapter_root "${_ros_ipc_bridge_dir}/.." ABSOLUTE)

if(NOT EXISTS "${_adapter_root}/ipc_runtime/CMakeLists.txt")
  message(FATAL_ERROR
    "Could not find the shared qrb_ros_adapter layers next to ros_ipc_bridge/.\n"
    "Expected them at: ${_adapter_root}\n"
    "This package must stay inside the qrb_ros_adapter repository, which "
    "also provides ipc_runtime/.")
endif()

set(QRB_CONTRACTS_SOURCE_DIR "${_adapter_root}/external/qrb_contracts" CACHE PATH
  "Path to the independent qrb_contracts source tree")
include("${_adapter_root}/cmake/QrbContracts.cmake")

add_subdirectory("${_adapter_root}/ipc_runtime" ipc_runtime)

# --- Component base ----------------------------------------------------------

file(GLOB _qrb_ros_converter_sources CONFIGURE_DEPENDS
  "${CMAKE_CURRENT_SOURCE_DIR}/source/flow_bridge/*_ros_converter.cpp")

add_library(qrb_ros_adapter_ros_flow_bridge SHARED
  source/flow_bridge/ros_flow_bridge_base.cpp
  source/flow_bridge/ros_contract_time.cpp
  source/flow_bridge/ros_dma_buffer_pool.cpp
  ${_qrb_ros_converter_sources})

target_include_directories(qrb_ros_adapter_ros_flow_bridge PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>")

target_link_libraries(qrb_ros_adapter_ros_flow_bridge PUBLIC
  qrb_ros_adapter_runtime
  qrb_contracts_proto)

ament_target_dependencies(qrb_ros_adapter_ros_flow_bridge PUBLIC
  rclcpp
  builtin_interfaces
  geometry_msgs
  sensor_msgs
  ${QRB_ROS_TRANSPORT_DEPENDENCIES})

# --- Dynamic Flow Manager and shared-process container -----------------------

add_library(qrb_ros_adapter_ros_ipc_bridge_manager SHARED
  source/manager/ros_contract_registry.cpp
  source/manager/ros_bridge_manager.cpp)
target_link_libraries(qrb_ros_adapter_ros_ipc_bridge_manager PUBLIC
  qrb_ros_adapter_ros_flow_bridge)
ament_target_dependencies(qrb_ros_adapter_ros_ipc_bridge_manager PUBLIC
  rclcpp rclcpp_components)

add_executable(ros_ipc_bridge
  source/ros_ipc_bridge.cpp)
target_link_libraries(ros_ipc_bridge
  qrb_ros_adapter_ros_ipc_bridge_manager)
ament_target_dependencies(ros_ipc_bridge
  rclcpp rclcpp_components)

# --- Install -----------------------------------------------------------------

install(DIRECTORY include/ DESTINATION include)

install(TARGETS
    qrb_ros_adapter_ros_flow_bridge
    qrb_ros_adapter_ros_ipc_bridge_manager
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(TARGETS ros_ipc_bridge
  RUNTIME DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

ament_package()
