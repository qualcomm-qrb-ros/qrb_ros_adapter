# -----------------------------------------------------------------------------
# qrb_ros_adapter
#
# The HCM IPC Adapter, plus the ROS-independent IPC layers. Buildable with
# plain CMake and no ROS installation at all -- that is the point: this code
# runs inside the Graph Framework process, and HLD §2's first design goal is to
# isolate the HCM and ROS 2 runtime dependencies from each other.
#
#   ipc_runtime/ ROS/HCM-independent IPC definitions and transport
#   hcm_ipc_adapter/ IPC Adapter Node plus HCM Flow Client
#
# The ROS IPC Bridge lives in ros_ipc_bridge/ and is a separate ament package, built with
# colcon: it needs rclcpp, rclcpp_components and qrb_ros_transport, none of
# which belong in this tree. It pulls the directories above in via
# add_subdirectory(), so there is exactly one copy of the shared code.
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

set(QRB_CONTRACTS_SOURCE_DIR
  "${CMAKE_CURRENT_SOURCE_DIR}/external/qrb_contracts" CACHE PATH
  "Path to the independent qrb_contracts source tree")
set(QRB_GF_SDK_ROOT
  "${CMAKE_CURRENT_SOURCE_DIR}/external/gf" CACHE PATH
  "Path to the native Graph Framework SDK")
set(QRB_QCNODE_SDK_ROOT
  "${CMAKE_CURRENT_SOURCE_DIR}/external/qcnode_sdk" CACHE PATH
  "Path to the native QCNode SDK")
set(QRB_GF_SOURCE_DIR
  "${CMAKE_CURRENT_SOURCE_DIR}/../QC-Graph-Framework-Release_1_0"
  CACHE PATH "QC Graph Framework source tree used when no packaged SDK is selected")
set(QRB_QCNODE_TARGET_SOC "8797" CACHE STRING "QCNode SDK target SoC definition")

set(QRB_GF_AVAILABLE OFF)
if(QRB_GF_SDK_ROOT)
  list(PREPEND CMAKE_PREFIX_PATH "${QRB_GF_SDK_ROOT}")
  find_package(QCGraphFramework CONFIG REQUIRED)
  set(QRB_QCNODE_INCLUDE_DIR "${QRB_QCNODE_SDK_ROOT}/include")
  set(QRB_GF_AVAILABLE ON)
else()
  set(QRB_QCNODE_INCLUDE_DIR "${QRB_GF_SOURCE_DIR}/QcNode/include")
endif()
include("${CMAKE_CURRENT_SOURCE_DIR}/cmake/QrbContracts.cmake")

add_subdirectory(ipc_runtime)
add_subdirectory(hcm_ipc_adapter)
