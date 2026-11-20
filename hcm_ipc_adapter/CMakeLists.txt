# -----------------------------------------------------------------------------
# qrb_hcm_ipc_adapter (HCM IPC Adapter)
#
# The HCM IPC Adapter has two cohesive submodules:
#   IPC Adapter Node  Direct QCNodeIfs implementation and configuration.
#   HCM Flow Client   IPC client state, synchronous requests and conversion.
#
# Deliberately does NOT depend on rclcpp: this code runs inside the HCM
# process, and the two runtimes stay isolated.
#
# The Adapter Node is a QCNode registered with QCNodeFactory, so it is built as
# part of the qcnode build and linked into libQCNode.so: the qcnode build adds
# this directory (as the qrb_qcnodes nodes are added) and links the OBJECT
# library QCNodeIpcAdapter into QCNode. SampleIpcAdapter is added to
# QCNodeSample, so that `qcrun QCNodeSampleApp -t IpcAdapter` runs it, and so is
# the HCM side of the end-to-end test (../test/hcm, `-t StreamProbe`).
#
# Installed CMake package: qrb_hcm_ipc_adapter, target
# qrb_ros_adapter::hcm_ipc_adapter (the headers and libQCNode.so).
# -----------------------------------------------------------------------------
cmake_minimum_required(VERSION 3.19)
project(qrb_hcm_ipc_adapter VERSION 0.2.0 LANGUAGES CXX)

if(NOT CMAKE_PROJECT_NAME STREQUAL "qcnode")
  message(FATAL_ERROR
    "qrb_hcm_ipc_adapter is built as part of the qcnode build, which links it into "
    "libQCNode.so; add this directory from the qcnode build instead.")
endif()

# This directory is added right after project(qcnode), before the qcnode build
# sets its own C++ standard and QC_TARGET_SOC.
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(QC_TARGET_SOC STREQUAL "x86")
  add_compile_definitions(QC_TARGET_SOC=0)
elseif(QC_TARGET_SOC)
  add_compile_definitions(QC_TARGET_SOC=${QC_TARGET_SOC})
else()
  add_compile_definitions(QC_TARGET_SOC=8797)
endif()

find_package(qrb_ros_adapter_runtime ${PROJECT_VERSION} REQUIRED)
find_package(nlohmann_json REQUIRED)
if(NOT TARGET qrb_contracts::qrb_contracts)
  find_package(qrb_contracts REQUIRED CONFIG)
endif()

# --- IPC Adapter Node, linked into libQCNode.so ------------------------------

file(GLOB _hcm_contract_converter_sources CONFIGURE_DEPENDS
  "${CMAKE_CURRENT_SOURCE_DIR}/source/flow_client/*_hcm_converter.cpp")

add_library(QCNodeIpcAdapter OBJECT
  source/flow_client/flow_context.cpp
  source/flow_client/hcm_flow_client.cpp
  source/ipc_adapter_node/hcm_ipc_adapter_config.cpp
  source/ipc_adapter_node/hcm_ipc_adapter_node.cpp
  source/flow_client/hcm_converter_registry.cpp
  source/flow_client/hcm_geometry_conversion.cpp
  ${_hcm_contract_converter_sources})

set_target_properties(QCNodeIpcAdapter PROPERTIES
  LINKER_LANGUAGE CXX
  POSITION_INDEPENDENT_CODE ON)

target_include_directories(QCNodeIpcAdapter PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include)

# QCNodeBase is defined later in the qcnode build; target names are resolved
# when the build system is generated.
target_link_libraries(QCNodeIpcAdapter PUBLIC
  QCNodeBase
  qrb_ros_adapter::runtime       # also brings qrb_contracts_proto::qrb_contracts_proto
  qrb_contracts::qrb_contracts   # QCNode buffer descriptors, one per Contract
  nlohmann_json::nlohmann_json)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  target_compile_options(QCNodeIpcAdapter PRIVATE -Wall -Wextra -Wpedantic)
endif()

# Run by `qcrun QCNodeSampleApp -t IpcAdapter`. QCNodeSample is defined later in
# the qcnode build (tests/sample), so the call is deferred.
cmake_language(EVAL CODE "cmake_language(DEFER DIRECTORY ${CMAKE_SOURCE_DIR} CALL
  target_sources QCNodeSample PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/sample/SampleIpcAdapter.cpp)")

option(QRB_HCM_IPC_ADAPTER_BUILD_STREAM_PROBE
  "Build the HCM side of the end-to-end test into QCNodeSample" ON)
if(QRB_HCM_IPC_ADAPTER_BUILD_STREAM_PROBE)
  add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../test/hcm ${CMAKE_CURRENT_BINARY_DIR}/stream_probe)
endif()

# --- Install and CMake package -----------------------------------------------

include(CMakePackageConfigHelpers)
set(_config_install_dir lib/cmake/${PROJECT_NAME})

install(DIRECTORY include/ DESTINATION include)

configure_package_config_file(cmake/${PROJECT_NAME}Config.cmake.in
  "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake"
  INSTALL_DESTINATION ${_config_install_dir})
write_basic_package_version_file(
  "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake"
  COMPATIBILITY SameMinorVersion)
install(FILES
    "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake"
    "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake"
    cmake/QrbQcNode.cmake
  DESTINATION ${_config_install_dir})

# --- Tests -------------------------------------------------------------------

option(QRB_HCM_IPC_ADAPTER_BUILD_TESTS "Build the HCM IPC Adapter unit tests" OFF)
if(QRB_HCM_IPC_ADAPTER_BUILD_TESTS)
  find_package(GTest REQUIRED)
  enable_testing()
  add_subdirectory(test)
endif()
