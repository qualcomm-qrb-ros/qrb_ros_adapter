# -----------------------------------------------------------------------------
# qrb_ros_adapter_hcm_ipc_adapter
#
# The HCM IPC Adapter has two cohesive submodules:
#   IPC Adapter Node  Direct QCNodeIfs implementation and configuration.
#   HCM Flow Client   IPC client state, synchronous requests and conversion.
#
# Deliberately does NOT depend on rclcpp: this code runs inside the Graph
# Framework process, and isolating the two runtimes is HLD §2's first design
# goal. The Adapter Node itself is the concrete QCNode implementation used by
# GF; there is no second wrapper lifecycle.
# -----------------------------------------------------------------------------

find_package(nlohmann_json REQUIRED)

if(NOT DEFINED QRB_QCNODE_INCLUDE_DIR)
  get_filename_component(_adapter_root "${CMAKE_CURRENT_SOURCE_DIR}/.." ABSOLUTE)
  set(QRB_GF_SDK_ROOT
    "${_adapter_root}/external/gf" CACHE PATH
    "Path to the native Graph Framework SDK")
  set(QRB_QCNODE_SDK_ROOT
    "${_adapter_root}/external/qcnode_sdk" CACHE PATH
    "Path to the native QCNode SDK")
  set(QRB_GF_SOURCE_DIR "${_adapter_root}/../QC-Graph-Framework-Release_1_0"
    CACHE PATH "QC Graph Framework source tree")
  if(QRB_GF_SDK_ROOT)
    list(PREPEND CMAKE_PREFIX_PATH "${QRB_GF_SDK_ROOT}")
    find_package(QCGraphFramework CONFIG REQUIRED)
    set(QRB_QCNODE_INCLUDE_DIR "${QRB_QCNODE_SDK_ROOT}/include")
    set(QRB_GF_AVAILABLE ON)
  else()
    set(QRB_QCNODE_INCLUDE_DIR "${QRB_GF_SOURCE_DIR}/QcNode/include")
  endif()
endif()

if(NOT EXISTS "${QRB_QCNODE_INCLUDE_DIR}/QC/Node/Ifs/QCNodeIfs.hpp")
  message(FATAL_ERROR
    "QCNode SDK headers not found. Set QRB_GF_SDK_ROOT to a native GF SDK package.")
endif()

file(GLOB _hcm_contract_converter_sources CONFIGURE_DEPENDS
  "${CMAKE_CURRENT_SOURCE_DIR}/source/flow_client/*_hcm_converter.cpp")

add_library(qrb_ros_adapter_hcm_ipc_adapter SHARED
  source/flow_client/flow_context.cpp
  source/flow_client/hcm_flow_client.cpp
  source/ipc_adapter_node/hcm_ipc_adapter_config.cpp
  source/ipc_adapter_node/hcm_ipc_adapter_node.cpp
  source/flow_client/hcm_converter_registry.cpp
  ${_hcm_contract_converter_sources})

target_include_directories(qrb_ros_adapter_hcm_ipc_adapter PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<BUILD_INTERFACE:${QRB_QCNODE_INCLUDE_DIR}>"
  "$<INSTALL_INTERFACE:include>")

target_compile_features(qrb_ros_adapter_hcm_ipc_adapter PUBLIC cxx_std_17)

target_link_libraries(qrb_ros_adapter_hcm_ipc_adapter PUBLIC
  qrb_ros_adapter_runtime
  qrb_contracts_proto
  qrb_contracts_descriptors
  nlohmann_json::nlohmann_json)

if(TARGET QCGraphFramework::QCNode)
  target_link_libraries(qrb_ros_adapter_hcm_ipc_adapter PUBLIC
    QCGraphFramework::QCNode)
endif()

set_target_properties(qrb_ros_adapter_hcm_ipc_adapter PROPERTIES POSITION_INDEPENDENT_CODE ON)

install(DIRECTORY include/ DESTINATION include)
target_compile_definitions(qrb_ros_adapter_hcm_ipc_adapter PUBLIC
  QC_TARGET_SOC=${QRB_QCNODE_TARGET_SOC})

install(TARGETS qrb_ros_adapter_hcm_ipc_adapter DESTINATION lib)
