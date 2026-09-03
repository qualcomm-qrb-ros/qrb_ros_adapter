# -----------------------------------------------------------------------------
# IPC Runtime
#
# ROS/HCM-independent runtime shared by the HCM IPC Adapter and ROS IPC Bridge.
# It owns Flow descriptors, validation, Adapter control protocol, Unix Domain
# Socket transport, request/response sessions, and fd RAII.
# -----------------------------------------------------------------------------

find_package(Protobuf REQUIRED)

set(_proto_root "${CMAKE_CURRENT_SOURCE_DIR}/proto")
set(_proto_gen_root "${CMAKE_CURRENT_BINARY_DIR}/generated")
file(MAKE_DIRECTORY "${_proto_gen_root}")

function(qrb_generate_ipc_proto_cpp out_srcs out_hdrs)
  set(_srcs "")
  set(_hdrs "")
  foreach(_proto ${ARGN})
    get_filename_component(_dir "${_proto}" DIRECTORY)
    get_filename_component(_name "${_proto}" NAME_WE)
    set(_src "${_proto_gen_root}/${_dir}/${_name}.pb.cc")
    set(_hdr "${_proto_gen_root}/${_dir}/${_name}.pb.h")
    add_custom_command(
      OUTPUT "${_src}" "${_hdr}"
      COMMAND ${Protobuf_PROTOC_EXECUTABLE}
              --proto_path=${_proto_root}
              --cpp_out=${_proto_gen_root}
              ${_proto}
      DEPENDS "${_proto_root}/${_proto}"
      WORKING_DIRECTORY "${_proto_root}"
      COMMENT "protoc ${_proto}"
      VERBATIM)
    list(APPEND _srcs "${_src}")
    list(APPEND _hdrs "${_hdr}")
  endforeach()
  set(${out_srcs} "${_srcs}" PARENT_SCOPE)
  set(${out_hdrs} "${_hdrs}" PARENT_SCOPE)
endfunction()

qrb_generate_ipc_proto_cpp(_control_srcs _control_hdrs v1/ipc_control.proto)

add_library(qrb_ros_adapter_runtime STATIC
  source/contract_validation.cpp
  source/status.cpp
  source/flow_types.cpp
  source/flow_config.cpp
  source/owned_fd.cpp
  source/transport_header.cpp
  source/transport_codec.cpp
  source/ipc_endpoint.cpp
  source/control_status.cpp
  source/ipc_session.cpp
  ${_control_srcs}
  ${_control_hdrs})

target_include_directories(qrb_ros_adapter_runtime
  PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>"
  SYSTEM PUBLIC
    "$<BUILD_INTERFACE:${_proto_gen_root}>"
    "$<INSTALL_INTERFACE:include>")
target_compile_features(qrb_ros_adapter_runtime PUBLIC cxx_std_17)
target_link_libraries(qrb_ros_adapter_runtime PUBLIC
  qrb_contracts_proto
  protobuf::libprotobuf-lite)
target_compile_options(qrb_ros_adapter_runtime PRIVATE -w)
set_target_properties(qrb_ros_adapter_runtime PROPERTIES POSITION_INDEPENDENT_CODE ON)

install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY "${_proto_gen_root}/" DESTINATION include
  FILES_MATCHING PATTERN "*.pb.h")
install(DIRECTORY proto/ DESTINATION share/${PROJECT_NAME}/proto
  FILES_MATCHING PATTERN "*.proto")
install(TARGETS qrb_ros_adapter_runtime DESTINATION lib)
