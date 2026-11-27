# -----------------------------------------------------------------------------
# qrb_ros_adapter_runtime (IPC Runtime)
#
# ROS/HCM-independent runtime shared by the HCM IPC Adapter and ROS IPC Bridge.
# It owns Flow descriptors, validation, Adapter control protocol, Unix Domain
# Socket transport, request/response sessions, and fd RAII. The Contract
# messages come from the qrb_contracts_proto package, which it links publicly
# so that both sides use one copy of them.
#
# Exported target: qrb_ros_adapter::runtime.
# -----------------------------------------------------------------------------
cmake_minimum_required(VERSION 3.16)
project(qrb_ros_adapter_runtime VERSION 1.0.0 LANGUAGES CXX)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Defines BUILD_TESTING (ON by default) and writes the ctest dashboard config
# that `colcon test` expects.
include(CTest)

include(cmake/QrbProtobuf.cmake)

# C++ of the qrb_contracts schemas. It must be a shared library, so that the
# HCM and ROS libraries loaded into one process register each Contract message
# in the protobuf descriptor pool only once.
find_package(qrb_contracts_proto REQUIRED CONFIG)

# --- Adapter control protocol ------------------------------------------------

set(_proto_root "${CMAKE_CURRENT_SOURCE_DIR}/proto")
set(_proto_gen_root "${CMAKE_CURRENT_BINARY_DIR}/generated")
file(MAKE_DIRECTORY "${_proto_gen_root}")

function(qrb_ros_adapter_generate_ipc_proto_cpp out_srcs out_hdrs)
  set(_srcs "")
  set(_hdrs "")
  foreach(_proto ${ARGN})
    get_filename_component(_dir "${_proto}" DIRECTORY)
    get_filename_component(_name "${_proto}" NAME_WE)
    set(_src "${_proto_gen_root}/${_dir}/${_name}.pb.cc")
    set(_hdr "${_proto_gen_root}/${_dir}/${_name}.pb.h")
    add_custom_command(
      OUTPUT "${_src}" "${_hdr}"
      COMMAND ${QRB_PROTOC}
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

qrb_ros_adapter_generate_ipc_proto_cpp(_control_srcs _control_hdrs v1/ipc_control.proto)

# --- Runtime library ---------------------------------------------------------

add_library(qrb_ros_adapter_runtime SHARED
  source/contract_time.cpp
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
add_library(qrb_ros_adapter::runtime ALIAS qrb_ros_adapter_runtime)

target_include_directories(qrb_ros_adapter_runtime PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>")
# The generated control header keeps its "v1/..." include path; install it
# under a package directory so it does not land in a bare include/v1/.
target_include_directories(qrb_ros_adapter_runtime SYSTEM PUBLIC
  "$<BUILD_INTERFACE:${_proto_gen_root}>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")
target_compile_features(qrb_ros_adapter_runtime PUBLIC cxx_std_17)
target_link_libraries(qrb_ros_adapter_runtime PUBLIC
  qrb_contracts_proto::qrb_contracts_proto
  protobuf::libprotobuf-lite)
target_compile_options(qrb_ros_adapter_runtime PRIVATE -w)
set_target_properties(qrb_ros_adapter_runtime PROPERTIES
  EXPORT_NAME runtime
  VERSION ${PROJECT_VERSION}
  SOVERSION ${PROJECT_VERSION_MAJOR})

# --- Install and CMake package -----------------------------------------------

include(CMakePackageConfigHelpers)
set(_config_install_dir lib/cmake/${PROJECT_NAME})

install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY "${_proto_gen_root}/" DESTINATION include/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.pb.h")
install(DIRECTORY proto/ DESTINATION share/${PROJECT_NAME}/proto
  FILES_MATCHING PATTERN "*.proto")

install(TARGETS qrb_ros_adapter_runtime
  EXPORT ${PROJECT_NAME}Targets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(EXPORT ${PROJECT_NAME}Targets
  NAMESPACE qrb_ros_adapter::
  DESTINATION ${_config_install_dir})

configure_package_config_file(cmake/${PROJECT_NAME}Config.cmake.in
  "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake"
  INSTALL_DESTINATION ${_config_install_dir})
write_basic_package_version_file(
  "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake"
  COMPATIBILITY SameMinorVersion)
install(FILES
    "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake"
    "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake"
  DESTINATION ${_config_install_dir})

# --- Tests -------------------------------------------------------------------

if(BUILD_TESTING)
  find_package(GTest)
  if(TARGET GTest::gtest_main)
    add_subdirectory(test)
  else()
    message(STATUS "GTest not found, skipping IPC Runtime tests")
  endif()
endif()
