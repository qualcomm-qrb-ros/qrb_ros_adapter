# -----------------------------------------------------------------------------
# qrb_ros_adapter (ROS IPC Bridge)
#
# The ament package: one generic ROS Flow implementation whose typed factories
# are generated from the shared Contract mapping list.
#
# The ROS-free IPC Runtime comes from the qrb_ros_adapter_runtime package,
# which also brings the qrb_contracts_proto messages; this package does not
# depend on QCNode or the HCM IPC Adapter.
#
# The implementation has two cohesive submodules: Bridge Manager and ROS Flow
# Bridge. Message converters are discovered automatically and linked into the
# common Flow implementation.
#
#   qrb_ros_adapter_ros_flow_bridge  Per-Flow server, typed ROS endpoint,
#                                    conversion and payload ownership.
#
# Exported targets: qrb_ros_adapter::ros_flow_bridge,
# qrb_ros_adapter::ros_ipc_bridge_manager.
# -----------------------------------------------------------------------------
cmake_minimum_required(VERSION 3.16)
project(qrb_ros_adapter VERSION 1.0.0 LANGUAGES CXX)

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
find_package(qrb_ros_adapter_runtime ${PROJECT_VERSION} REQUIRED)
# ROS message packages used by the mapped Contracts; also list them in package.xml.
set(QRB_ROS_MSG_DEPENDENCIES
  builtin_interfaces
  std_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
  tf2_msgs
  vision_msgs)
foreach(_msg_dependency IN LISTS QRB_ROS_MSG_DEPENDENCIES)
  find_package(${_msg_dependency} REQUIRED)
endforeach()
# qrb_ros_transport types of the DmaBuf Contracts; also list them in package.xml.
set(QRB_ROS_TRANSPORT_DEPENDENCIES
  qrb_ros_transport_image_type
  qrb_ros_transport_point_cloud2_type)
foreach(_transport_dependency IN LISTS QRB_ROS_TRANSPORT_DEPENDENCIES)
  find_package(${_transport_dependency} REQUIRED)
endforeach()

# --- Component base ----------------------------------------------------------

file(GLOB _qrb_ros_converter_sources CONFIGURE_DEPENDS
  "${CMAKE_CURRENT_SOURCE_DIR}/source/flow_bridge/*_ros_converter.cpp")

add_library(qrb_ros_adapter_ros_flow_bridge SHARED
  source/flow_bridge/ros_flow_bridge_base.cpp
  source/flow_bridge/ros_contract_header.cpp
  source/flow_bridge/ros_contract_geometry.cpp
  source/flow_bridge/ros_dma_buffer_pool.cpp
  ${_qrb_ros_converter_sources})

target_include_directories(qrb_ros_adapter_ros_flow_bridge PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>")

# The runtime brings qrb_contracts_proto::qrb_contracts_proto.
target_link_libraries(qrb_ros_adapter_ros_flow_bridge PUBLIC
  qrb_ros_adapter::runtime)

set_target_properties(qrb_ros_adapter_ros_flow_bridge PROPERTIES
  EXPORT_NAME ros_flow_bridge)
ament_target_dependencies(qrb_ros_adapter_ros_flow_bridge PUBLIC
  rclcpp
  ${QRB_ROS_MSG_DEPENDENCIES}
  ${QRB_ROS_TRANSPORT_DEPENDENCIES})

# --- Dynamic Flow Manager and shared-process container -----------------------

add_library(qrb_ros_adapter_ros_ipc_bridge_manager SHARED
  source/manager/ros_contract_registry.cpp
  source/manager/ros_bridge_manager.cpp)
target_link_libraries(qrb_ros_adapter_ros_ipc_bridge_manager PUBLIC
  qrb_ros_adapter_ros_flow_bridge)
set_target_properties(qrb_ros_adapter_ros_ipc_bridge_manager PROPERTIES
  EXPORT_NAME ros_ipc_bridge_manager)
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
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(TARGETS ros_ipc_bridge
  RUNTIME DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  rclcpp
  rclcpp_components
  ${QRB_ROS_MSG_DEPENDENCIES}
  qrb_ros_adapter_runtime
  ${QRB_ROS_TRANSPORT_DEPENDENCIES})

if(BUILD_TESTING)
  find_package(GTest)
  if(TARGET GTest::gtest_main)
    add_subdirectory(test)
  else()
    message(STATUS "GTest not found, skipping ROS IPC Bridge tests")
  endif()
endif()

ament_package()
