# -----------------------------------------------------------------------------
# ROS IPC Bridge unit tests (UT-ROS-*).
#
# Converters, LatestSample and the registry; no ROS Context, no socket and
# no DMA heap. Flow Bridges and the Manager are exercised by the end-to-end
# test in the top-level test/ directory.
#
# Tests link the production libraries; no production source is compiled twice.
# -----------------------------------------------------------------------------

file(GLOB _ros_ipc_bridge_unit_test_sources CONFIGURE_DEPENDS
  "${CMAKE_CURRENT_SOURCE_DIR}/unit/*_test.cpp")

ament_add_gtest(ros_ipc_bridge_unit_tests
  ${_ros_ipc_bridge_unit_test_sources}
  TIMEOUT 60)
target_include_directories(ros_ipc_bridge_unit_tests PRIVATE
  "${CMAKE_CURRENT_SOURCE_DIR}/../include")
target_link_libraries(ros_ipc_bridge_unit_tests
  qrb_ros_adapter_ros_ipc_bridge_manager)
ament_target_dependencies(ros_ipc_bridge_unit_tests
  rclcpp ${QRB_ROS_MSG_DEPENDENCIES} ${QRB_ROS_TRANSPORT_DEPENDENCIES})
set_tests_properties(ros_ipc_bridge_unit_tests PROPERTIES LABELS "unit")
