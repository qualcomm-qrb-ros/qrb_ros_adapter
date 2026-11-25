# -----------------------------------------------------------------------------
# IPC Runtime unit tests (UT-IPC-*).
#
# Pure functions; no socket, no ROS, no DMA heap. The IPC Runtime over real
# sockets is exercised by the end-to-end test in the top-level test/ directory.
# -----------------------------------------------------------------------------

function(qrb_ros_adapter_add_ipc_runtime_test name label)
  add_executable(${name} ${ARGN})
  target_link_libraries(${name} qrb_ros_adapter_runtime GTest::gtest_main)
  add_test(NAME ${name} COMMAND ${name})
  set_tests_properties(${name} PROPERTIES LABELS "${label}" TIMEOUT 60)
endfunction()

file(GLOB _ipc_runtime_unit_test_sources CONFIGURE_DEPENDS
  "${CMAKE_CURRENT_SOURCE_DIR}/unit/*_test.cpp")

qrb_ros_adapter_add_ipc_runtime_test(ipc_runtime_unit_tests unit
  ${_ipc_runtime_unit_test_sources})
