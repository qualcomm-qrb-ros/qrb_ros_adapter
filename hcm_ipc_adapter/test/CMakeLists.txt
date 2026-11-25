# -----------------------------------------------------------------------------
# HCM IPC Adapter unit tests (UT-HCM-*).
#
# Pure functions and single classes; no socket and no peer. HcmFlowClient and
# HcmIpcAdapterNode against the real ROS Bridge Manager are exercised by
# the end-to-end test in the top-level test/ directory.
#
# Built in the qcnode build with -DQRB_HCM_IPC_ADAPTER_BUILD_TESTS=ON. The tests
# link libQCNode.so, which holds the Adapter code, rather than QCNodeIpcAdapter:
# its objects in both would register the node with QCNodeFactory twice.
# -----------------------------------------------------------------------------

function(qrb_ros_adapter_add_hcm_adapter_test name label)
  add_executable(${name} ${ARGN})
  target_link_libraries(${name}
    QCNode
    GTest::gtest_main)
  add_test(NAME ${name} COMMAND ${name})
  set_tests_properties(${name} PROPERTIES LABELS "${label}" TIMEOUT 60)
endfunction()

file(GLOB _hcm_adapter_unit_test_sources CONFIGURE_DEPENDS
  "${CMAKE_CURRENT_SOURCE_DIR}/unit/*_test.cpp")

qrb_ros_adapter_add_hcm_adapter_test(hcm_ipc_adapter_unit_tests unit
  ${_hcm_adapter_unit_test_sources})
