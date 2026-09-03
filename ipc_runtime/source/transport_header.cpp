// Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
// SPDX-License-Identifier: BSD-3-Clause-Clear

#include "qrb_ros_adapter/ipc_runtime/transport_header.hpp"

namespace qrb::adapter
{

const char * ToString(Operation operation)
{
  switch (operation) {
    case Operation::kConnectRequest:
      return "CONNECT_REQUEST";
    case Operation::kConnectResponse:
      return "CONNECT_RESPONSE";
    case Operation::kFetchRequest:
      return "FETCH_REQUEST";
    case Operation::kFetchResponse:
      return "FETCH_RESPONSE";
    case Operation::kPublishRequest:
      return "PUBLISH_REQUEST";
    case Operation::kPublishResponse:
      return "PUBLISH_RESPONSE";
    case Operation::kRegisterFlowRequest:
      return "REGISTER_FLOW_REQUEST";
    case Operation::kRegisterFlowResponse:
      return "REGISTER_FLOW_RESPONSE";
    case Operation::kUnregisterFlowRequest:
      return "UNREGISTER_FLOW_REQUEST";
    case Operation::kUnregisterFlowResponse:
      return "UNREGISTER_FLOW_RESPONSE";
    case Operation::kFetchComplete:
      return "FETCH_COMPLETE";
  }
  return "UNKNOWN";
}

bool IsValidOperation(std::uint16_t value)
{
  switch (static_cast<Operation>(value)) {
    case Operation::kConnectRequest:
    case Operation::kConnectResponse:
    case Operation::kFetchRequest:
    case Operation::kFetchResponse:
    case Operation::kPublishRequest:
    case Operation::kPublishResponse:
    case Operation::kRegisterFlowRequest:
    case Operation::kRegisterFlowResponse:
    case Operation::kUnregisterFlowRequest:
    case Operation::kUnregisterFlowResponse:
    case Operation::kFetchComplete:
      return true;
  }
  return false;
}

}  // namespace qrb::adapter
