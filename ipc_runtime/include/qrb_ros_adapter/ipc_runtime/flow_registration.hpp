// Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
// SPDX-License-Identifier: BSD-3-Clause-Clear

#ifndef QRB_ROS_ADAPTER_IPC_RUNTIME_FLOW_REGISTRATION_HPP
#define QRB_ROS_ADAPTER_IPC_RUNTIME_FLOW_REGISTRATION_HPP

#include <string>

namespace qrb::adapter
{

/** @brief Runtime identity assigned by the ROS Bridge Manager at registration. */
struct FlowRegistration
{
  std::string dataSocketPath;
  std::string instanceToken;
};

}  // namespace qrb::adapter

#endif  // QRB_ROS_ADAPTER_IPC_RUNTIME_FLOW_REGISTRATION_HPP
