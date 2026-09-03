// Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
// SPDX-License-Identifier: BSD-3-Clause-Clear

#ifndef QRB_ROS_ADAPTER_IPC_RUNTIME_CONTROL_STATUS_HPP
#define QRB_ROS_ADAPTER_IPC_RUNTIME_CONTROL_STATUS_HPP

#include "qrb_ros_adapter/ipc_runtime/flow_types.hpp"
#include "qrb_ros_adapter/ipc_runtime/status.hpp"
#include "v1/ipc_control.pb.h"

namespace qrb::adapter
{

namespace ipc_proto = ::qrb::adapter::ipc::v1;

/**
 * @brief Map a Status onto the wire StatusCode (HLD §6.5).
 *
 * Local-only Status values collapse onto the closest wire code. Transport
 * version mismatch and timeout map to PROTOCOL_ERROR; kBadState maps to
 * INTERNAL_ERROR.
 */
ipc_proto::StatusCode ToProtoStatusCode(Status status);

/**
 * @brief Map a wire StatusCode onto a Status.
 *
 * An unrecognized or unspecified code becomes kProtocolError rather than being
 * silently treated as success -- control logic must depend only on the stable
 * enum values (HLD §6.5).
 */
Status FromProtoStatusCode(ipc_proto::StatusCode code);

/** @brief Map a Direction to its wire enum. */
ipc_proto::Direction ToProtoDirection(Direction direction);

/** @brief Map a wire Direction to Direction (unknown -> kUnspecified). */
Direction FromProtoDirection(ipc_proto::Direction direction);

/** @brief Map the restricted QoS profile to and from its wire enum. */
ipc_proto::QosProfile ToProtoQosProfile(QosProfile profile);
QosProfile FromProtoQosProfile(ipc_proto::QosProfile profile);

}  // namespace qrb::adapter

#endif  // QRB_ROS_ADAPTER_IPC_RUNTIME_CONTROL_STATUS_HPP
