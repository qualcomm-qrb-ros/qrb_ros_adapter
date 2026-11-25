// Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
// SPDX-License-Identifier: BSD-3-Clause-Clear

#ifndef QRB_ROS_ADAPTER_IPC_RUNTIME_STATUS_HPP
#define QRB_ROS_ADAPTER_IPC_RUNTIME_STATUS_HPP

#include <cstdint>

namespace qrb::adapter
{

/**
 * @brief Result of an Adapter operation.
 *
 * The first eight values correspond to `qrb.adapter.ipc.v1.StatusCode`, so a
 * Status can cross the wire without losing its meaning -- see
 * ToProtoStatusCode()/FromProtoStatusCode() in control_status.hpp.
 *
 * The last three are local-only: they describe transport or call failures
 * rather than what a peer reported, and never appear in a control message.
 *
 * This header deliberately has no Protobuf dependency: converters, registries
 * and FlowConfig validation can share one result type without linking the
 * control protocol.
 */
enum class Status : std::uint8_t
{
  kOk = 0,              /**< Success. */
  kNoData,              /**< The ROS bridge had no fresh sample when asked.
                             A normal outcome, not an error: the host call
                             succeeds without writing the descriptor, and
                             the Flow and data Session remain usable. */
  kInvalidArgument,     /**< Caller supplied an invalid argument, or a
                             received message failed validation. */
  kUnsupportedContract, /**< Contract type name is not registered, its
                             descriptor does not support the requested
                             direction, or the Flow direction is unspecified. */
  kProtocolError,       /**< Malformed frame, unexpected operation, bad fd
                             count, or a mismatched request ID. */
  kConversionError,     /**< Contract <-> ROS/HCM conversion failed a
                             consistency or finite-value check. */
  kResourceError,       /**< fd, DMA-BUF, socket or memory resource
                             failure. */
  kInternalError,       /**< Invariant violation on our own side. */

  kVersionMismatch, /**< Transport protocol version disagreement. */
  kTimeout,         /**< No frame arrived within the caller's bound.
                         Distinct from kNoData: the peer said nothing
                         at all, rather than "nothing to give". */
  kBadState,        /**< Operation not valid for the current resource
                         setup (e.g. a data op before the handshake). */
};

/** @brief Human-readable name for a Status, for logging. */
const char * ToString(Status status);

/** @brief True if `status` means "the call did what was asked". */
constexpr bool IsOk(Status status)
{
  return status == Status::kOk;
}

/**
 * @brief True if `status` rejects one frame and leaves the Flow usable.
 *
 * The frame did not fit the descriptor or the Contract (for example a payload
 * larger than the destination buffer). Both sides finish the request/response
 * exchange, drop that frame and keep the Session open for the next one.
 */
constexpr bool IsFrameError(Status status)
{
  return status == Status::kInvalidArgument || status == Status::kConversionError;
}

/**
 * @brief True if `status` is terminal for a Flow.
 *
 * A Protocol/Transport/fd/resource error or request timeout closes the Session
 * and the Flow never reconnects on its own. kNoData is a completed, nonfatal
 * request; a frame error (IsFrameError()) drops only that frame.
 */
constexpr bool IsFatalForFlow(Status status)
{
  return status != Status::kOk && status != Status::kNoData && !IsFrameError(status);
}

}  // namespace qrb::adapter

#endif  // QRB_ROS_ADAPTER_IPC_RUNTIME_STATUS_HPP
