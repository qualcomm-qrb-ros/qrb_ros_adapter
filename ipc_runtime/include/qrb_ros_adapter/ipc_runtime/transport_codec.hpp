// Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
// SPDX-License-Identifier: BSD-3-Clause-Clear

#ifndef QRB_ROS_ADAPTER_IPC_RUNTIME_TRANSPORT_CODEC_HPP
#define QRB_ROS_ADAPTER_IPC_RUNTIME_TRANSPORT_CODEC_HPP

#include <cstdint>
#include <vector>

#include "qrb_ros_adapter/ipc_runtime/status.hpp"
#include "qrb_ros_adapter/ipc_runtime/transport_header.hpp"

namespace qrb::adapter
{

/**
 * @brief Serialize a TransportHeader (little-endian, field by field).
 *
 * This codec knows about the header and nothing else. Business payloads are
 * Contract Protobuf and serialize themselves; control payloads are control
 * Protobuf and do the same. Contract-specific encoding does not belong in the
 * transport codec.
 */
std::vector<std::uint8_t> EncodeTransportHeader(const TransportHeader & header);

/**
 * @brief Parse a TransportHeader out of `data`.
 *
 * Validates every self-describing field so no caller has to: magic, protocol
 * version, that `operation` is a defined Operation, and that `payloadSize`
 * and `fdCount` are within kMaxPayloadSize / kMaxFdCount.
 *
 * @param data Must be exactly kTransportHeaderEncodedSize bytes.
 * @return kOk, kInvalidArgument if `data` is null or the wrong size,
 *         kVersionMismatch on a protocol version disagreement, or
 *         kProtocolError if any field fails validation.
 */
Status DecodeTransportHeader(const std::uint8_t * data, std::size_t len, TransportHeader & out);

}  // namespace qrb::adapter

#endif  // QRB_ROS_ADAPTER_IPC_RUNTIME_TRANSPORT_CODEC_HPP
