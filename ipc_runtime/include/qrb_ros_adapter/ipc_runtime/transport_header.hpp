// Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
// SPDX-License-Identifier: BSD-3-Clause-Clear

#ifndef QRB_ROS_ADAPTER_IPC_RUNTIME_TRANSPORT_HEADER_HPP
#define QRB_ROS_ADAPTER_IPC_RUNTIME_TRANSPORT_HEADER_HPP

#include <cstddef>
#include <cstdint>

namespace qrb::adapter
{

/** @brief Magic bytes identifying an Adapter transport frame: "QRBA". */
constexpr std::uint32_t kTransportMagic = 0x51524241u;

/**
 * @brief Current transport protocol version.
 *
 * Version 1 adds the DMA payload offset to the fixed wire header. Peers with a
 * different version are rejected before operation-specific parsing.
 */
constexpr std::uint16_t kTransportVersion = 1;

/**
 * @brief Frame operations (HLD §11.3).
 *
 * Every operation maps to exactly one payload type:
 *
 *   CONNECT_REQUEST   -> ipc.v1.ConnectRequest
 *   CONNECT_RESPONSE  -> ipc.v1.ConnectResponse
 *   FETCH_REQUEST     -> (no payload)
 *   FETCH_RESPONSE    -> bound qrb.contract.* message, when flags has
 *                        kHasData; otherwise ipc.v1.OperationStatus
 *   FETCH_COMPLETE    -> ipc.v1.OperationStatus after a received DMA-BUF has
 *                        been copied into HCM-owned storage
 *   PUBLISH_REQUEST   -> bound qrb.contract.* message
 *   PUBLISH_RESPONSE  -> ipc.v1.OperationStatus
 *   REGISTER_FLOW_REQUEST/RESPONSE -> ipc.v1.RegisterFlowRequest/Response
 *   UNREGISTER_FLOW_REQUEST/RESPONSE -> ipc.v1.UnregisterFlowRequest/Response
 *
 * Values are stable within one protocol version; incompatible changes bump
 * kTransportVersion.
 */
enum class Operation : std::uint16_t
{
  kConnectRequest = 1,
  kConnectResponse = 2,
  kFetchRequest = 3,
  kFetchResponse = 4,
  kPublishRequest = 5,
  kPublishResponse = 6,
  kRegisterFlowRequest = 7,
  kRegisterFlowResponse = 8,
  kUnregisterFlowRequest = 9,
  kUnregisterFlowResponse = 10,
  kFetchComplete = 11,
};

/** @brief Human-readable name for an Operation, for logging. */
const char * ToString(Operation operation);

/** @brief True if `value` is a defined Operation. */
bool IsValidOperation(std::uint16_t value);

namespace transport_flags
{
/**
 * @brief FETCH_RESPONSE only: a Contract message is present.
 *
 * Without it the FETCH_RESPONSE payload is an OperationStatus carrying
 * NO_DATA -- the ROS IPC Bridge's bounded fresh-sample wait expired.
 */
constexpr std::uint32_t kHasData = 1u << 0;
}  // namespace transport_flags

/**
 * @brief Fixed-size header prefixing every frame -- HLD §9.2, which fixes the
 * field list to exactly: magic, protocol version, operation, request ID,
 * payload size, flags, fd count, DMA payload offset.
 *
 * Nothing Contract-specific is added here. A new Contract adds its descriptor
 * and converters without changing this transport header.
 *
 * Always encoded/decoded field-by-field in little-endian order via
 * transport_codec.hpp rather than memcpy'd as a struct, so this process's
 * padding and alignment choices are never a wire compatibility concern.
 */
struct TransportHeader
{
  std::uint32_t magic{ kTransportMagic };
  std::uint16_t protocolVersion{ kTransportVersion };
  std::uint16_t operation{ 0 };
  std::uint64_t requestId{ 0 };
  std::uint32_t payloadSize{ 0 };
  std::uint32_t flags{ 0 };
  std::uint32_t fdCount{ 0 };
  /** Byte offset of the logical payload within the attached DMA-BUF. */
  std::uint64_t payloadOffset{ 0 };
};

/** @brief Encoded size of TransportHeader on the wire. */
constexpr std::size_t kTransportHeaderEncodedSize = 36;

/**
 * @brief Upper bound on one frame's Protobuf payload.
 *
 * Contract messages are metadata only -- DMA-BUF payloads travel as fds, and
 * even a PointCloud's field list is a few hundred bytes. 1 MiB is a sanity
 * bound on a decoded `payloadSize`, not a design target.
 */
constexpr std::size_t kMaxPayloadSize = 1u << 20;

/**
 * @brief Upper bound on fds in one frame.
 *
 * The Contracts implemented today carry at most one DMA-BUF payload fd, and
 * both DMA-BUF Converters reject anything but exactly one. The bound is >1
 * only so that a peer sending a malformed fdCount is rejected by validation
 * rather than by a buffer overrun in the SCM_RIGHTS control-message math.
 */
constexpr std::uint32_t kMaxFdCount = 4;

}  // namespace qrb::adapter

#endif  // QRB_ROS_ADAPTER_IPC_RUNTIME_TRANSPORT_HEADER_HPP
