// Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
// SPDX-License-Identifier: BSD-3-Clause-Clear

#ifndef QRB_ROS_ADAPTER_IPC_RUNTIME_IPC_SESSION_HPP
#define QRB_ROS_ADAPTER_IPC_RUNTIME_IPC_SESSION_HPP

#include <chrono>
#include <cstdint>
#include <string>
#include <vector>

#include "qrb_ros_adapter/ipc_runtime/flow_config.hpp"
#include "qrb_ros_adapter/ipc_runtime/flow_registration.hpp"
#include "qrb_ros_adapter/ipc_runtime/flow_types.hpp"
#include "qrb_ros_adapter/ipc_runtime/ipc_endpoint.hpp"
#include "qrb_ros_adapter/ipc_runtime/owned_fd.hpp"
#include "qrb_ros_adapter/ipc_runtime/status.hpp"
#include "qrb_ros_adapter/ipc_runtime/transport_header.hpp"

namespace google::protobuf
{
class MessageLite;
}

namespace qrb::adapter
{

/**
 * @brief One IPC Session: an IpcEndpoint plus the handshake and the Protobuf
 * framing rules on top of it.
 *
 * After a successful handshake the Session is permanently bound to one
 * registered Flow instance. Contract, direction, payload class, ROS type and
 * Topic are fixed by that registration.
 *
 * This class is Contract-*agnostic*: it serializes and parses whatever
 * MessageLite it is handed. It links the control proto for handshakes and
 * operation status, but never links Contract-specific protos.
 *
 * Not thread-safe. A Flow's service loop owns its Session; nothing else may
 * touch it concurrently.
 */
class IpcSession
{
public:
  IpcSession() = default;

  IpcSession(const IpcSession &) = delete;
  IpcSession & operator=(const IpcSession &) = delete;

  // --- Server role ----------------------------------------------------------

  /** @brief Bind and listen on `socketPath`. */
  Status Listen(const std::string & socketPath);

  /** @brief Wait up to `timeout` for the one ROS IPC Bridge peer. */
  Status Accept(std::chrono::milliseconds timeout);

  /** @brief Accept one connection into a separate Session while retaining this listener. */
  Status AcceptConnection(IpcSession & connectionOut,
      std::chrono::milliseconds timeout,
      int wakeFd,
      bool & interrupted);

  /**
   * @brief Receive a CONNECT_REQUEST, validate it, and reply
   * CONNECT_RESPONSE.
   *
   * Validates the request's flow_id and Manager-issued instance_token against
   * `expected`. Flow configuration was already validated during registration.
   * On mismatch a CONNECT_RESPONSE carrying the mapped StatusCode and
   * diagnostic is still sent before the connection is closed.
   */
  Status PerformServerHandshake(const std::string & flowId,
      const FlowRegistration & registration,
      std::chrono::milliseconds timeout,
      std::string & diagnostic);

  // --- Client role ----------------------------------------------------------

  /** @brief Wait up to `timeout` for the initial connection to `socketPath`. */
  Status Connect(const std::string & socketPath, std::chrono::milliseconds timeout);

  /**
   * @brief Send CONNECT_REQUEST for one registered Flow and await CONNECT_RESPONSE.
   *
   * @return kOk, or the Status the peer reported / the transport failure.
   */
  Status PerformClientHandshake(const std::string & flowId,
      const FlowRegistration & registration,
      std::chrono::milliseconds timeout,
      std::string & diagnostic);

  // --- Framing --------------------------------------------------------------

  /**
   * @brief Send a frame whose payload is `message` (control or Contract --
   * this layer does not care which), plus `fdCount` SCM_RIGHTS fds.
   *
   * The caller keeps ownership of its local fds; send does not close them.
   * `payloadOffset` identifies a subrange in the attached DMA-BUF and must be
   * zero for frames without an fd.
   */
  Status SendProtoFrame(Operation operation,
      std::uint64_t requestId,
      const google::protobuf::MessageLite & message,
      const int * fds = nullptr,
      std::uint32_t fdCount = 0,
      std::uint32_t flags = 0,
      std::uint64_t payloadOffset = 0);

  /** @brief Send a frame with no payload, such as FETCH_REQUEST. */
  Status SendEmptyFrame(Operation operation, std::uint64_t requestId);

  /**
   * @brief Send an OperationStatus-carrying response frame.
   */
  Status SendStatusResponse(Operation operation,
      std::uint64_t requestId,
      Status status,
      const std::string & diagnostic);

  /** @brief Receive one frame. See IpcEndpoint::RecvFrame(). */
  Status ReceiveFrame(TransportHeader & headerOut,
      std::vector<std::uint8_t> & payloadOut,
      std::vector<OwnedFd> & fdsOut,
      std::chrono::milliseconds timeout);

  /**
   * @brief Parse a received payload into `out`.
   * @return kOk, or kProtocolError if the bytes are not a valid encoding of
   *         `out`'s type.
   */
  static Status ParseProtoPayload(const std::vector<std::uint8_t> & payload,
      google::protobuf::MessageLite & out);

  /** @brief Close the connection without an application-level CLOSE frame. */
  void Close() noexcept;

private:
  /** @brief Serialize `message` into scratch_ and send it. */
  Status SendSerialized(Operation operation,
      std::uint64_t requestId,
      const google::protobuf::MessageLite & message,
      const int * fds,
      std::uint32_t fdCount,
      std::uint32_t flags,
      std::uint64_t payloadOffset = 0);

  IpcEndpoint endpoint_;
  /** Reused serialization buffer -- one Flow sends one frame at a time, and
   * a per-frame heap allocation on the HCM IPC Adapter's scheduling thread is
   * exactly the kind of jitter the Graph Framework does not want. */
  std::vector<std::uint8_t> scratch_;
  bool handshakeDone_{ false };
};

}  // namespace qrb::adapter

#endif  // QRB_ROS_ADAPTER_IPC_RUNTIME_IPC_SESSION_HPP
