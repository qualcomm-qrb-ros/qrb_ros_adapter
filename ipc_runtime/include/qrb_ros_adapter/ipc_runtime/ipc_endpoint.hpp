// Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
// SPDX-License-Identifier: BSD-3-Clause-Clear

#ifndef QRB_ROS_ADAPTER_IPC_RUNTIME_IPC_ENDPOINT_HPP
#define QRB_ROS_ADAPTER_IPC_RUNTIME_IPC_ENDPOINT_HPP

#include <chrono>
#include <cstdint>
#include <string>
#include <vector>

#include "qrb_ros_adapter/ipc_runtime/owned_fd.hpp"
#include "qrb_ros_adapter/ipc_runtime/status.hpp"
#include "qrb_ros_adapter/ipc_runtime/transport_header.hpp"

namespace qrb::adapter
{

/**
 * @brief One Unix Domain Socket (SOCK_SEQPACKET) connection: framing plus
 * SCM_RIGHTS fd passing, and nothing else.
 *
 * This class knows about TransportHeader and file descriptors. It does not
 * know about Protobuf, Contracts, Flows or handshakes; those belong to
 * IpcSession and the owning runtime.
 *
 * One Flow is a 1:1 point-to-point connection. The dynamic ROS Flow Bridge
 * listens and HCM connects. After Accept()/Connect() there is exactly one peer
 * for this endpoint's lifetime. This class never
 * retries or reconnects once connected; connection recovery policy belongs to
 * the runtime layer (HLD §5.5).
 *
 * Not copyable: owns raw fds.
 */
class IpcEndpoint
{
public:
  IpcEndpoint() = default;
  ~IpcEndpoint();

  IpcEndpoint(const IpcEndpoint &) = delete;
  IpcEndpoint & operator=(const IpcEndpoint &) = delete;

  /**
   * @brief Server side: create, bind and listen on `socketPath`.
   *
   * The deployment system must create the parent directory. An existing path
   * is removed only when it is a same-owner socket with no active listener.
   * The created socket is mode 0660. Must be followed by Accept().
   */
  Status Listen(const std::string & socketPath);

  /**
   * @brief Server side: wait up to `timeout` for the one peer to connect.
   *
   * On success the listening socket is closed immediately -- one Flow, one
   * connection, so there is nothing left to listen for.
   *
   * @return kOk, kTimeout (no peer within the window), or kResourceError.
   */
  Status Accept(std::chrono::milliseconds timeout);

  /**
   * @brief Accept one peer into a separate endpoint while keeping this listener open.
   *
   * Used by the process-level bootstrap server, which handles many short
   * control connections on one stable socket path.
   */
  Status AcceptConnection(IpcEndpoint & connectionOut,
      std::chrono::milliseconds timeout,
      int wakeFd,
      bool & interrupted);

  /**
   * @brief Client side: one non-blocking connect attempt.
   *
   * Exactly one attempt, no sleeping, no retry loop. A runtime may schedule
   * the initial attempt outside its constructor, but a failed established
   * Flow is never reconnected by this endpoint.
   *
   * @return kOk (connected), kTimeout (the peer is not listening), or
   *         kResourceError.
   */
  Status ConnectOnce(const std::string & socketPath);

  /**
   * @brief Send one frame: header + payload + `fdCount` SCM_RIGHTS fds.
   *
   * Non-blocking (MSG_DONTWAIT): this layer never waits for kernel
   * send-buffer space. The caller retains ownership of the fds in `fds`;
   * this call does not close them. SCM_RIGHTS creates receiver-local fd
   * references without transferring HCM buffer lifecycle authority.
   *
   * @param fds May be null when `fdCount` is 0.
   * @param payloadOffset Byte offset of the logical DMA payload in the first
   *        attached fd. Must be zero when no fd is attached.
   * @return kOk, kBadState if not connected, kInvalidArgument if
   *         `payloadSize`/`fdCount` exceed their bounds, or kResourceError.
   *         EAGAIN is reported as kResourceError rather than retried: a
   *         metadata-sized frame hitting the send buffer limit is itself
   *         evidence the peer is not draining.
   */
  Status SendFrame(Operation operation,
      std::uint64_t requestId,
      const void * payload,
      std::size_t payloadSize,
      const int * fds = nullptr,
      std::uint32_t fdCount = 0,
      std::uint32_t flags = 0,
      std::uint64_t payloadOffset = 0);

  /**
   * @brief Receive one frame, waiting up to `timeout`.
   *
   * Any fds the frame carried are moved into `fdsOut` as OwnedFd, so they
   * are closed even if the caller abandons the frame on an early return.
   * `fdsOut` is cleared first.
   *
   * A frame whose actual fd count disagrees with its header's `fdCount` is
   * rejected as kProtocolError -- and the fds that *did* arrive are still
   * closed, because they landed in `fdsOut` before the check ran.
   *
   * @return kOk, kTimeout (nothing arrived), kBadState if not connected,
   *         kProtocolError (malformed frame), kVersionMismatch, or
   *         kResourceError (peer closed / socket failure).
   */
  Status RecvFrame(TransportHeader & headerOut,
      std::vector<std::uint8_t> & payloadOut,
      std::vector<OwnedFd> & fdsOut,
      std::chrono::milliseconds timeout);

  /** @brief Close the connection, the listening socket if still open, and
   * unlink the socket file if this endpoint created it. Idempotent. */
  void Close() noexcept;

private:
  Status WaitAndAccept(std::chrono::milliseconds timeout, int wakeFd, bool & woken, int & connOut);

  int fd_{ -1 };        // the connected (post accept/connect) socket
  int listenFd_{ -1 };  // only set between Listen() and Accept()
  std::string socketPath_;
  bool isServer_{ false };
  /** Reused recvmsg storage; one endpoint is consumed by one Session thread. */
  std::vector<std::uint8_t> receiveBuffer_;
};

}  // namespace qrb::adapter

#endif  // QRB_ROS_ADAPTER_IPC_RUNTIME_IPC_ENDPOINT_HPP
