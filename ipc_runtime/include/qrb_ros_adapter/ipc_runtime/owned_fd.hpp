// Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
// SPDX-License-Identifier: BSD-3-Clause-Clear

#ifndef QRB_ROS_ADAPTER_IPC_RUNTIME_OWNED_FD_HPP
#define QRB_ROS_ADAPTER_IPC_RUNTIME_OWNED_FD_HPP

#include <utility>

namespace qrb::adapter
{

/**
 * @brief Move-only owner of one file descriptor.
 *
 * Every fd that crosses the IPC boundary via SCM_RIGHTS is received into one
 * of these. The receiving side owns the fd the kernel just installed, and the
 * Definition of Done requires fd lifecycle tests to pass -- which is far
 * easier to guarantee with an RAII owner than with the `if ( fd >= 0 )
 * ::close( fd )` sprinkled along every early-return path.
 *
 * Deliberately has no Status/logging dependency: this is the lowest-level
 * low-level transport utility and is used by IpcEndpoint itself.
 */
class OwnedFd
{
public:
  OwnedFd() = default;

  explicit OwnedFd(int fd) : fd_(fd) {}

  ~OwnedFd() { Reset(); }

  OwnedFd(const OwnedFd &) = delete;
  OwnedFd & operator=(const OwnedFd &) = delete;

  OwnedFd(OwnedFd && other) noexcept : fd_(other.fd_) { other.fd_ = -1; }

  OwnedFd & operator=(OwnedFd && other) noexcept
  {
    if (this != &other) {
      Reset();
      fd_ = other.fd_;
      other.fd_ = -1;
    }
    return *this;
  }

  /** @brief The owned descriptor, or -1. Ownership stays here. */
  int Get() const { return fd_; }

  bool IsValid() const { return fd_ >= 0; }

  /** @brief Give up ownership without closing; caller becomes responsible. */
  [[nodiscard]] int Release()
  {
    int fd = fd_;
    fd_ = -1;
    return fd;
  }

  /** @brief Close the current fd (if any) and take ownership of `fd`. */
  void Reset(int fd = -1);

private:
  int fd_{ -1 };
};

}  // namespace qrb::adapter

#endif  // QRB_ROS_ADAPTER_IPC_RUNTIME_OWNED_FD_HPP
