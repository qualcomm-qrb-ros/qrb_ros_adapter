// Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
// SPDX-License-Identifier: BSD-3-Clause-Clear

#include "qrb_ros_adapter/ipc_runtime/owned_fd.hpp"

#include <unistd.h>

namespace qrb::adapter
{

void OwnedFd::Reset(int fd)
{
  if (fd_ >= 0 && fd_ != fd)
    ::close(fd_);
  fd_ = fd;
}

}  // namespace qrb::adapter
