// Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
// SPDX-License-Identifier: BSD-3-Clause-Clear

#include "qrb_ros_adapter/ipc_runtime/status.hpp"

namespace qrb::adapter
{

const char * ToString(Status status)
{
  switch (status) {
    case Status::kOk:
      return "kOk";
    case Status::kNoData:
      return "kNoData";
    case Status::kInvalidArgument:
      return "kInvalidArgument";
    case Status::kUnsupportedContract:
      return "kUnsupportedContract";
    case Status::kVersionMismatch:
      return "kVersionMismatch";
    case Status::kProtocolError:
      return "kProtocolError";
    case Status::kConversionError:
      return "kConversionError";
    case Status::kResourceError:
      return "kResourceError";
    case Status::kInternalError:
      return "kInternalError";
    case Status::kTimeout:
      return "kTimeout";
    case Status::kBadState:
      return "kBadState";
  }
  return "kUnknown";
}

}  // namespace qrb::adapter
