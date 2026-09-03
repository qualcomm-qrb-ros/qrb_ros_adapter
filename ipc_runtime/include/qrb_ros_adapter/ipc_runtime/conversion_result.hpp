// Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
// SPDX-License-Identifier: BSD-3-Clause-Clear

#ifndef QRB_ROS_ADAPTER_IPC_RUNTIME_CONVERSION_RESULT_HPP
#define QRB_ROS_ADAPTER_IPC_RUNTIME_CONVERSION_RESULT_HPP

#include <string>
#include <utility>

namespace qrb::adapter
{

/** @brief Result returned by a typed message conversion. */
class ConversionResult final
{
public:
  static ConversionResult Success() { return ConversionResult(true, {}); }

  static ConversionResult Failure(std::string diagnostic)
  {
    if (diagnostic.empty())
      diagnostic = "message conversion failed";
    return ConversionResult(false, std::move(diagnostic));
  }

  explicit operator bool() const noexcept { return succeeded_; }
  bool Succeeded() const noexcept { return succeeded_; }
  const std::string & Diagnostic() const noexcept { return diagnostic_; }

private:
  ConversionResult(bool succeeded, std::string diagnostic)
    : succeeded_(succeeded), diagnostic_(std::move(diagnostic))
  {
  }

  bool succeeded_;
  std::string diagnostic_;
};

}  // namespace qrb::adapter

#endif  // QRB_ROS_ADAPTER_IPC_RUNTIME_CONVERSION_RESULT_HPP
