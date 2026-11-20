// Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
// SPDX-License-Identifier: BSD-3-Clause-Clear

#include "qrb_ros_adapter/ipc_runtime/contract_validation.hpp"

#include <cmath>
#include <limits>

#include "qrb_ros_adapter/ipc_runtime/contract_time.hpp"

namespace qrb::adapter
{

namespace
{

ConversionResult Fail(const char * message)
{
  return ConversionResult::Failure(message);
}

bool IsFinitePose(const qrb::contract::geometry::v1::Pose & pose)
{
  const auto & position = pose.position();
  const auto & orientation = pose.orientation();
  return std::isfinite(position.x()) && std::isfinite(position.y()) &&
         std::isfinite(position.z()) && std::isfinite(orientation.x()) &&
         std::isfinite(orientation.y()) && std::isfinite(orientation.z()) &&
         std::isfinite(orientation.w());
}

bool IsFiniteVector3(const qrb::contract::geometry::v1::Vector3 & vector)
{
  return std::isfinite(vector.x()) && std::isfinite(vector.y()) && std::isfinite(vector.z());
}

/** @brief A 6x6 covariance is either unknown (empty) or 36 finite values. */
ConversionResult ValidateCovariance(const google::protobuf::RepeatedField<double> & covariance,
    const char * sizeMessage,
    const char * finiteMessage)
{
  if (covariance.size() != 0 && covariance.size() != 36)
    return Fail(sizeMessage);
  for (double value : covariance) {
    if (!std::isfinite(value))
      return Fail(finiteMessage);
  }
  return ConversionResult::Success();
}

/** @brief The rules for a PoseWithCovariance, wherever a Contract embeds one. */
ConversionResult ValidatePoseWithCovariance(
    const qrb::contract::geometry::v1::PoseWithCovariance & value)
{
  if (!value.has_pose() || !IsFinitePose(value.pose()))
    return Fail("pose contains missing or non-finite values");
  const auto & orientation = value.pose().orientation();
  if (orientation.x() == 0.0 && orientation.y() == 0.0 && orientation.z() == 0.0 &&
      orientation.w() == 0.0)
    return Fail("orientation quaternion must be non-zero");
  return ValidateCovariance(value.covariance(), "covariance must be empty or contain 36 values",
      "covariance contains a non-finite value");
}

std::uint32_t PointFieldElementSize(qrb::contract::sensor::v1::PointFieldType type)
{
  using namespace qrb::contract::sensor::v1;
  switch (type) {
    case POINT_FIELD_TYPE_INT8:
    case POINT_FIELD_TYPE_UINT8:
      return 1;
    case POINT_FIELD_TYPE_INT16:
    case POINT_FIELD_TYPE_UINT16:
      return 2;
    case POINT_FIELD_TYPE_INT32:
    case POINT_FIELD_TYPE_UINT32:
    case POINT_FIELD_TYPE_FLOAT32:
      return 4;
    case POINT_FIELD_TYPE_FLOAT64:
      return 8;
    default:
      return 0;
  }
}

}  // namespace

ConversionResult ValidateFrameId(const std::string & frameId)
{
  if (frameId.size() > contract_limits::kMaxFrameIdLength)
    return Fail("frame_id exceeds the contract limit");
  if (!frameId.empty() && frameId.front() == '/')
    return Fail("frame_id must not start with '/'");
  if (frameId.find('\0') != std::string::npos)
    return Fail("frame_id must not contain NUL");
  return ConversionResult::Success();
}

ConversionResult ValidateHeader(const qrb::contract::common::v1::Header & header)
{
  ConversionResult frameId = ValidateFrameId(header.frame_id());
  if (!frameId)
    return frameId;
  if (header.stamp().nanosec() >= kNanosPerSecond)
    return Fail("header.stamp.nanosec must be below one second");
  return ConversionResult::Success();
}

ConversionResult ValidatePoseWithCovarianceStamped(
    const qrb::contract::geometry::v1::PoseWithCovarianceStamped & pose)
{
  ConversionResult header = ValidateHeader(pose.header());
  if (!header)
    return header;
  if (!pose.has_pose())
    return Fail("pose is required");
  return ValidatePoseWithCovariance(pose.pose());
}

ConversionResult ValidateOdometry(const qrb::contract::nav::v1::Odometry & odometry)
{
  ConversionResult header = ValidateHeader(odometry.header());
  if (!header)
    return header;
  ConversionResult childFrameId = ValidateFrameId(odometry.child_frame_id());
  if (!childFrameId)
    return ConversionResult::Failure("child_frame_id: " + childFrameId.Diagnostic());

  if (!odometry.has_pose())
    return Fail("pose is required");
  ConversionResult pose = ValidatePoseWithCovariance(odometry.pose());
  if (!pose)
    return pose;

  if (!odometry.has_twist())
    return Fail("twist is required");
  const auto & value = odometry.twist();
  if (!value.has_twist() || !IsFiniteVector3(value.twist().linear()) ||
      !IsFiniteVector3(value.twist().angular()))
    return Fail("twist contains missing or non-finite values");
  return ValidateCovariance(value.covariance(),
      "twist covariance must be empty or contain 36 values",
      "twist covariance contains a non-finite value");
}

ConversionResult ValidatePointCloud2(const qrb::contract::sensor::v1::PointCloud2 & cloud,
    std::uint64_t externalPayloadSize)
{
  ConversionResult header = ValidateHeader(cloud.header());
  if (!header)
    return header;
  if (cloud.height() == 0 || cloud.width() == 0)
    return Fail("point cloud dimensions must be non-zero");
  if (cloud.point_step() == 0)
    return Fail("point_step must be non-zero");
  if (cloud.width() > std::numeric_limits<std::uint32_t>::max() / cloud.point_step())
    return Fail("point_step * width overflows");
  const std::uint32_t minimumRowStep = cloud.point_step() * cloud.width();
  if (cloud.row_step() < minimumRowStep)
    return Fail("row_step is smaller than point_step * width");
  if (cloud.height() > std::numeric_limits<std::uint64_t>::max() / cloud.row_step())
    return Fail("row_step * height overflows");
  const std::uint64_t payloadSize = static_cast<std::uint64_t>(cloud.row_step()) * cloud.height();
  if (externalPayloadSize < payloadSize)
    return Fail("external payload is smaller than the point cloud layout");
  if (!cloud.data().empty() && cloud.data().size() != payloadSize)
    return Fail("embedded data size does not match the point cloud layout");

  if (static_cast<std::size_t>(cloud.fields_size()) > contract_limits::kMaxPointFieldCount)
    return Fail("point field count is outside the contract limit");
  for (const auto & field : cloud.fields()) {
    if (field.name().empty() || field.name().size() > contract_limits::kMaxPointFieldNameLength)
      return Fail("point field name is outside the contract limit");
    if (field.name().find('\0') != std::string::npos)
      return Fail("point field name must not contain NUL");
    const std::uint32_t elementSize = PointFieldElementSize(field.datatype());
    if (elementSize == 0)
      return Fail("point field datatype is invalid");
    if (field.count() == 0 || field.count() > contract_limits::kMaxPointFieldElementCount)
      return Fail("point field element count is outside the contract limit");
    const std::uint64_t fieldEnd = static_cast<std::uint64_t>(field.offset()) +
                                   static_cast<std::uint64_t>(elementSize) * field.count();
    if (fieldEnd > cloud.point_step())
      return Fail("point field extends beyond point_step");
  }
  return ConversionResult::Success();
}

}  // namespace qrb::adapter
