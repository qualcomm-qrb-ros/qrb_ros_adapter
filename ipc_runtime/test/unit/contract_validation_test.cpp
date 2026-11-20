// Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
// SPDX-License-Identifier: BSD-3-Clause-Clear

#include "qrb_ros_adapter/ipc_runtime/contract_validation.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <string>

using namespace qrb::adapter;

namespace geometry_proto = ::qrb::contract::geometry::v1;
namespace sensor_proto = ::qrb::contract::sensor::v1;

namespace
{

geometry_proto::PoseWithCovarianceStamped MakePose()
{
  geometry_proto::PoseWithCovarianceStamped pose;
  pose.mutable_header()->mutable_stamp()->set_sec(1);
  pose.mutable_header()->set_frame_id("map");
  auto * value = pose.mutable_pose();
  value->mutable_pose()->mutable_position()->set_x(1.0);
  value->mutable_pose()->mutable_orientation()->set_w(1.0);
  return pose;
}

// 2 rows x 4 points of 16 bytes: x/y/z FLOAT32 and one UINT8 intensity.
sensor_proto::PointCloud2 MakeCloud()
{
  sensor_proto::PointCloud2 cloud;
  cloud.mutable_header()->set_frame_id("lidar");
  cloud.set_height(2);
  cloud.set_width(4);
  cloud.set_point_step(16);
  cloud.set_row_step(64);
  const char * names[] = {"x", "y", "z"};
  for (std::uint32_t i = 0; i < 3; ++i) {
    auto * field = cloud.add_fields();
    field->set_name(names[i]);
    field->set_offset(4 * i);
    field->set_datatype(sensor_proto::POINT_FIELD_TYPE_FLOAT32);
    field->set_count(1);
  }
  auto * intensity = cloud.add_fields();
  intensity->set_name("intensity");
  intensity->set_offset(12);
  intensity->set_datatype(sensor_proto::POINT_FIELD_TYPE_UINT8);
  intensity->set_count(4);
  return cloud;
}

constexpr std::uint64_t kCloudBytes = 128;

}  // namespace

// UT-IPC-03: Contract validation is the last line before a value reaches either
// side; a PointCloud layout must fit the external payload exactly as declared.
TEST(ContractValidation, RejectsInvalidPosesAndLayoutsThePayloadCannotBack)
{
  geometry_proto::PoseWithCovarianceStamped full = MakePose();
  EXPECT_TRUE(ValidatePoseWithCovarianceStamped(full));
  for (int i = 0; i < 36; ++i)
    full.mutable_pose()->add_covariance(0.1);
  EXPECT_TRUE(ValidatePoseWithCovarianceStamped(full));

  using Pose = geometry_proto::PoseWithCovarianceStamped;
  const struct
  {
    const char * name;
    void (*mutate)(Pose &);
  } cases[] = {
      {"NonFinitePosition",
          [](Pose & p) {
            p.mutable_pose()->mutable_pose()->mutable_position()->set_y(
                std::numeric_limits<double>::quiet_NaN());
          }},
      {"ZeroQuaternion",
          [](Pose & p) { p.mutable_pose()->mutable_pose()->mutable_orientation()->set_w(0.0); }},
      {"PartialCovariance",
          [](Pose & p) {
            for (int i = 0; i < 35; ++i) {
              p.mutable_pose()->add_covariance(0);
            }
          }},
      {"MissingPose", [](Pose & p) { p.clear_pose(); }},
      {"AbsoluteFrameId", [](Pose & p) { p.mutable_header()->set_frame_id("/map"); }},
      {"NanosecOverflow",
          [](Pose & p) { p.mutable_header()->mutable_stamp()->set_nanosec(1000000000U); }},
  };
  for (const auto & c : cases) {
    SCOPED_TRACE(c.name);
    Pose pose = MakePose();
    c.mutate(pose);
    const ConversionResult rejected = ValidatePoseWithCovarianceStamped(pose);
    EXPECT_FALSE(rejected);
    EXPECT_FALSE(rejected.Diagnostic().empty());
  }

  EXPECT_TRUE(ValidatePointCloud2(MakeCloud(), kCloudBytes));
  // A larger DMA-BUF is fine; the layout addresses only its prefix.
  EXPECT_TRUE(ValidatePointCloud2(MakeCloud(), kCloudBytes + 4096));

  using Cloud = sensor_proto::PointCloud2;
  const struct
  {
    const char * name;
    void (*mutate)(Cloud &);
    std::uint64_t payloadBytes;
  } clouds[] = {
      {"PayloadTooSmall", [](Cloud &) {}, kCloudBytes - 1},
      {"NanosecOverflow",
          [](Cloud & c) { c.mutable_header()->mutable_stamp()->set_nanosec(1000000000U); },
          kCloudBytes},
      {"WidthOverflow", [](Cloud & c) { c.set_width(std::numeric_limits<std::uint32_t>::max()); },
          std::numeric_limits<std::uint64_t>::max()},
      {"FieldBeyondPointStep", [](Cloud & c) { c.mutable_fields(3)->set_count(5); }, kCloudBytes},
      {"UnknownDatatype",
          [](Cloud & c) {
            c.mutable_fields(0)->set_datatype(sensor_proto::POINT_FIELD_TYPE_UNSPECIFIED);
          },
          kCloudBytes},
  };
  for (const auto & c : clouds) {
    SCOPED_TRACE(c.name);
    Cloud cloud = MakeCloud();
    c.mutate(cloud);
    const ConversionResult result = ValidatePointCloud2(cloud, c.payloadBytes);
    EXPECT_FALSE(result);
    EXPECT_FALSE(result.Diagnostic().empty());
  }
}

namespace
{

qrb::contract::nav::v1::Odometry MakeOdometry()
{
  qrb::contract::nav::v1::Odometry odom;
  odom.mutable_header()->mutable_stamp()->set_sec(1);
  odom.mutable_header()->set_frame_id("odom");
  odom.set_child_frame_id("base_link");
  odom.mutable_pose()->mutable_pose()->mutable_orientation()->set_w(1.0);
  odom.mutable_twist()->mutable_twist()->mutable_linear()->set_x(0.3);
  return odom;
}

}  // namespace

// UT-IPC-03: Odometry shares the Pose rules for its pose and adds the same
// finite / 36-value rules for its twist and a frame_id check for child_frame_id.
TEST(ContractValidation, RejectsInvalidOdometry)
{
  qrb::contract::nav::v1::Odometry full = MakeOdometry();
  EXPECT_TRUE(ValidateOdometry(full));
  for (int i = 0; i < 36; ++i) {
    full.mutable_pose()->add_covariance(0.1);
    full.mutable_twist()->add_covariance(0.2);
  }
  EXPECT_TRUE(ValidateOdometry(full));
  // An empty child_frame_id is allowed, as an empty frame_id is.
  qrb::contract::nav::v1::Odometry noChild = MakeOdometry();
  noChild.clear_child_frame_id();
  EXPECT_TRUE(ValidateOdometry(noChild));

  using Odom = qrb::contract::nav::v1::Odometry;
  const struct
  {
    const char * name;
    void (*mutate)(Odom &);
  } cases[] = {
      {"MissingPose", [](Odom & o) { o.clear_pose(); }},
      {"MissingTwist", [](Odom & o) { o.clear_twist(); }},
      {"ZeroQuaternion",
          [](Odom & o) { o.mutable_pose()->mutable_pose()->mutable_orientation()->set_w(0.0); }},
      {"NonFiniteLinear",
          [](Odom & o) {
            o.mutable_twist()->mutable_twist()->mutable_linear()->set_y(
                std::numeric_limits<double>::quiet_NaN());
          }},
      {"NonFiniteAngular",
          [](Odom & o) {
            o.mutable_twist()->mutable_twist()->mutable_angular()->set_z(
                std::numeric_limits<double>::infinity());
          }},
      {"PartialTwistCovariance",
          [](Odom & o) {
            for (int i = 0; i < 35; ++i)
              o.mutable_twist()->add_covariance(0);
          }},
      {"NonFiniteTwistCovariance",
          [](Odom & o) {
            for (int i = 0; i < 36; ++i)
              o.mutable_twist()->add_covariance(
                  i == 7 ? std::numeric_limits<double>::infinity() : 0);
          }},
      {"AbsoluteChildFrameId", [](Odom & o) { o.set_child_frame_id("/base_link"); }},
      {"AbsoluteFrameId", [](Odom & o) { o.mutable_header()->set_frame_id("/odom"); }},
      {"NanosecOverflow",
          [](Odom & o) { o.mutable_header()->mutable_stamp()->set_nanosec(1000000000U); }},
  };
  for (const auto & c : cases) {
    SCOPED_TRACE(c.name);
    Odom odom = MakeOdometry();
    c.mutate(odom);
    const ConversionResult rejected = ValidateOdometry(odom);
    EXPECT_FALSE(rejected);
    EXPECT_FALSE(rejected.Diagnostic().empty());
  }
}
