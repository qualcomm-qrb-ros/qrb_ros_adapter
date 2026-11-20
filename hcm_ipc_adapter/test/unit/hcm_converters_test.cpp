// Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
// SPDX-License-Identifier: BSD-3-Clause-Clear

#include <gtest/gtest.h>

#include <cstring>
#include <limits>
#include <string>
#include <vector>

#include "qrb_ros_adapter/hcm_ipc_adapter/flow_client/odometry_hcm_converter.hpp"
#include "qrb_ros_adapter/hcm_ipc_adapter/flow_client/point_cloud_hcm_converter.hpp"
#include "qrb_ros_adapter/hcm_ipc_adapter/flow_client/pose_hcm_converter.hpp"

using namespace qrb::adapter;

namespace sensor_proto = ::qrb::contract::sensor::v1;

namespace
{

PoseHcmContract MakePose()
{
  PoseHcmContract pose;
  pose.mutable_header()->mutable_stamp()->set_sec(1);
  pose.mutable_header()->mutable_stamp()->set_nanosec(23456789);
  pose.mutable_header()->set_frame_id("map");
  auto * value = pose.mutable_pose();
  value->mutable_pose()->mutable_position()->set_x(1.5);
  value->mutable_pose()->mutable_position()->set_y(-2.0);
  value->mutable_pose()->mutable_position()->set_z(0.25);
  value->mutable_pose()->mutable_orientation()->set_z(0.6);
  value->mutable_pose()->mutable_orientation()->set_w(0.8);
  for (int i = 0; i < 36; ++i)
    value->add_covariance(0.01 * i);
  return pose;
}

// 1 row x 4 points of 16 bytes, x/y/z FLOAT32.
constexpr std::size_t kCloudBytes = 64;

PointCloudHcmContract MakeCloud()
{
  PointCloudHcmContract cloud;
  cloud.mutable_header()->mutable_stamp()->set_sec(2);
  cloud.mutable_header()->mutable_stamp()->set_nanosec(42);
  cloud.mutable_header()->set_frame_id("lidar");
  cloud.set_height(1);
  cloud.set_width(4);
  cloud.set_point_step(16);
  cloud.set_row_step(64);
  cloud.set_is_dense(true);
  const char * names[] = {"x", "y", "z"};
  for (std::uint32_t i = 0; i < 3; ++i) {
    auto * field = cloud.add_fields();
    field->set_name(names[i]);
    field->set_offset(4 * i);
    field->set_datatype(sensor_proto::POINT_FIELD_TYPE_FLOAT32);
    field->set_count(1);
  }
  return cloud;
}

/** @brief A PointCloud descriptor over heap storage; converters only touch pBuf. */
struct HeapCloudDescriptor
{
  explicit HeapCloudDescriptor(std::size_t capacity, std::size_t offset = 0) : storage(capacity, 0)
  {
    descriptor.pBuf = storage.data();
    descriptor.size = capacity;
    descriptor.offset = offset;
  }

  std::vector<std::uint8_t> storage;
  PointCloudHcmDescriptor descriptor;
};

}  // namespace

// UT-HCM-03: Contract <-> HCM conversion keeps every field, rejects an invalid
// value before it reaches either side, and bounds the PointCloud payload copy on
// both ends.
TEST(HcmConverters, RoundTripEveryFieldAndRejectInvalidValuesOrPayloads)
{
  {
    const PoseHcmContract source = MakePose();
    PoseHcmDescriptor descriptor;
    ASSERT_TRUE((ConvertContractToHcm<PoseHcmContract, PoseHcmDescriptor>(
        source, DmaBufView{}, descriptor)));
    EXPECT_EQ(descriptor.timestampNs, 1023456789u);
    EXPECT_TRUE(descriptor.frameId == "map");
    EXPECT_DOUBLE_EQ(descriptor.poseWithCovariance.pose.position.y, -2.0);
    EXPECT_DOUBLE_EQ(descriptor.poseWithCovariance.covariance[35], 0.35);

    PoseHcmContract back;
    back.mutable_header()->set_frame_id("stale");
    ASSERT_TRUE((ConvertHcmToContract<PoseHcmContract, PoseHcmDescriptor>(descriptor, back)));
    EXPECT_EQ(back.SerializeAsString(), source.SerializeAsString());

    PoseHcmContract invalid = MakePose();
    invalid.mutable_pose()->mutable_pose()->mutable_position()->set_x(
        std::numeric_limits<double>::quiet_NaN());
    PoseHcmDescriptor untouched;
    ASSERT_EQ(untouched.frameId.Assign("untouched"), QC::QC_STATUS_OK);
    const ConversionResult in =
        ConvertContractToHcm<PoseHcmContract, PoseHcmDescriptor>(invalid, DmaBufView{}, untouched);
    EXPECT_FALSE(in);
    EXPECT_FALSE(in.Diagnostic().empty());
    EXPECT_TRUE(untouched.frameId == "untouched");

    // FrameId_t is fixed-size: a longer frame_id is rejected, never truncated.
    PoseHcmContract longFrame = MakePose();
    longFrame.mutable_header()->set_frame_id(
        std::string(QRB::Memory::QRB_FRAME_ID_MAX_LEN + 1, 'f'));
    EXPECT_FALSE((ConvertContractToHcm<PoseHcmContract, PoseHcmDescriptor>(
        longFrame, DmaBufView{}, untouched)));
    EXPECT_TRUE(untouched.frameId == "untouched");

    // A non-finite value produced on the HCM side must not be published to ROS.
    descriptor.poseWithCovariance.covariance[7] = std::numeric_limits<double>::infinity();
    PoseHcmContract out;
    EXPECT_FALSE((ConvertHcmToContract<PoseHcmContract, PoseHcmDescriptor>(descriptor, out)));
  }
  {
    std::vector<std::uint8_t> source(kCloudBytes + 32, 0xA5);
    HeapCloudDescriptor target(256, 16);
    ASSERT_TRUE((ConvertContractToHcm<PointCloudHcmContract, PointCloudHcmDescriptor>(
        MakeCloud(), DmaBufView{source.data(), source.size()}, target.descriptor)));

    EXPECT_EQ(target.descriptor.validSize, kCloudBytes);
    EXPECT_EQ(target.descriptor.timestampNs, 2000000042u);
    EXPECT_TRUE(target.descriptor.frameId == "lidar");
    EXPECT_EQ(target.descriptor.width, 4u);
    EXPECT_EQ(target.descriptor.rowStep, 64u);
    EXPECT_TRUE(target.descriptor.isDense);
    ASSERT_EQ(target.descriptor.numFields, 3u);
    EXPECT_TRUE(target.descriptor.fields[2].name == "z");
    EXPECT_EQ(target.descriptor.fields[2].offset, 8u);
    // Copy lands at the descriptor offset and stops at the layout size.
    EXPECT_EQ(target.storage[15], 0u);
    EXPECT_EQ(target.storage[16], 0xA5);
    EXPECT_EQ(target.storage[16 + kCloudBytes - 1], 0xA5);
    EXPECT_EQ(target.storage[16 + kCloudBytes], 0u);

    PointCloudHcmContract back;
    ASSERT_TRUE((ConvertHcmToContract<PointCloudHcmContract, PointCloudHcmDescriptor>(
        target.descriptor, back)));
    EXPECT_EQ(back.SerializeAsString(), MakeCloud().SerializeAsString());

    // ROS -> HCM: the host buffer and the source payload must both hold the layout.
    HeapCloudDescriptor small(kCloudBytes - 1);
    EXPECT_FALSE((ConvertContractToHcm<PointCloudHcmContract, PointCloudHcmDescriptor>(
        MakeCloud(), DmaBufView{source.data(), source.size()}, small.descriptor)));
    EXPECT_EQ(small.descriptor.validSize, 0u);
    HeapCloudDescriptor other(256);
    EXPECT_FALSE((ConvertContractToHcm<PointCloudHcmContract, PointCloudHcmDescriptor>(
        MakeCloud(), DmaBufView{source.data(), kCloudBytes - 1}, other.descriptor)));
    EXPECT_FALSE((ConvertContractToHcm<PointCloudHcmContract, PointCloudHcmDescriptor>(
        MakeCloud(), DmaBufView{nullptr, kCloudBytes}, other.descriptor)));

    // The descriptor holds at most QRB_POINT_CLOUD_MAX_FIELDS fields and
    // fixed-size names; a cloud beyond that is rejected before any write.
    PointCloudHcmContract tooManyFields = MakeCloud();
    tooManyFields.set_point_step(QRB::Memory::QRB_POINT_CLOUD_MAX_FIELDS + 1);
    tooManyFields.set_row_step(tooManyFields.point_step() * tooManyFields.width());
    tooManyFields.clear_fields();
    for (std::uint32_t i = 0; i <= QRB::Memory::QRB_POINT_CLOUD_MAX_FIELDS; ++i) {
      auto * field = tooManyFields.add_fields();
      field->set_name("f" + std::to_string(i));
      field->set_offset(i);
      field->set_datatype(sensor_proto::POINT_FIELD_TYPE_UINT8);
      field->set_count(1);
    }
    PointCloudHcmContract longName = MakeCloud();
    longName.mutable_fields(0)->set_name(
        std::string(QRB::Memory::QRB_POINT_FIELD_NAME_MAX_LEN + 1, 'n'));
    for (const PointCloudHcmContract * rejected : {&tooManyFields, &longName}) {
      HeapCloudDescriptor capacity(256);
      EXPECT_FALSE((ConvertContractToHcm<PointCloudHcmContract, PointCloudHcmDescriptor>(
          *rejected, DmaBufView{source.data(), source.size()}, capacity.descriptor)));
      EXPECT_EQ(capacity.descriptor.validSize, 0u);
      EXPECT_EQ(capacity.descriptor.numFields, 0u);
    }

    // HCM -> ROS: validSize must cover the declared layout and fit the buffer.
    PointCloudHcmContract out;
    target.descriptor.validSize = kCloudBytes - 1;
    EXPECT_FALSE((ConvertHcmToContract<PointCloudHcmContract, PointCloudHcmDescriptor>(
        target.descriptor, out)));
    target.descriptor.validSize = target.descriptor.size + 1;
    EXPECT_FALSE((ConvertHcmToContract<PointCloudHcmContract, PointCloudHcmDescriptor>(
        target.descriptor, out)));
  }
}

namespace
{

OdometryHcmContract MakeOdometry()
{
  OdometryHcmContract odom;
  odom.mutable_header()->mutable_stamp()->set_sec(3);
  odom.mutable_header()->mutable_stamp()->set_nanosec(7);
  odom.mutable_header()->set_frame_id("odom");
  odom.set_child_frame_id("base_link");
  auto * pose = odom.mutable_pose();
  pose->mutable_pose()->mutable_position()->set_x(4.0);
  pose->mutable_pose()->mutable_orientation()->set_z(0.6);
  pose->mutable_pose()->mutable_orientation()->set_w(0.8);
  auto * twist = odom.mutable_twist();
  twist->mutable_twist()->mutable_linear()->set_x(0.3);
  twist->mutable_twist()->mutable_angular()->set_z(-0.1);
  for (int i = 0; i < 36; ++i) {
    pose->add_covariance(0.01 * i);
    twist->add_covariance(0.02 * i);
  }
  return odom;
}

}  // namespace

// UT-HCM-03: Odometry keeps both frames, the pose and the twist with their
// covariances, and rejects what the fixed-size descriptor cannot hold.
TEST(HcmConverters, OdometryRoundTripsEveryFieldAndRejectsInvalidValues)
{
  const OdometryHcmContract source = MakeOdometry();
  OdometryHcmDescriptor descriptor;
  ASSERT_TRUE((ConvertContractToHcm<OdometryHcmContract, OdometryHcmDescriptor>(
      source, DmaBufView{}, descriptor)));
  EXPECT_EQ(descriptor.timestampNs, 3000000007u);
  EXPECT_TRUE(descriptor.frameId == "odom");
  EXPECT_TRUE(descriptor.childFrameId == "base_link");
  EXPECT_DOUBLE_EQ(descriptor.poseWithCovariance.pose.position.x, 4.0);
  EXPECT_DOUBLE_EQ(descriptor.poseWithCovariance.pose.orientation.w, 0.8);
  EXPECT_DOUBLE_EQ(descriptor.twistWithCovariance.twist.linear.x, 0.3);
  EXPECT_DOUBLE_EQ(descriptor.twistWithCovariance.twist.angular.z, -0.1);
  EXPECT_DOUBLE_EQ(descriptor.poseWithCovariance.covariance[35], 0.35);
  EXPECT_DOUBLE_EQ(descriptor.twistWithCovariance.covariance[35], 0.70);

  OdometryHcmContract back;
  back.set_child_frame_id("stale");
  ASSERT_TRUE((ConvertHcmToContract<OdometryHcmContract, OdometryHcmDescriptor>(descriptor, back)));
  EXPECT_EQ(back.SerializeAsString(), source.SerializeAsString());

  // A rejected message leaves the descriptor untouched.
  OdometryHcmDescriptor untouched;
  ASSERT_EQ(untouched.childFrameId.Assign("untouched"), QC::QC_STATUS_OK);
  OdometryHcmContract invalid = MakeOdometry();
  invalid.mutable_twist()->mutable_twist()->mutable_linear()->set_x(
      std::numeric_limits<double>::quiet_NaN());
  const ConversionResult in = ConvertContractToHcm<OdometryHcmContract, OdometryHcmDescriptor>(
      invalid, DmaBufView{}, untouched);
  EXPECT_FALSE(in);
  EXPECT_FALSE(in.Diagnostic().empty());
  EXPECT_TRUE(untouched.childFrameId == "untouched");

  // FrameId_t is fixed-size: either frame longer than it is rejected, never truncated.
  const std::string tooLong(QRB::Memory::QRB_FRAME_ID_MAX_LEN + 1, 'f');
  OdometryHcmContract longFrame = MakeOdometry();
  longFrame.mutable_header()->set_frame_id(tooLong);
  OdometryHcmContract longChild = MakeOdometry();
  longChild.set_child_frame_id(tooLong);
  for (const OdometryHcmContract * rejected : {&longFrame, &longChild}) {
    EXPECT_FALSE((ConvertContractToHcm<OdometryHcmContract, OdometryHcmDescriptor>(
        *rejected, DmaBufView{}, untouched)));
    EXPECT_TRUE(untouched.childFrameId == "untouched");
  }

  // A non-finite twist covariance produced on the HCM side must not reach ROS.
  descriptor.twistWithCovariance.covariance[3] = std::numeric_limits<double>::infinity();
  OdometryHcmContract out;
  EXPECT_FALSE((ConvertHcmToContract<OdometryHcmContract, OdometryHcmDescriptor>(descriptor, out)));
}
