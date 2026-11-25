// Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
// SPDX-License-Identifier: BSD-3-Clause-Clear

#include <gtest/gtest.h>

#include <cstring>
#include <limits>
#include <string>
#include <vector>

#include "qrb_ros_adapter/hcm_ipc_adapter/flow_client/joint_state_hcm_converter.hpp"
#include "qrb_ros_adapter/hcm_ipc_adapter/flow_client/odometry_hcm_converter.hpp"
#include "qrb_ros_adapter/hcm_ipc_adapter/flow_client/point_cloud_hcm_converter.hpp"
#include "qrb_ros_adapter/hcm_ipc_adapter/flow_client/pose_hcm_converter.hpp"
#include "qrb_ros_adapter/hcm_ipc_adapter/flow_client/tf_message_hcm_converter.hpp"
#include "qrb_ros_adapter/hcm_ipc_adapter/flow_client/twist_hcm_converter.hpp"
#include "qrb_ros_adapter/ipc_runtime/contract_limits.hpp"

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

namespace
{

TwistHcmContract MakeTwist()
{
  TwistHcmContract twist;
  twist.mutable_header()->mutable_stamp()->set_sec(2);
  twist.mutable_header()->mutable_stamp()->set_nanosec(9);
  twist.mutable_header()->set_frame_id("base_link");
  twist.mutable_twist()->mutable_linear()->set_x(0.5);
  twist.mutable_twist()->mutable_linear()->set_y(-0.2);
  twist.mutable_twist()->mutable_angular()->set_z(0.25);
  return twist;
}

}  // namespace

// UT-HCM-04: TwistStamped keeps its frame, stamp and twist, and rejects what the
// fixed-size descriptor cannot hold.
TEST(HcmConverters, TwistRoundTripsEveryFieldAndRejectsInvalidValues)
{
  const TwistHcmContract source = MakeTwist();
  TwistHcmDescriptor descriptor;
  ASSERT_TRUE((ConvertContractToHcm<TwistHcmContract, TwistHcmDescriptor>(
      source, DmaBufView{}, descriptor)));
  EXPECT_EQ(descriptor.timestampNs, 2000000009u);
  EXPECT_TRUE(descriptor.frameId == "base_link");
  EXPECT_DOUBLE_EQ(descriptor.twist.linear.x, 0.5);
  EXPECT_DOUBLE_EQ(descriptor.twist.linear.y, -0.2);
  EXPECT_DOUBLE_EQ(descriptor.twist.angular.z, 0.25);

  TwistHcmContract back;
  back.mutable_header()->set_frame_id("stale");
  ASSERT_TRUE((ConvertHcmToContract<TwistHcmContract, TwistHcmDescriptor>(descriptor, back)));
  EXPECT_EQ(back.SerializeAsString(), source.SerializeAsString());

  // A rejected message leaves the descriptor untouched.
  TwistHcmDescriptor untouched;
  ASSERT_EQ(untouched.frameId.Assign("untouched"), QC::QC_STATUS_OK);
  TwistHcmContract invalid = MakeTwist();
  invalid.mutable_twist()->mutable_angular()->set_x(std::numeric_limits<double>::quiet_NaN());
  const ConversionResult in =
      ConvertContractToHcm<TwistHcmContract, TwistHcmDescriptor>(invalid, DmaBufView{}, untouched);
  EXPECT_FALSE(in);
  EXPECT_FALSE(in.Diagnostic().empty());
  EXPECT_TRUE(untouched.frameId == "untouched");

  // FrameId_t is fixed-size: a longer frame_id is rejected, never truncated.
  TwistHcmContract longFrame = MakeTwist();
  longFrame.mutable_header()->set_frame_id(std::string(QRB::Memory::QRB_FRAME_ID_MAX_LEN + 1, 'f'));
  EXPECT_FALSE((ConvertContractToHcm<TwistHcmContract, TwistHcmDescriptor>(
      longFrame, DmaBufView{}, untouched)));
  EXPECT_TRUE(untouched.frameId == "untouched");

  // A non-finite twist produced on the HCM side must not reach ROS.
  descriptor.twist.linear.z = std::numeric_limits<double>::infinity();
  TwistHcmContract out;
  EXPECT_FALSE((ConvertHcmToContract<TwistHcmContract, TwistHcmDescriptor>(descriptor, out)));
}

namespace
{

TFMessageHcmContract MakeTFMessage(int count)
{
  TFMessageHcmContract message;
  for (int i = 0; i < count; ++i) {
    auto * transform = message.add_transforms();
    transform->mutable_header()->mutable_stamp()->set_sec(4 + i);
    transform->mutable_header()->mutable_stamp()->set_nanosec(11u * static_cast<unsigned>(i));
    transform->mutable_header()->set_frame_id(i == 0 ? "map" : "odom");
    transform->set_child_frame_id(i == 0 ? "odom" : "base_link");
    transform->mutable_transform()->mutable_translation()->set_x(1.5 * i);
    transform->mutable_transform()->mutable_translation()->set_z(-0.25);
    transform->mutable_transform()->mutable_rotation()->set_z(0.6);
    transform->mutable_transform()->mutable_rotation()->set_w(0.8);
  }
  return message;
}

/** @brief A TFMessage descriptor over heap storage for `capacity` transforms. */
struct HeapTFDescriptor
{
  explicit HeapTFDescriptor(std::uint32_t capacity, std::size_t offset = 0)
    : storage(offset + TFMessageHcmDescriptor::CalcBufferSize(capacity), 0)
  {
    descriptor.pBuf = storage.data();
    descriptor.size = storage.size();
    descriptor.offset = offset;
    descriptor.allocatorType = QC::Memory::QC_MEMORY_ALLOCATOR_HEAP;
  }

  std::vector<std::uint8_t> storage;
  TFMessageHcmDescriptor descriptor;
};

}  // namespace

// UT-HCM-05: TFMessage writes every transform into the descriptor's buffer,
// reads them back unchanged, and rejects what that buffer cannot hold.
TEST(HcmConverters, TFMessageRoundTripsThroughTheBufferAndRejectsWhatItCannotHold)
{
  const TFMessageHcmContract source = MakeTFMessage(3);
  // An odd offset: the converter must not need the array to be aligned.
  HeapTFDescriptor heap(3, 5);
  ASSERT_TRUE((ConvertContractToHcm<TFMessageHcmContract, TFMessageHcmDescriptor>(
      source, DmaBufView{}, heap.descriptor)));
  EXPECT_EQ(heap.descriptor.transformCount, 3u);
  EXPECT_EQ(heap.descriptor.validSize, TFMessageHcmDescriptor::CalcBufferSize(3));
  QRB::Memory::TransformStamped_t second{};
  std::memcpy(&second, static_cast<std::uint8_t *>(heap.descriptor.GetDataPtr()) + sizeof(second),
      sizeof(second));
  EXPECT_EQ(second.timestampNs, 5000000011u);
  EXPECT_TRUE(second.frameId == "odom");
  EXPECT_TRUE(second.childFrameId == "base_link");
  EXPECT_DOUBLE_EQ(second.transform.translation.x, 1.5);
  EXPECT_DOUBLE_EQ(second.transform.rotation.w, 0.8);

  TFMessageHcmContract back;
  back.add_transforms();  // stale
  ASSERT_TRUE(
      (ConvertHcmToContract<TFMessageHcmContract, TFMessageHcmDescriptor>(heap.descriptor, back)));
  EXPECT_EQ(back.SerializeAsString(), source.SerializeAsString());

  // An empty message needs no buffer.
  TFMessageHcmDescriptor bufferless;
  ASSERT_TRUE((ConvertContractToHcm<TFMessageHcmContract, TFMessageHcmDescriptor>(
      TFMessageHcmContract{}, DmaBufView{}, bufferless)));
  EXPECT_EQ(bufferless.transformCount, 0u);
  ASSERT_TRUE(
      (ConvertHcmToContract<TFMessageHcmContract, TFMessageHcmDescriptor>(bufferless, back)));
  EXPECT_EQ(back.transforms_size(), 0);

  // A rejected message leaves the descriptor and its buffer untouched.
  HeapTFDescriptor untouched(2);
  untouched.descriptor.transformCount = 1;
  const std::vector<std::uint8_t> before = untouched.storage;
  TFMessageHcmContract tooMany = MakeTFMessage(3);
  TFMessageHcmContract invalid = MakeTFMessage(2);
  invalid.mutable_transforms(1)->mutable_transform()->mutable_rotation()->set_x(
      std::numeric_limits<double>::quiet_NaN());
  TFMessageHcmContract longChild = MakeTFMessage(2);
  longChild.mutable_transforms(1)->set_child_frame_id(
      std::string(QRB::Memory::QRB_FRAME_ID_MAX_LEN + 1, 'c'));
  for (const TFMessageHcmContract * rejected : {&tooMany, &invalid, &longChild}) {
    const ConversionResult result =
        ConvertContractToHcm<TFMessageHcmContract, TFMessageHcmDescriptor>(
            *rejected, DmaBufView{}, untouched.descriptor);
    EXPECT_FALSE(result);
    EXPECT_FALSE(result.Diagnostic().empty());
    EXPECT_EQ(untouched.descriptor.transformCount, 1u);
    EXPECT_EQ(untouched.storage, before);
  }

  // A buffer the CPU cannot reach is rejected, like one of another process.
  HeapTFDescriptor foreign(3);
  foreign.descriptor.allocatorType = QC::Memory::QC_MEMORY_ALLOCATOR_LAST;
  EXPECT_FALSE((ConvertContractToHcm<TFMessageHcmContract, TFMessageHcmDescriptor>(
      source, DmaBufView{}, foreign.descriptor)));

  // A transformCount beyond the buffer, or a non-finite transform from HCM, never reaches ROS.
  HeapTFDescriptor overCount(2);
  overCount.descriptor.transformCount = 3;
  TFMessageHcmContract out;
  EXPECT_FALSE((ConvertHcmToContract<TFMessageHcmContract, TFMessageHcmDescriptor>(
      overCount.descriptor, out)));
  QRB::Memory::TransformStamped_t bad{};
  bad.transform.translation.y = std::numeric_limits<double>::infinity();
  std::memcpy(heap.descriptor.GetDataPtr(), &bad, sizeof(bad));
  EXPECT_FALSE(
      (ConvertHcmToContract<TFMessageHcmContract, TFMessageHcmDescriptor>(heap.descriptor, out)));
}

namespace
{

JointStateHcmContract MakeJointState(std::uint32_t count)
{
  JointStateHcmContract message;
  message.mutable_header()->mutable_stamp()->set_sec(2);
  message.mutable_header()->mutable_stamp()->set_nanosec(9);
  message.mutable_header()->set_frame_id("base_link");
  for (std::uint32_t i = 0; i < count; ++i) {
    message.add_name("joint_" + std::to_string(i));
    message.add_position(-0.5 * i);
    message.add_velocity(0.25 * i);
    message.add_effort(0.0);
  }
  return message;
}

/** @brief Constructed and aligned host storage for a JointState descriptor. */
struct HeapJointDescriptor
{
  explicit HeapJointDescriptor(std::uint32_t count) : storage(count)
  {
    descriptor.pBuf = storage.data();
    descriptor.size = JointStateHcmDescriptor::CalcBufferSize(count);
    descriptor.allocatorType = QC::Memory::QC_MEMORY_ALLOCATOR_HEAP;
  }

  std::vector<QRB::Memory::JointState_t> storage;
  JointStateHcmDescriptor descriptor;
};

}  // namespace

// UT-HCM-06: JointState keeps the parallel arrays and bounds its aligned host records.
TEST(HcmConverters, JointStateRoundTripsThroughHostStorageAndRejectsInvalidFrames)
{
  for (std::uint32_t count :
      {0u, 3u, static_cast<std::uint32_t>(contract_limits::kMaxJointCount)}) {
    SCOPED_TRACE(count);
    const JointStateHcmContract source = MakeJointState(count);
    HeapJointDescriptor heap(count);
    ASSERT_TRUE((ConvertContractToHcm<JointStateHcmContract, JointStateHcmDescriptor>(
        source, DmaBufView{}, heap.descriptor)));
    EXPECT_EQ(heap.descriptor.jointCount, count);
    EXPECT_EQ(heap.descriptor.validSize, JointStateHcmDescriptor::CalcBufferSize(count));
    EXPECT_EQ(heap.descriptor.timestampNs, 2000000009u);
    EXPECT_TRUE(heap.descriptor.frameId == "base_link");
    if (count != 0) {
      ASSERT_NE(heap.descriptor.GetJointPtr(count - 1), nullptr);
      EXPECT_DOUBLE_EQ(heap.storage.back().position, source.position(count - 1));
      EXPECT_DOUBLE_EQ(heap.storage.back().velocity, source.velocity(count - 1));
      EXPECT_DOUBLE_EQ(heap.storage.back().effort, 0.0);
    }
    JointStateHcmContract back;
    back.add_name("stale");
    ASSERT_TRUE((ConvertHcmToContract<JointStateHcmContract, JointStateHcmDescriptor>(
        heap.descriptor, back)));
    EXPECT_EQ(back.SerializeAsString(), source.SerializeAsString());
  }

  HeapJointDescriptor heap(3);
  const JointStateHcmContract source = MakeJointState(3);
  ASSERT_TRUE((ConvertContractToHcm<JointStateHcmContract, JointStateHcmDescriptor>(
      source, DmaBufView{}, heap.descriptor)));
  const std::string before(
      static_cast<const char *>(heap.descriptor.GetDataPtr()), heap.descriptor.validSize);
  JointStateHcmContract missing = source;
  missing.clear_velocity();
  JointStateHcmContract invalid = source;
  invalid.set_effort(1, std::numeric_limits<double>::quiet_NaN());
  JointStateHcmContract longName = source;
  longName.set_name(1, std::string(contract_limits::kMaxJointNameLength + 1, 'n'));
  JointStateHcmContract negativeTime = source;
  negativeTime.mutable_header()->mutable_stamp()->set_sec(-1);
  for (const JointStateHcmContract * rejected : {&missing, &invalid, &longName, &negativeTime}) {
    EXPECT_FALSE((ConvertContractToHcm<JointStateHcmContract, JointStateHcmDescriptor>(
        *rejected, DmaBufView{}, heap.descriptor)));
    EXPECT_EQ(heap.descriptor.jointCount, 3u);
    EXPECT_EQ(std::string(static_cast<const char *>(heap.descriptor.GetDataPtr()),
                  heap.descriptor.validSize),
        before);
  }
  HeapJointDescriptor small(2);
  EXPECT_FALSE((ConvertContractToHcm<JointStateHcmContract, JointStateHcmDescriptor>(
      source, DmaBufView{}, small.descriptor)));
  heap.descriptor.offset = 1;  // enough capacity is not enough: records must also be aligned
  heap.descriptor.size += 1;
  EXPECT_FALSE((ConvertContractToHcm<JointStateHcmContract, JointStateHcmDescriptor>(
      source, DmaBufView{}, heap.descriptor)));
  heap.descriptor.offset = 0;
  --heap.descriptor.size;

  JointStateHcmContract out = source;
  --heap.descriptor.validSize;
  EXPECT_FALSE(
      (ConvertHcmToContract<JointStateHcmContract, JointStateHcmDescriptor>(heap.descriptor, out)));
  EXPECT_EQ(out.SerializeAsString(), source.SerializeAsString());
  ++heap.descriptor.validSize;
  heap.storage[1].velocity = std::numeric_limits<double>::infinity();
  EXPECT_FALSE(
      (ConvertHcmToContract<JointStateHcmContract, JointStateHcmDescriptor>(heap.descriptor, out)));
  EXPECT_EQ(out.SerializeAsString(), source.SerializeAsString());
}
