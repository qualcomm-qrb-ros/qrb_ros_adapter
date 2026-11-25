// Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
// SPDX-License-Identifier: BSD-3-Clause-Clear

#include "qrb_ros_adapter/hcm_ipc_adapter/flow_client/hcm_dma_buffer.hpp"

#include <gtest/gtest.h>
#include <unistd.h>

#include <climits>
#include <cstdint>
#include <vector>

#include "QC/Infras/Memory/BufferDescriptor.hpp"
#include "qrb_ros_adapter/hcm_ipc_adapter/flow_client/point_cloud_hcm_converter.hpp"

using namespace qrb::adapter;

namespace sensor_proto = ::qrb::contract::sensor::v1;

// The inherited buffer is classified before the CPU touches it; sync calls with an
// fd of -1 succeed without work.
TEST(HcmDmaBuffer, ClassifiesTheInheritedBufferAndNoOpsSyncForFdMinusOne)
{
  char storage[64] = {};
  int fd = 0;

  QC::Memory::QCBufferDescriptorBase_t base;
  EXPECT_EQ(InspectHcmBuffer(base, fd), HcmBufferAccess::kNone);
  EXPECT_EQ(fd, -1);

  QC::Memory::BufferDescriptor_t buffer;
  buffer.pBuf = nullptr;
  EXPECT_EQ(InspectHcmBuffer(buffer, fd), HcmBufferAccess::kNone);

  buffer.pBuf = storage;
  buffer.size = sizeof(storage);
  buffer.offset = 0;
  buffer.allocatorType = QC::Memory::QC_MEMORY_ALLOCATOR_HEAP;
  fd = 0;
  EXPECT_EQ(InspectHcmBuffer(buffer, fd), HcmBufferAccess::kHostMemory);
  EXPECT_EQ(fd, -1);

  buffer.offset = sizeof(storage) + 1;
  EXPECT_EQ(InspectHcmBuffer(buffer, fd), HcmBufferAccess::kUnusable);
  buffer.offset = 0;

  buffer.allocatorType = QC::Memory::QC_MEMORY_ALLOCATOR_DMA;
  buffer.dmaHandle = 5;
  buffer.pid = 0;
  EXPECT_EQ(InspectHcmBuffer(buffer, fd), HcmBufferAccess::kLocalDmaBuf);
  EXPECT_EQ(fd, 5);
  buffer.pid = ::getpid();
  EXPECT_EQ(InspectHcmBuffer(buffer, fd), HcmBufferAccess::kLocalDmaBuf);

  buffer.pid = ::getpid() + 1;
  EXPECT_EQ(InspectHcmBuffer(buffer, fd), HcmBufferAccess::kUnusable);
  EXPECT_EQ(fd, -1);
  buffer.pid = 0;

  // Any DMA allocator, not only the generic one.
  buffer.allocatorType = QC::Memory::QC_MEMORY_ALLOCATOR_DMA_GPU;
  EXPECT_EQ(InspectHcmBuffer(buffer, fd), HcmBufferAccess::kLocalDmaBuf);
  EXPECT_EQ(fd, 5);
  buffer.allocatorType = QC::Memory::QC_MEMORY_ALLOCATOR_DMA;

  buffer.dmaHandle = static_cast<std::uint64_t>(INT_MAX) + 1;
  EXPECT_EQ(InspectHcmBuffer(buffer, fd), HcmBufferAccess::kUnusable);
  buffer.dmaHandle = 5;

  buffer.allocatorType = QC::Memory::QC_MEMORY_ALLOCATOR_LAST;
  EXPECT_EQ(InspectHcmBuffer(buffer, fd), HcmBufferAccess::kUnusable);

  EXPECT_EQ(BeginDmaBufCpuRead(-1), Status::kOk);
  EXPECT_EQ(EndDmaBufCpuRead(-1), Status::kOk);
  EXPECT_EQ(BeginDmaBufCpuWrite(-1), Status::kOk);
  EXPECT_EQ(EndDmaBufCpuWrite(-1), Status::kOk);
}

// A DmaBuf Contract may be received into heap storage. Such a target is
// classified kHostMemory with no fd, and the PointCloud converter fills and reads it.
TEST(HcmDmaBuffer, PointCloudDescriptorAcceptsHeapStorage)
{
  PointCloudHcmContract cloud;
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

  const std::vector<std::uint8_t> source(64, 0x5A);
  std::vector<std::uint8_t> storage(256, 0);
  PointCloudHcmDescriptor target;
  target.pBuf = storage.data();
  target.size = storage.size();
  target.offset = 0;
  target.allocatorType = QC::Memory::QC_MEMORY_ALLOCATOR_HEAP;

  ASSERT_TRUE((ConvertContractToHcm<PointCloudHcmContract, PointCloudHcmDescriptor>(
      cloud, DmaBufView{source.data(), source.size()}, target)));
  EXPECT_EQ(target.validSize, 64u);
  EXPECT_EQ(storage[0], 0x5A);

  int fd = 0;
  EXPECT_EQ(InspectHcmBuffer(target, fd), HcmBufferAccess::kHostMemory);
  EXPECT_EQ(fd, -1);

  PointCloudHcmContract back;
  ASSERT_TRUE((ConvertHcmToContract<PointCloudHcmContract, PointCloudHcmDescriptor>(target, back)));
  EXPECT_EQ(back.header().frame_id(), "lidar");
  EXPECT_EQ(back.height(), 1u);
  EXPECT_EQ(back.width(), 4u);
  EXPECT_EQ(back.point_step(), 16u);
  EXPECT_EQ(back.row_step(), 64u);
  EXPECT_TRUE(back.is_dense());
  ASSERT_EQ(back.fields_size(), 3);
  EXPECT_EQ(back.fields(2).name(), "z");
  EXPECT_EQ(back.fields(2).offset(), 8u);
}
