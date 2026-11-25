// Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
// SPDX-License-Identifier: BSD-3-Clause-Clear

#include "qrb_ros_adapter/ipc_runtime/transport_codec.hpp"

#include <gtest/gtest.h>

#include <cstdint>
#include <string>
#include <vector>

using namespace qrb::adapter;

namespace
{

std::uint64_t ReadLe(const std::vector<std::uint8_t> & bytes, std::size_t offset, std::size_t width)
{
  std::uint64_t value = 0;
  for (std::size_t i = 0; i < width; ++i) {
    value |= static_cast<std::uint64_t>(bytes[offset + i]) << (8 * i);
  }
  return value;
}

TransportHeader MakeValidHeader()
{
  TransportHeader header;
  header.operation = static_cast<std::uint16_t>(Operation::kFetchResponse);
  header.requestId = 0x0102030405060708ull;
  header.payloadSize = 0x1234;
  header.flags = transport_flags::kHasData;
  header.fdCount = 1;
  header.payloadOffset = 0x1122334455667788ull;
  return header;
}

}  // namespace

// The header is a fixed 36-byte little-endian record, and every
// malformed header is rejected without touching the output.
TEST(TransportCodec, RoundTripsTheFixedLayoutAndRejectsMalformedHeaders)
{
  const TransportHeader header = MakeValidHeader();
  const std::vector<std::uint8_t> bytes = EncodeTransportHeader(header);

  ASSERT_EQ(bytes.size(), kTransportHeaderEncodedSize);
  EXPECT_EQ(ReadLe(bytes, 0, 4), kTransportMagic);
  EXPECT_EQ(ReadLe(bytes, 4, 2), kTransportVersion);
  EXPECT_EQ(ReadLe(bytes, 6, 2), header.operation);
  EXPECT_EQ(ReadLe(bytes, 8, 8), header.requestId);
  EXPECT_EQ(ReadLe(bytes, 16, 4), header.payloadSize);
  EXPECT_EQ(ReadLe(bytes, 20, 4), header.flags);
  EXPECT_EQ(ReadLe(bytes, 24, 4), header.fdCount);
  EXPECT_EQ(ReadLe(bytes, 28, 8), header.payloadOffset);

  TransportHeader decoded;
  ASSERT_EQ(DecodeTransportHeader(bytes.data(), bytes.size(), decoded), Status::kOk);
  EXPECT_EQ(decoded.operation, header.operation);
  EXPECT_EQ(decoded.requestId, header.requestId);
  EXPECT_EQ(decoded.payloadSize, header.payloadSize);
  EXPECT_EQ(decoded.flags, header.flags);
  EXPECT_EQ(decoded.fdCount, header.fdCount);
  EXPECT_EQ(decoded.payloadOffset, header.payloadOffset);

  const struct
  {
    const char * name;
    void (*mutate)(TransportHeader &);
    Status expected;
  } cases[] = {
      {"BadMagic", [](TransportHeader & h) { h.magic = 0xDEADBEEFu; }, Status::kProtocolError},
      {"FutureVersion", [](TransportHeader & h) { h.protocolVersion = kTransportVersion + 1; },
          Status::kVersionMismatch},
      {"UnknownOperation",
          [](TransportHeader & h) {
            h.operation = static_cast<std::uint16_t>(Operation::kFetchComplete) + 1;
          },
          Status::kProtocolError},
      {"PayloadTooLarge",
          [](TransportHeader & h) {
            h.payloadSize = static_cast<std::uint32_t>(kMaxPayloadSize) + 1;
          },
          Status::kProtocolError},
      {"OffsetWithoutFd",
          [](TransportHeader & h) {
            h.fdCount = 0;
            h.payloadOffset = 8;
          },
          Status::kProtocolError},
  };
  for (const auto & c : cases) {
    SCOPED_TRACE(c.name);
    TransportHeader malformed = MakeValidHeader();
    c.mutate(malformed);
    const std::vector<std::uint8_t> encoded = EncodeTransportHeader(malformed);
    TransportHeader untouched;
    untouched.requestId = 77;
    EXPECT_EQ(DecodeTransportHeader(encoded.data(), encoded.size(), untouched), c.expected);
    EXPECT_EQ(untouched.requestId, 77u);
  }
  EXPECT_EQ(
      DecodeTransportHeader(bytes.data(), bytes.size() - 1, decoded), Status::kInvalidArgument);
}
