// Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
// SPDX-License-Identifier: BSD-3-Clause-Clear

#include "qrb_ros_adapter/ipc_runtime/transport_codec.hpp"

namespace qrb::adapter
{

namespace
{

void PutU16(std::vector<std::uint8_t> & out, std::uint16_t v)
{
  for (int i = 0; i < 2; ++i)
    out.push_back(static_cast<std::uint8_t>((v >> (8 * i)) & 0xFF));
}

void PutU32(std::vector<std::uint8_t> & out, std::uint32_t v)
{
  for (int i = 0; i < 4; ++i)
    out.push_back(static_cast<std::uint8_t>((v >> (8 * i)) & 0xFF));
}

void PutU64(std::vector<std::uint8_t> & out, std::uint64_t v)
{
  for (int i = 0; i < 8; ++i)
    out.push_back(static_cast<std::uint8_t>((v >> (8 * i)) & 0xFF));
}

std::uint16_t GetU16(const std::uint8_t * data, std::size_t & offset)
{
  std::uint16_t v = 0;
  for (int i = 0; i < 2; ++i)
    v |= static_cast<std::uint16_t>(data[offset + i]) << (8 * i);
  offset += 2;
  return v;
}

std::uint32_t GetU32(const std::uint8_t * data, std::size_t & offset)
{
  std::uint32_t v = 0;
  for (int i = 0; i < 4; ++i)
    v |= static_cast<std::uint32_t>(data[offset + i]) << (8 * i);
  offset += 4;
  return v;
}

std::uint64_t GetU64(const std::uint8_t * data, std::size_t & offset)
{
  std::uint64_t v = 0;
  for (int i = 0; i < 8; ++i)
    v |= static_cast<std::uint64_t>(data[offset + i]) << (8 * i);
  offset += 8;
  return v;
}

}  // namespace

std::vector<std::uint8_t> EncodeTransportHeader(const TransportHeader & header)
{
  std::vector<std::uint8_t> out;
  out.reserve(kTransportHeaderEncodedSize);
  PutU32(out, header.magic);
  PutU16(out, header.protocolVersion);
  PutU16(out, header.operation);
  PutU64(out, header.requestId);
  PutU32(out, header.payloadSize);
  PutU32(out, header.flags);
  PutU32(out, header.fdCount);
  PutU64(out, header.payloadOffset);
  return out;
}

Status DecodeTransportHeader(const std::uint8_t * data, std::size_t len, TransportHeader & out)
{
  if (data == nullptr || len != kTransportHeaderEncodedSize)
    return Status::kInvalidArgument;

  std::size_t offset = 0;
  TransportHeader header;
  header.magic = GetU32(data, offset);
  header.protocolVersion = GetU16(data, offset);
  header.operation = GetU16(data, offset);
  header.requestId = GetU64(data, offset);
  header.payloadSize = GetU32(data, offset);
  header.flags = GetU32(data, offset);
  header.fdCount = GetU32(data, offset);
  header.payloadOffset = GetU64(data, offset);

  if (header.magic != kTransportMagic)
    return Status::kProtocolError;
  // Checked before anything else field-specific: a version disagreement is
  // the one failure a peer can act on (HLD §18), so it must not be reported
  // as a generic protocol error.
  if (header.protocolVersion != kTransportVersion)
    return Status::kVersionMismatch;
  if (!IsValidOperation(header.operation))
    return Status::kProtocolError;
  if (header.payloadSize > kMaxPayloadSize)
    return Status::kProtocolError;
  if (header.fdCount > kMaxFdCount)
    return Status::kProtocolError;
  if (header.fdCount == 0 && header.payloadOffset != 0)
    return Status::kProtocolError;

  out = header;
  return Status::kOk;
}

}  // namespace qrb::adapter
