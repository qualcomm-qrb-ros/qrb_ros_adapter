// Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
// SPDX-License-Identifier: BSD-3-Clause-Clear

#include "qrb_ros_adapter/hcm_ipc_adapter/flow_client/hcm_flow_client.hpp"

#include <google/protobuf/descriptor.h>
#include <google/protobuf/message.h>
#include <sys/mman.h>
#include <unistd.h>

#include <algorithm>
#include <cstdint>

#include "QC/Infras/Memory/BufferDescriptor.hpp"
#include "qrb_ros_adapter/hcm_ipc_adapter/flow_client/hcm_converters.hpp"
#include "qrb_ros_adapter/hcm_ipc_adapter/flow_client/hcm_dma_buffer.hpp"
#include "qrb_ros_adapter/ipc_runtime/contract_descriptor_registry.hpp"
#include "qrb_ros_adapter/ipc_runtime/control_status.hpp"
#include "qrb_ros_adapter/ipc_runtime/transport_codec.hpp"

namespace qrb::adapter
{

namespace
{

using Clock = std::chrono::steady_clock;

std::chrono::milliseconds Remaining(Clock::time_point deadline)
{
  const auto now = Clock::now();
  if (now >= deadline)
    return std::chrono::milliseconds(0);
  // Round up so a wait never ends before the deadline.
  return std::chrono::ceil<std::chrono::milliseconds>(deadline - now);
}

Status QueryDmaBufSize(int fd, std::uint64_t & outSize)
{
  if (fd < 0)
    return Status::kInvalidArgument;

  off_t end = ::lseek(fd, 0, SEEK_END);
  if (end < 0)
    return Status::kResourceError;
  ::lseek(fd, 0, SEEK_SET);

  outSize = static_cast<std::uint64_t>(end);
  return Status::kOk;
}

}  // namespace

HcmFlowClient::~HcmFlowClient()
{
  Close();
}

Status HcmFlowClient::Initialize(const FlowConfig & config,
    const std::string & bootstrapSocketPath,
    std::string & diagnostic)
{
  if (context_.Descriptor() != nullptr || managerRegistered_ || binding_ != nullptr ||
      contractMessage_ != nullptr || !bootstrapSocketPath_.empty()) {
    diagnostic += "HcmFlowClient is already initialized; ";
    return Status::kBadState;
  }

  const auto deadline = Clock::now() + std::chrono::milliseconds(config.startupTimeoutMs);

  const HcmContractBinding * hcmBinding = FindHcmContractBinding(config.contractTypeName);
  if (hcmBinding == nullptr || hcmBinding->descriptor == nullptr ||
      hcmBinding->acceptsDescriptor == nullptr || hcmBinding->contractToHcm == nullptr ||
      hcmBinding->hcmToContract == nullptr || hcmBinding->createDescriptor == nullptr) {
    diagnostic +=
        "contract_type_name '" + config.contractTypeName + "' has no HCM implementation; ";
    return Status::kUnsupportedContract;
  }
  const ContractDescriptor & descriptor = *hcmBinding->descriptor;

  if (bootstrapSocketPath.empty()) {
    diagnostic += "bootstrap_socket_path must not be empty; ";
    return Status::kInvalidArgument;
  }

  Status validated = ValidateFlowConfig(config, descriptor, diagnostic);
  if (validated != Status::kOk)
    return validated;

  const auto * messageDescriptor =
      google::protobuf::DescriptorPool::generated_pool()->FindMessageTypeByName(
          config.contractTypeName);
  const auto * messagePrototype =
      messageDescriptor == nullptr ?
          nullptr :
          google::protobuf::MessageFactory::generated_factory()->GetPrototype(messageDescriptor);
  if (messagePrototype == nullptr) {
    diagnostic += "contract type is not registered in the Protobuf generated pool; ";
    return Status::kUnsupportedContract;
  }
  std::unique_ptr<google::protobuf::MessageLite> contractMessage(messagePrototype->New());
  if (contractMessage == nullptr) {
    diagnostic += "failed to allocate the Contract message; ";
    return Status::kResourceError;
  }

  Status configured = context_.Configure(config, descriptor);
  if (configured != Status::kOk)
    return configured;

  // Commit all prepared resources only after validation and context setup
  // succeed. A rejected re-entry therefore cannot mutate the active Session.
  binding_ = hcmBinding;
  contractMessage_ = std::move(contractMessage);
  bootstrapSocketPath_ = bootstrapSocketPath;

  Status registered = RegisterManagedFlow(deadline, diagnostic);
  if (registered != Status::kOk)
    return Fail(registered, diagnostic);

  Status connected = session_.Connect(context_.Registration().dataSocketPath, Remaining(deadline));
  if (connected != Status::kOk) {
    return Fail(connected, connected == Status::kTimeout ? "data socket connect timed out" :
                                                           "data socket connect failed");
  }

  const auto handshakeTimeout = Remaining(deadline);
  if (handshakeTimeout.count() == 0) {
    return Fail(Status::kTimeout, "startup timed out before data handshake");
  }
  Status handshake = session_.PerformClientHandshake(
      context_.Config().flowId, context_.Registration(), handshakeTimeout, diagnostic);
  if (handshake != Status::kOk)
    return Fail(handshake, diagnostic);

  return Status::kOk;
}

Status HcmFlowClient::RegisterManagedFlow(Deadline deadline, std::string & diagnostic)
{
  // The ROS Bridge Manager must already run: one attempt, no waiting for it.
  IpcSession bootstrap;
  Status connected = bootstrap.ConnectOnce(bootstrapSocketPath_);
  if (connected == Status::kTimeout) {
    diagnostic +=
        "ROS Bridge Manager is not running: nothing listens on " + bootstrapSocketPath_ + "; ";
    return Status::kResourceError;
  }
  if (connected != Status::kOk) {
    diagnostic += "ROS Bridge Manager bootstrap connection failed; ";
    return connected;
  }

  ipc_proto::RegisterFlowRequest request;
  const FlowConfig & config = context_.Config();
  request.set_flow_id(config.flowId);
  request.set_contract_type_name(config.contractTypeName);
  request.set_direction(ToProtoDirection(config.direction));
  request.set_ros_topic(config.rosTopic);
  request.set_timeout_ms(config.timeoutMs);
  request.set_startup_timeout_ms(config.startupTimeoutMs);
  ToProtoQosProfile(config.qosProfile, *request.mutable_qos_profile());

  Status sent = bootstrap.SendProtoFrame(Operation::kRegisterFlowRequest, 1, request);
  if (sent != Status::kOk)
    return sent;

  TransportHeader header;
  std::vector<std::uint8_t> payload;
  std::vector<OwnedFd> fds;
  const auto responseTimeout = Remaining(deadline);
  if (responseTimeout.count() == 0) {
    diagnostic += "startup timed out waiting for RegisterFlowResponse; ";
    return Status::kTimeout;
  }
  Status received = bootstrap.ReceiveFrame(header, payload, fds, responseTimeout);
  if (received != Status::kOk)
    return received;
  if (header.operation != static_cast<std::uint16_t>(Operation::kRegisterFlowResponse) ||
      header.requestId != 1 || header.fdCount != 0) {
    return Status::kProtocolError;
  }

  ipc_proto::RegisterFlowResponse response;
  if (IpcSession::ParseProtoPayload(payload, response) != Status::kOk) {
    return Status::kProtocolError;
  }
  Status result = FromProtoStatusCode(response.status());
  if (result != Status::kOk) {
    diagnostic += response.diagnostic_message();
    return result;
  }
  managerRegistered_ = true;
  Status assigned = context_.AssignRegistration(
      FlowRegistration{response.data_socket_path(), response.instance_token()});
  if (assigned != Status::kOk)
    return assigned;
  return Status::kOk;
}

void HcmFlowClient::UnregisterManagedFlow()
{
  if (!managerRegistered_)
    return;
  managerRegistered_ = false;

  // Without a running Manager there is no registration left to remove.
  IpcSession bootstrap;
  if (bootstrap.ConnectOnce(bootstrapSocketPath_) != Status::kOk)
    return;

  ipc_proto::UnregisterFlowRequest request;
  request.set_flow_id(context_.Config().flowId);
  request.set_instance_token(context_.Registration().instanceToken);
  if (bootstrap.SendProtoFrame(Operation::kUnregisterFlowRequest, 1, request) != Status::kOk) {
    return;
  }

  TransportHeader header;
  std::vector<std::uint8_t> payload;
  std::vector<OwnedFd> fds;
  if (bootstrap.ReceiveFrame(header, payload, fds, std::chrono::milliseconds(200)) != Status::kOk ||
      header.operation != static_cast<std::uint16_t>(Operation::kUnregisterFlowResponse) ||
      header.requestId != 1 || header.fdCount != 0) {
    return;
  }

  ipc_proto::UnregisterFlowResponse response;
  if (IpcSession::ParseProtoPayload(payload, response) != Status::kOk)
    return;
  (void)FromProtoStatusCode(response.status());
}

Status HcmFlowClient::Fail(Status reason, const std::string & diagnostic)
{
  (void)diagnostic;
  Close();
  return reason;
}

Status HcmFlowClient::RejectFetched(std::uint64_t requestId,
    Status reason,
    const std::string & diagnostic)
{
  if (context_.Descriptor()->payloadClass == PayloadClass::kDmaBuf) {
    Status completed =
        session_.SendStatusResponse(Operation::kFetchComplete, requestId, reason, diagnostic);
    if (completed != Status::kOk)
      return Fail(completed, "FETCH_COMPLETE send failed");
    ++context_.framesOut;
  }
  return reason;
}

Status HcmFlowClient::Fetch(QC::Memory::QCBufferDescriptorBase_t & target)
{
  if (context_.Descriptor() == nullptr || context_.Registration().dataSocketPath.empty() ||
      context_.Registration().instanceToken.empty() || binding_ == nullptr ||
      contractMessage_ == nullptr) {
    return Status::kBadState;
  }
  if (context_.Config().direction != Direction::kRosToHcm)
    return Status::kBadState;
  if (!binding_->acceptsDescriptor(target)) {
    return Status::kInvalidArgument;
  }

  const std::uint64_t requestId = context_.NextRequestId();
  Status sent = session_.SendEmptyFrame(Operation::kFetchRequest, requestId);
  if (sent != Status::kOk)
    return Fail(sent, "FETCH_REQUEST send failed");
  ++context_.framesOut;

  TransportHeader header;
  std::vector<std::uint8_t> payload;
  std::vector<OwnedFd> fds;
  for (;;) {
    Status received = session_.ReceiveFrame(header, payload, fds, Timeout());
    if (received != Status::kOk) {
      return Fail(received, "no FETCH_RESPONSE within timeout_ms");
    }
    ++context_.framesIn;

    if (header.operation == static_cast<std::uint16_t>(Operation::kFetchResponse)) {
      if (header.requestId != requestId) {
        return Fail(Status::kProtocolError, "FETCH_RESPONSE request id mismatch");
      }
      break;
    }

    return Fail(Status::kProtocolError,
        "unexpected operation while waiting for FETCH_RESPONSE: " +
            std::string(ToString(static_cast<Operation>(header.operation))));
  }

  if ((header.flags & transport_flags::kHasData) == 0) {
    // NO_DATA is a completed request, not a transport failure. Keep the
    // Session alive so a later host invocation can fetch the next sample.
    ipc_proto::OperationStatus status;
    if (IpcSession::ParseProtoPayload(payload, status) != Status::kOk) {
      return Fail(Status::kProtocolError, "malformed no-data FETCH_RESPONSE");
    }
    if (header.fdCount != 0) {
      return Fail(Status::kProtocolError, "no-data FETCH_RESPONSE carried an fd");
    }
    Status peerStatus = FromProtoStatusCode(status.status());
    if (peerStatus == Status::kOk) {
      return Fail(Status::kProtocolError, "FETCH_RESPONSE without HAS_DATA reported success");
    }
    if (peerStatus == Status::kNoData) {
      ++context_.noDataResponses;
      return Status::kNoData;
    }
    if (IsFrameError(peerStatus)) {
      // The bridge could not convert its sample: it dropped that sample
      // and keeps serving this Session.
      ++context_.conversionErrors;
      return peerStatus;
    }
    return Fail(peerStatus, "ROS IPC Bridge rejected fetch: " + status.diagnostic_message());
  }

  Status parsed = IpcSession::ParseProtoPayload(payload, *contractMessage_);
  if (parsed != Status::kOk)
    return Fail(parsed, "malformed contract message");

  const bool isDmaBuf = context_.Descriptor()->payloadClass == PayloadClass::kDmaBuf;
  DmaBufView view;
  void * mappedAddress = MAP_FAILED;
  std::size_t mappedSize = 0;
  bool cpuReadStarted = false;

  if (isDmaBuf) {
    // A DmaBuf Contract carries exactly one payload fd; any other count is a protocol error.
    if (header.fdCount != 1 || fds.size() != 1) {
      return Fail(Status::kProtocolError, "DmaBuf Contract frame requires exactly one fd");
    }

    std::uint64_t bufferSize = 0;
    Status queried = QueryDmaBufSize(fds[0].Get(), bufferSize);
    if (queried != Status::kOk)
      return Fail(queried, "received fd is not a DMA-BUF");
    if (bufferSize == 0 || bufferSize > static_cast<std::uint64_t>(SIZE_MAX)) {
      return Fail(Status::kProtocolError, "invalid received DMA-BUF capacity");
    }

    if (header.payloadOffset > bufferSize ||
        header.payloadOffset > static_cast<std::uint64_t>(SIZE_MAX)) {
      return Fail(Status::kProtocolError, "received DMA-BUF payload offset exceeds capacity");
    }

    mappedAddress = ::mmap(
        nullptr, static_cast<std::size_t>(bufferSize), PROT_READ, MAP_SHARED, fds[0].Get(), 0);
    if (mappedAddress == MAP_FAILED) {
      return Fail(Status::kResourceError, "failed to map received DMA-BUF");
    }
    mappedSize = static_cast<std::size_t>(bufferSize);
    Status syncStarted = BeginDmaBufCpuRead(fds[0].Get());
    if (syncStarted != Status::kOk) {
      ::munmap(mappedAddress, mappedSize);
      return Fail(syncStarted, "failed to begin DMA-BUF CPU read");
    }
    cpuReadStarted = true;
    view.data =
        static_cast<std::uint8_t *>(mappedAddress) + static_cast<std::size_t>(header.payloadOffset);
    view.size = static_cast<std::size_t>(bufferSize - header.payloadOffset);
  } else if (header.fdCount != 0) {
    return Fail(Status::kProtocolError, "Inline Contract frame must not carry an fd");
  } else if (header.payloadOffset != 0) {
    return Fail(Status::kProtocolError, "Inline Contract frame must not carry a payload offset");
  }

  // targetFd is -1 for heap storage, which makes the target sync below a no-op.
  int targetFd = -1;
  const HcmBufferAccess targetAccess = InspectHcmBuffer(target, targetFd);
  if (isDmaBuf && targetAccess != HcmBufferAccess::kLocalDmaBuf &&
      targetAccess != HcmBufferAccess::kHostMemory) {
    if (cpuReadStarted)
      EndDmaBufCpuRead(fds[0].Get());
    ::munmap(mappedAddress, mappedSize);
    fds[0].Reset();
    return RejectFetched(requestId, Status::kInvalidArgument,
        "HCM descriptor provides neither heap memory nor a local DMA-BUF");
  }
  Status targetSync = BeginDmaBufCpuWrite(targetFd);
  if (targetSync != Status::kOk) {
    if (cpuReadStarted)
      EndDmaBufCpuRead(fds[0].Get());
    if (mappedAddress != MAP_FAILED)
      ::munmap(mappedAddress, mappedSize);
    return Fail(targetSync, "failed to begin HCM descriptor DMA-BUF CPU write");
  }

  ConversionResult converted = binding_->contractToHcm(*contractMessage_, view, target);
  Status targetCleanup = EndDmaBufCpuWrite(targetFd);
  Status copyCleanup = Status::kOk;
  if (isDmaBuf) {
    if (cpuReadStarted)
      copyCleanup = EndDmaBufCpuRead(fds[0].Get());
    if (::munmap(mappedAddress, mappedSize) != 0)
      copyCleanup = Status::kResourceError;
    fds[0].Reset();
    if (copyCleanup != Status::kOk) {
      return Fail(copyCleanup, "failed to finish DMA-BUF CPU read");
    }
  }

  if (targetCleanup != Status::kOk) {
    return Fail(targetCleanup, "failed to finish HCM descriptor DMA-BUF CPU write");
  }

  if (!converted) {
    // For example a sample larger than the host's buffer: drop this one.
    ++context_.conversionErrors;
    return RejectFetched(requestId, Status::kConversionError,
        "Contract-to-HCM conversion failed: " + converted.Diagnostic());
  }

  if (isDmaBuf) {
    // The ROS peer retains the producer's shared Buffer until this
    // completion arrives, preventing its pool from reusing the storage
    // while the copy into the host-owned destination is in progress.
    Status completed =
        session_.SendStatusResponse(Operation::kFetchComplete, requestId, Status::kOk, "");
    if (completed != Status::kOk) {
      return Fail(completed, "FETCH_COMPLETE send failed");
    }
    ++context_.framesOut;
  }

  return Status::kOk;
}

Status HcmFlowClient::Publish(const QC::Memory::QCBufferDescriptorBase_t & source)
{
  if (context_.Descriptor() == nullptr || context_.Registration().dataSocketPath.empty() ||
      context_.Registration().instanceToken.empty() || binding_ == nullptr ||
      contractMessage_ == nullptr) {
    return Status::kBadState;
  }
  if (context_.Config().direction != Direction::kHcmToRos)
    return Status::kBadState;
  if (!binding_->acceptsDescriptor(source)) {
    return Status::kInvalidArgument;
  }

  const bool isDmaBuf = context_.Descriptor()->payloadClass == PayloadClass::kDmaBuf;
  // sourceFd is -1 unless the source is a local DMA-BUF, which makes the sync below a no-op.
  int sourceFd = -1;
  const HcmBufferAccess sourceAccess = InspectHcmBuffer(source, sourceFd);
  // Frame errors before PUBLISH_REQUEST is sent drop only this frame.
  if (isDmaBuf && sourceAccess != HcmBufferAccess::kLocalDmaBuf) {
    return Status::kInvalidArgument;  // no readable local DMA-BUF
  }
  // A local DMA-BUF is a BufferDescriptor_t with offset <= size.
  const auto * sourceBuffer = dynamic_cast<const QC::Memory::BufferDescriptor_t *>(&source);
  if (isDmaBuf && (sourceBuffer == nullptr || sourceBuffer->validSize == 0 ||
                      sourceBuffer->validSize > sourceBuffer->size - sourceBuffer->offset)) {
    return Status::kInvalidArgument;  // invalid payload size
  }

  Status sourceSync = BeginDmaBufCpuRead(sourceFd);
  if (sourceSync != Status::kOk) {
    return Fail(sourceSync, "failed to begin HCM descriptor DMA-BUF CPU read");
  }

  ConversionResult converted = binding_->hcmToContract(source, *contractMessage_);
  Status sourceCleanup = EndDmaBufCpuRead(sourceFd);
  if (sourceCleanup != Status::kOk) {
    return Fail(sourceCleanup, "failed to finish HCM descriptor DMA-BUF CPU read");
  }
  if (!converted) {
    ++context_.conversionErrors;
    return Status::kConversionError;
  }
  if (contractMessage_->ByteSizeLong() > kMaxPayloadSize) {
    // Too large for one IPC frame: drop this frame like a failed conversion.
    ++context_.conversionErrors;
    return Status::kConversionError;
  }

  const std::uint64_t requestId = context_.NextRequestId();
  Status sent = session_.SendProtoFrame(Operation::kPublishRequest, requestId, *contractMessage_,
      isDmaBuf ? &sourceFd : nullptr, isDmaBuf ? 1 : 0, 0,
      isDmaBuf ? static_cast<std::uint64_t>(sourceBuffer->offset) : 0);
  if (sent != Status::kOk) {
    return Fail(sent, "PUBLISH_REQUEST send failed");
  }
  ++context_.framesOut;

  // PUBLISH_RESPONSE confirms that the ROS IPC Bridge copied any DMA-BUF payload out of
  // host storage and accepted the publish.
  TransportHeader header;
  std::vector<std::uint8_t> payload;
  std::vector<OwnedFd> fds;
  for (;;) {
    Status received = session_.ReceiveFrame(header, payload, fds, Timeout());
    if (received != Status::kOk) {
      return Fail(received, "no PUBLISH_RESPONSE within timeout_ms");
    }
    ++context_.framesIn;

    if (header.operation == static_cast<std::uint16_t>(Operation::kPublishResponse)) {
      if (header.requestId != requestId) {
        return Fail(Status::kProtocolError, "PUBLISH_RESPONSE request id mismatch");
      }
      break;
    }

    return Fail(Status::kProtocolError,
        "unexpected operation while waiting for PUBLISH_RESPONSE: " +
            std::string(ToString(static_cast<Operation>(header.operation))));
  }

  ipc_proto::OperationStatus status;
  if (IpcSession::ParseProtoPayload(payload, status) != Status::kOk) {
    return Fail(Status::kProtocolError, "malformed PUBLISH_RESPONSE");
  }

  Status peerStatus = FromProtoStatusCode(status.status());
  if (peerStatus != Status::kOk) {
    ++context_.conversionErrors;
    // The bridge dropped this frame and keeps serving the Session.
    if (IsFrameError(peerStatus))
      return peerStatus;
    return Fail(peerStatus, "ROS IPC Bridge rejected publish: " + status.diagnostic_message());
  }

  return Status::kOk;
}

void HcmFlowClient::Close()
{
  session_.Close();
  UnregisterManagedFlow();
  contractMessage_.reset();
  binding_ = nullptr;
  context_.Reset();
  bootstrapSocketPath_.clear();
}

}  // namespace qrb::adapter
