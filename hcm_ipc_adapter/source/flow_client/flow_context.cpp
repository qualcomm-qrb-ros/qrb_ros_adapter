// Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
// SPDX-License-Identifier: BSD-3-Clause-Clear

#include "qrb_ros_adapter/hcm_ipc_adapter/flow_client/flow_context.hpp"

namespace qrb::adapter
{

Status FlowContext::Configure( const FlowConfig &config, const ContractDescriptor &descriptor )
{
    if ( descriptor_ != nullptr ) return Status::kBadState;
    config_ = config;
    descriptor_ = &descriptor;
    return Status::kOk;
}

Status FlowContext::AssignRegistration( const FlowRegistration &registration )
{
    if ( descriptor_ == nullptr || !registration_.dataSocketPath.empty()
        || !registration_.instanceToken.empty() )
    {
        return Status::kBadState;
    }
    if ( registration.dataSocketPath.empty() || registration.instanceToken.empty() )
    {
        return Status::kInvalidArgument;
    }
    registration_ = registration;
    return Status::kOk;
}

void FlowContext::Reset()
{
    config_ = {};
    registration_ = {};
    descriptor_ = nullptr;
    lastRequestId_ = 0;
    framesIn = 0;
    framesOut = 0;
    noDataResponses = 0;
    conversionErrors = 0;
}

}   // namespace qrb::adapter
