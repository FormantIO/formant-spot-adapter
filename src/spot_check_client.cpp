#include "formant_spot_adapter/spot_check_client.hpp"

#include "bosdyn/client/error_codes/sdk_error_code.h"
#include "bosdyn/client/lease/lease_processors.h"
#include "bosdyn/client/lease/lease_resources.h"

using namespace std::placeholders;

namespace bosdyn {
namespace client {

const char* SpotCheckClient::s_default_service_name = "spot-check";
const char* SpotCheckClient::s_service_type = "bosdyn.api.spot.SpotCheckService";

std::shared_future<CameraCalibrationCommandResultType> SpotCheckClient::CameraCalibrationCommandAsync(
    ::bosdyn::api::spot::CameraCalibrationCommandRequest& request, const RPCParameters& parameters) {
  std::promise<CameraCalibrationCommandResultType> response;
  std::shared_future<CameraCalibrationCommandResultType> future = response.get_future();
  BOSDYN_ASSERT_PRECONDITION(m_stub != nullptr, "Stub for service is unset!");

  auto lease_status = ProcessRequestWithLease(&request, m_lease_wallet.get(), ::bosdyn::client::kBodyResource);
  if (!lease_status) {
    response.set_value({lease_status, {}});
    return future;
  }

  MessagePumpCallBase* one_time = InitiateAsyncCall<
      ::bosdyn::api::spot::CameraCalibrationCommandRequest,
      ::bosdyn::api::spot::CameraCalibrationCommandResponse,
      ::bosdyn::api::spot::CameraCalibrationCommandResponse>(
      request,
      std::bind(&::bosdyn::api::spot::SpotCheckService::StubInterface::AsyncCameraCalibrationCommand,
                m_stub.get(), _1, _2, _3),
      std::bind(&SpotCheckClient::OnCameraCalibrationCommandComplete, this, _1, _2, _3, _4, _5),
      std::move(response), parameters);
  (void)one_time;
  return future;
}

CameraCalibrationCommandResultType SpotCheckClient::CameraCalibrationCommand(
    ::bosdyn::api::spot::CameraCalibrationCommandRequest& request, const RPCParameters& parameters) {
  return CameraCalibrationCommandAsync(request, parameters).get();
}

std::shared_future<CameraCalibrationFeedbackResultType> SpotCheckClient::CameraCalibrationFeedbackAsync(
    const RPCParameters& parameters) {
  std::promise<CameraCalibrationFeedbackResultType> response;
  std::shared_future<CameraCalibrationFeedbackResultType> future = response.get_future();
  BOSDYN_ASSERT_PRECONDITION(m_stub != nullptr, "Stub for service is unset!");

  ::bosdyn::api::spot::CameraCalibrationFeedbackRequest request;
  MessagePumpCallBase* one_time = InitiateAsyncCall<
      ::bosdyn::api::spot::CameraCalibrationFeedbackRequest,
      ::bosdyn::api::spot::CameraCalibrationFeedbackResponse,
      ::bosdyn::api::spot::CameraCalibrationFeedbackResponse>(
      request,
      std::bind(&::bosdyn::api::spot::SpotCheckService::StubInterface::AsyncCameraCalibrationFeedback,
                m_stub.get(), _1, _2, _3),
      std::bind(&SpotCheckClient::OnCameraCalibrationFeedbackComplete, this, _1, _2, _3, _4, _5),
      std::move(response), parameters);
  (void)one_time;
  return future;
}

CameraCalibrationFeedbackResultType SpotCheckClient::CameraCalibrationFeedback(
    const RPCParameters& parameters) {
  return CameraCalibrationFeedbackAsync(parameters).get();
}

void SpotCheckClient::OnCameraCalibrationCommandComplete(
    MessagePumpCallBase* call,
    const ::bosdyn::api::spot::CameraCalibrationCommandRequest& request,
    ::bosdyn::api::spot::CameraCalibrationCommandResponse&& response,
    const grpc::Status& status,
    std::promise<CameraCalibrationCommandResultType> promise) {
  ::bosdyn::common::Status ret_status =
      ProcessResponseWithLeaseAndGetFinalStatus<::bosdyn::api::spot::CameraCalibrationCommandResponse>(
          status, response, response.lease_use_result().status(), m_lease_wallet.get());
  promise.set_value({ret_status, std::move(response)});
}

void SpotCheckClient::OnCameraCalibrationFeedbackComplete(
    MessagePumpCallBase* call,
    const ::bosdyn::api::spot::CameraCalibrationFeedbackRequest& request,
    ::bosdyn::api::spot::CameraCalibrationFeedbackResponse&& response,
    const grpc::Status& status,
    std::promise<CameraCalibrationFeedbackResultType> promise) {
  ::bosdyn::common::Status ret_status =
      ProcessResponseAndGetFinalStatus<::bosdyn::api::spot::CameraCalibrationFeedbackResponse>(
          status, response, SDKErrorCode::Success);
  promise.set_value({ret_status, std::move(response)});
}

ServiceClient::QualityOfService SpotCheckClient::GetQualityOfService() const {
  return QualityOfService::NORMAL;
}

void SpotCheckClient::SetComms(const std::shared_ptr<grpc::ChannelInterface>& channel) {
  m_stub.reset(new ::bosdyn::api::spot::SpotCheckService::Stub(channel));
}

void SpotCheckClient::UpdateServiceFrom(RequestProcessorChain& request_processor_chain,
                                        ResponseProcessorChain& response_processor_chain,
                                        const std::shared_ptr<LeaseWallet>& lease_wallet) {
  m_request_processor_chain = request_processor_chain;
  m_response_processor_chain = response_processor_chain;
  m_lease_wallet = lease_wallet;
}

}  // namespace client
}  // namespace bosdyn
