#pragma once

#include <future>
#include <memory>

#include <bosdyn/api/spot/spot_check_service.grpc.pb.h>
#include <bosdyn/api/spot/spot_check.pb.h>

#include "bosdyn/client/service_client/service_client.h"

namespace bosdyn {
namespace client {

typedef Result<::bosdyn::api::spot::CameraCalibrationCommandResponse>
    CameraCalibrationCommandResultType;
typedef Result<::bosdyn::api::spot::CameraCalibrationFeedbackResponse>
    CameraCalibrationFeedbackResultType;

class SpotCheckClient : public ServiceClient {
 public:
  SpotCheckClient() = default;
  ~SpotCheckClient() = default;

  std::shared_future<CameraCalibrationCommandResultType> CameraCalibrationCommandAsync(
      ::bosdyn::api::spot::CameraCalibrationCommandRequest& request,
      const RPCParameters& parameters = RPCParameters());
  CameraCalibrationCommandResultType CameraCalibrationCommand(
      ::bosdyn::api::spot::CameraCalibrationCommandRequest& request,
      const RPCParameters& parameters = RPCParameters());

  std::shared_future<CameraCalibrationFeedbackResultType> CameraCalibrationFeedbackAsync(
      const RPCParameters& parameters = RPCParameters());
  CameraCalibrationFeedbackResultType CameraCalibrationFeedback(
      const RPCParameters& parameters = RPCParameters());

  QualityOfService GetQualityOfService() const override;
  void SetComms(const std::shared_ptr<grpc::ChannelInterface>& channel) override;
  void UpdateServiceFrom(RequestProcessorChain& request_processor_chain,
                         ResponseProcessorChain& response_processor_chain,
                         const std::shared_ptr<LeaseWallet>& lease_wallet) override;

  static std::string GetDefaultServiceName() { return s_default_service_name; }
  static std::string GetServiceType() { return s_service_type; }

 private:
  void OnCameraCalibrationCommandComplete(
      MessagePumpCallBase* call,
      const ::bosdyn::api::spot::CameraCalibrationCommandRequest& request,
      ::bosdyn::api::spot::CameraCalibrationCommandResponse&& response,
      const grpc::Status& status,
      std::promise<CameraCalibrationCommandResultType> promise);
  void OnCameraCalibrationFeedbackComplete(
      MessagePumpCallBase* call,
      const ::bosdyn::api::spot::CameraCalibrationFeedbackRequest& request,
      ::bosdyn::api::spot::CameraCalibrationFeedbackResponse&& response,
      const grpc::Status& status,
      std::promise<CameraCalibrationFeedbackResultType> promise);

  std::unique_ptr<::bosdyn::api::spot::SpotCheckService::StubInterface> m_stub;
  std::shared_ptr<LeaseWallet> m_lease_wallet;

  static const char* s_default_service_name;
  static const char* s_service_type;
};

}  // namespace client
}  // namespace bosdyn
