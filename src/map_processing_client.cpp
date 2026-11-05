#include "formant_spot_adapter/map_processing_client.hpp"

#include "bosdyn/client/error_codes/sdk_error_code.h"

using namespace std::placeholders;

namespace bosdyn {
namespace client {

const char* MapProcessingClient::s_default_service_name = "map-processing-service";
const char* MapProcessingClient::s_service_type = "bosdyn.api.graph_nav.MapProcessingService";

std::shared_future<ProcessTopologyResultType> MapProcessingClient::ProcessTopologyAsync(
    ::bosdyn::api::graph_nav::ProcessTopologyRequest& request, const RPCParameters& parameters) {
  std::promise<ProcessTopologyResultType> response;
  std::shared_future<ProcessTopologyResultType> future = response.get_future();
  BOSDYN_ASSERT_PRECONDITION(m_stub != nullptr, "Stub for service is unset!");

  MessagePumpCallBase* one_time = InitiateResponseStreamAsyncCall<
      ::bosdyn::api::graph_nav::ProcessTopologyRequest,
      ::bosdyn::api::graph_nav::ProcessTopologyResponse,
      std::vector<::bosdyn::api::graph_nav::ProcessTopologyResponse>>(
      request,
      std::bind(&::bosdyn::api::graph_nav::MapProcessingService::StubInterface::AsyncProcessTopology,
                m_stub.get(), _1, _2, _3, _4),
      std::bind(&MapProcessingClient::OnProcessTopologyComplete, this, _1, _2, _3, _4, _5),
      std::move(response), parameters);
  (void)one_time;
  return future;
}

ProcessTopologyResultType MapProcessingClient::ProcessTopology(
    ::bosdyn::api::graph_nav::ProcessTopologyRequest& request, const RPCParameters& parameters) {
  return ProcessTopologyAsync(request, parameters).get();
}

void MapProcessingClient::OnProcessTopologyComplete(
    MessagePumpCallBase* call,
    const ::bosdyn::api::graph_nav::ProcessTopologyRequest& request,
    std::vector<::bosdyn::api::graph_nav::ProcessTopologyResponse>&& responses,
    const grpc::Status& status,
    std::promise<ProcessTopologyResultType> promise) {
  (void)call;
  (void)request;
  // Header/gRPC errors only. The per-response Status enum has no SDK error
  // category, so the caller checks it.
  ::bosdyn::common::Status ret_status =
      ProcessResponseVector<::bosdyn::api::graph_nav::ProcessTopologyResponse>(status, responses);
  if (!ret_status) {
    promise.set_value({ret_status, {}});
    return;
  }
  promise.set_value({ret_status, std::move(responses)});
}

ServiceClient::QualityOfService MapProcessingClient::GetQualityOfService() const {
  return QualityOfService::NORMAL;
}

void MapProcessingClient::SetComms(const std::shared_ptr<grpc::ChannelInterface>& channel) {
  m_stub.reset(new ::bosdyn::api::graph_nav::MapProcessingService::Stub(channel));
}

void MapProcessingClient::UpdateServiceFrom(RequestProcessorChain& request_processor_chain,
                                            ResponseProcessorChain& response_processor_chain,
                                            const std::shared_ptr<LeaseWallet>& lease_wallet) {
  (void)lease_wallet;
  m_request_processor_chain = request_processor_chain;
  m_response_processor_chain = response_processor_chain;
}

}  // namespace client
}  // namespace bosdyn
