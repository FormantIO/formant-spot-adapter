#pragma once

#include <future>
#include <memory>
#include <vector>

#include <bosdyn/api/graph_nav/map_processing.pb.h>
#include <bosdyn/api/graph_nav/map_processing_service.grpc.pb.h>

#include "bosdyn/client/service_client/service_client.h"

namespace bosdyn {
namespace client {

// The vendored C++ SDK has no map processing client, so this is a minimal one
// for ProcessTopology (a server-streaming RPC). The result holds every streamed
// response; callers read status/new_subgraph from them.
typedef Result<std::vector<::bosdyn::api::graph_nav::ProcessTopologyResponse>>
    ProcessTopologyResultType;

class MapProcessingClient : public ServiceClient {
 public:
  MapProcessingClient() = default;
  ~MapProcessingClient() = default;

  std::shared_future<ProcessTopologyResultType> ProcessTopologyAsync(
      ::bosdyn::api::graph_nav::ProcessTopologyRequest& request,
      const RPCParameters& parameters = RPCParameters());
  ProcessTopologyResultType ProcessTopology(
      ::bosdyn::api::graph_nav::ProcessTopologyRequest& request,
      const RPCParameters& parameters = RPCParameters());

  QualityOfService GetQualityOfService() const override;
  void SetComms(const std::shared_ptr<grpc::ChannelInterface>& channel) override;
  void UpdateServiceFrom(RequestProcessorChain& request_processor_chain,
                         ResponseProcessorChain& response_processor_chain,
                         const std::shared_ptr<LeaseWallet>& lease_wallet) override;

  static std::string GetDefaultServiceName() { return s_default_service_name; }
  static std::string GetServiceType() { return s_service_type; }

 private:
  void OnProcessTopologyComplete(
      MessagePumpCallBase* call,
      const ::bosdyn::api::graph_nav::ProcessTopologyRequest& request,
      std::vector<::bosdyn::api::graph_nav::ProcessTopologyResponse>&& responses,
      const grpc::Status& status,
      std::promise<ProcessTopologyResultType> promise);

  std::unique_ptr<::bosdyn::api::graph_nav::MapProcessingService::StubInterface> m_stub;

  static const char* s_default_service_name;
  static const char* s_service_type;
};

}  // namespace client
}  // namespace bosdyn
