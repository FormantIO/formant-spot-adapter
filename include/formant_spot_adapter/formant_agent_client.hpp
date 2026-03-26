#pragma once

#include <atomic>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <grpcpp/grpcpp.h>

#include "formant_agent.pb.h"
#include "formant_agent.grpc.pb.h"
#include "formant_model.pb.h"

namespace fsa {

class FormantAgentClient {
 public:
  explicit FormantAgentClient(const std::string& target);

  bool PostImage(const std::string& stream, const std::string& content_type,
                 const std::string& bytes, int timeout_ms = 75);
  bool PostNumeric(const std::string& stream, double value);
  bool PostText(const std::string& stream, const std::string& value);
  bool PostLocalization(const std::string& stream, const v1::model::Localization& localization);
  bool PostBitset(const std::string& stream, const std::vector<std::pair<std::string, bool>>& bits);
  bool SendCommandResponse(const std::string& request_id, bool success);

  void StartTeleopLoop(const std::vector<std::string>& stream_filter,
                      std::function<void(const v1::model::ControlDatapoint&)> cb);
  void StartCommandLoop(const std::vector<std::string>& command_filter,
                        std::function<void(const v1::model::CommandRequest&)> cb);
  void StartHeartbeatLoop(std::function<void(const v1::agent::GetTeleopHeartbeatStreamResponse&)> cb);
  void StopLoops();

  std::string GetAppConfig(const std::string& key, const std::string& default_value);

 private:
  long long NowMs() const;
  void SleepWithBackoff(int attempt) const;

  std::string target_;
  std::unique_ptr<v1::agent::Agent::Stub> stub_;
  std::atomic<bool> stop_{false};
  std::thread teleop_thread_;
  std::thread command_thread_;
  std::thread heartbeat_thread_;
  std::mutex ctx_mu_;
  std::mutex post_mu_;
  std::shared_ptr<grpc::ClientContext> teleop_ctx_;
  std::shared_ptr<grpc::ClientContext> command_ctx_;
  std::shared_ptr<grpc::ClientContext> heartbeat_ctx_;
};

}  // namespace fsa
