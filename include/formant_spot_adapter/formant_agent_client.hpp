#pragma once

#include <atomic>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

#include <grpcpp/grpcpp.h>

#include "protos/agent/v1/agent.pb.h"
#include "protos/agent/v1/agent.grpc.pb.h"
#include "protos/model/v1/commands.pb.h"
#include "protos/model/v1/datapoint.pb.h"
#include "protos/model/v1/navigation.pb.h"

namespace fsa {

class FormantAgentClient {
 public:
  struct PostResult {
    bool ok{false};
    bool throttled{false};
    bool locally_suppressed{false};
    long long retry_after_ms{0};
    int grpc_code{0};
  };

  explicit FormantAgentClient(const std::string& target);

  PostResult PostImage(const std::string& stream, const std::string& content_type,
                       const std::string& bytes, int timeout_ms = 75);
  PostResult PostNumeric(const std::string& stream, double value);
  PostResult PostText(const std::string& stream, const std::string& value);
  PostResult PostLocalization(const std::string& stream,
                              const v1::model::Localization& localization);
  PostResult PostBitset(const std::string& stream,
                        const std::vector<std::pair<std::string, bool>>& bits);
  bool SendCommandResponse(const std::string& request_id, bool success);

  void StartTeleopLoop(const std::vector<std::string>& stream_filter,
                      std::function<void(const v1::model::ControlDatapoint&)> cb);
  void StartCommandLoop(const std::vector<std::string>& command_filter,
                        std::function<void(const v1::model::CommandRequest&)> cb);
  void StartHeartbeatLoop(std::function<void(const v1::agent::GetTeleopHeartbeatStreamResponse&)> cb);
  void StopLoops();

  std::string GetAppConfig(const std::string& key, const std::string& default_value);

 private:
  struct StreamThrottleState {
    long long next_allowed_ms{0};
    long long last_success_ms{0};
    int learned_interval_ms{0};
    int success_streak{0};
  };

  long long NowMs() const;
  void SleepWithBackoff(int attempt) const;
  PostResult BeginPost(const std::string& stream);
  void RecordPostSuccess(const std::string& stream);
  PostResult RecordPostFailure(const std::string& stream,
                               const grpc::Status& status,
                               const char* method_name);

  std::string target_;
  std::unique_ptr<v1::agent::Agent::Stub> stub_;
  std::atomic<bool> stop_{false};
  std::thread teleop_thread_;
  std::thread command_thread_;
  std::thread heartbeat_thread_;
  std::mutex ctx_mu_;
  std::mutex post_mu_;
  std::mutex throttle_mu_;
  std::unordered_map<std::string, StreamThrottleState> stream_throttle_state_;
  std::shared_ptr<grpc::ClientContext> teleop_ctx_;
  std::shared_ptr<grpc::ClientContext> command_ctx_;
  std::shared_ptr<grpc::ClientContext> heartbeat_ctx_;
};

}  // namespace fsa
