#include "formant_spot_adapter/formant_agent_client.hpp"

#include <algorithm>
#include <chrono>
#include <iostream>

namespace fsa {

namespace {

constexpr int kSuccessesBeforeSpeedup = 8;
constexpr int kSpeedupMinStepMs = 50;

// gRPC reuses one TCP connection for channels that have the same target and the
// same arguments. A distinct argument gives each role its own connection, so a
// flood of image posts cannot queue in front of commands and teleop.
std::shared_ptr<grpc::Channel> make_channel(const std::string& target, const char* role) {
  grpc::ChannelArguments args;
  args.SetString("fsa.channel_role", role);
  return grpc::CreateCustomChannel(target, grpc::InsecureChannelCredentials(), args);
}

}  // namespace

PostPressure ClassifyPostFailure(grpc::StatusCode code, const std::string& message) {
  if (code == grpc::StatusCode::RESOURCE_EXHAUSTED) return PostPressure::kThrottled;
  if (message.find("throttl") != std::string::npos ||
      message.find("rate limit") != std::string::npos) {
    return PostPressure::kThrottled;
  }
  if (code == grpc::StatusCode::DEADLINE_EXCEEDED) return PostPressure::kDeadlineExceeded;
  if (code == grpc::StatusCode::UNAVAILABLE) return PostPressure::kUnavailable;
  return PostPressure::kNone;
}

const char* PostPressureName(PostPressure pressure) {
  switch (pressure) {
    case PostPressure::kThrottled:
      return "throttled";
    case PostPressure::kDeadlineExceeded:
      return "deadline_exceeded";
    case PostPressure::kUnavailable:
      return "unavailable";
    case PostPressure::kNone:
      break;
  }
  return "none";
}

int NextBackoffIntervalMs(PostPressure pressure, int current_interval_ms,
                          long long ms_since_last_success) {
  const int current_ms = std::max(0, current_interval_ms);
  if (pressure == PostPressure::kNone) return current_ms;

  int candidate_ms =
      current_ms > 0
          ? current_ms + std::max(kThrottlePaddingMinMs, current_ms / 2)
          : kThrottleInitialBackoffMs;
  if (pressure == PostPressure::kThrottled && ms_since_last_success > 0) {
    // The gap since the last accepted post approximates the rate the agent is
    // enforcing. Clamp before narrowing: the gap is unbounded.
    const long long observed_ms =
        std::min<long long>(ms_since_last_success, kThrottleMaxBackoffMs);
    const int padded_ms = static_cast<int>(observed_ms) +
                          std::max(kThrottlePaddingMinMs, static_cast<int>(observed_ms) / 4);
    candidate_ms = std::max(candidate_ms, padded_ms);
  }

  // An inferred overload must not undo a longer interval already learned from an
  // explicit throttle on the same stream.
  const int cap_ms = pressure == PostPressure::kThrottled
                         ? kThrottleMaxBackoffMs
                         : std::max(kOverloadMaxBackoffMs, current_ms);
  return std::min(cap_ms, candidate_ms);
}

int OverloadRecoveryIntervalMs(int current_interval_ms) {
  const int halved_ms = std::max(0, current_interval_ms) / 2;
  return halved_ms < kThrottlePaddingMinMs ? 0 : halved_ms;
}

FormantAgentClient::FormantAgentClient(const std::string& target)
    : target_(target),
      data_stub_(v1::agent::Agent::NewStub(make_channel(target, "data"))),
      control_stub_(v1::agent::Agent::NewStub(make_channel(target, "control"))) {
  std::cerr << "[formant] target=" << target_ << std::endl;
}

long long FormantAgentClient::NowMs() const {
  using namespace std::chrono;
  return duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count();
}

void FormantAgentClient::SleepWithBackoff(int attempt) const {
  const int capped_attempt = std::max(0, std::min(6, attempt));
  const int base_ms = std::min(5000, 250 * (1 << capped_attempt));
  const int jitter_ms = static_cast<int>(NowMs() % 250);
  const int delay_ms = base_ms + jitter_ms;
  const int step_ms = 100;
  int slept_ms = 0;
  while (!stop_ && slept_ms < delay_ms) {
    const int remaining_ms = delay_ms - slept_ms;
    std::this_thread::sleep_for(std::chrono::milliseconds(std::min(step_ms, remaining_ms)));
    slept_ms += step_ms;
  }
}

FormantAgentClient::PostResult FormantAgentClient::BeginPost(const std::string& stream) {
  PostResult result;
  const long long now = NowMs();
  std::lock_guard<std::mutex> lk(throttle_mu_);
  auto it = stream_throttle_state_.find(stream);
  if (it == stream_throttle_state_.end()) return result;
  if (now < it->second.next_allowed_ms) {
    result.throttled = true;
    result.locally_suppressed = true;
    result.retry_after_ms = it->second.next_allowed_ms;
  }
  return result;
}

void FormantAgentClient::RecordPostSuccess(const std::string& stream) {
  const long long now = NowMs();
  std::lock_guard<std::mutex> lk(throttle_mu_);
  auto& state = stream_throttle_state_[stream];
  state.last_success_ms = now;
  state.consecutive_unavailable = 0;
  if (state.learned_interval_ms > 0 && state.inferred_overload) {
    state.learned_interval_ms = OverloadRecoveryIntervalMs(state.learned_interval_ms);
    state.inferred_overload = state.learned_interval_ms > 0;
    state.success_streak = 0;
    state.next_allowed_ms =
        state.learned_interval_ms > 0 ? now + state.learned_interval_ms : 0;
  } else if (state.learned_interval_ms > 0) {
    state.success_streak += 1;
    if (state.success_streak >= kSuccessesBeforeSpeedup) {
      const int speedup_ms =
          std::max(kSpeedupMinStepMs, state.learned_interval_ms / 10);
      state.learned_interval_ms =
          std::max(0, state.learned_interval_ms - speedup_ms);
      state.success_streak = 0;
    }
    state.next_allowed_ms = now + state.learned_interval_ms;
  } else {
    state.success_streak = 0;
    state.next_allowed_ms = 0;
  }
}

FormantAgentClient::PostResult FormantAgentClient::RecordPostFailure(
    const std::string& stream, const grpc::Status& status, const char* method_name) {
  PostResult result;
  result.grpc_code = static_cast<int>(status.error_code());
  PostPressure pressure = ClassifyPostFailure(status.error_code(), status.error_message());
  int learned_interval_ms = 0;
  {
    const long long now = NowMs();
    std::lock_guard<std::mutex> lk(throttle_mu_);
    auto& state = stream_throttle_state_[stream];
    if (pressure == PostPressure::kUnavailable) {
      state.consecutive_unavailable += 1;
      if (state.consecutive_unavailable < kUnavailableFailuresBeforeBackoff) {
        pressure = PostPressure::kNone;
      }
    } else {
      state.consecutive_unavailable = 0;
    }
    if (pressure != PostPressure::kNone) {
      const long long ms_since_last_success =
          state.last_success_ms > 0 && now > state.last_success_ms
              ? now - state.last_success_ms
              : 0;
      // An interval stays "inferred" only while no explicit throttle has
      // contributed to it; an explicit throttle keeps its slower recovery.
      state.inferred_overload =
          pressure != PostPressure::kThrottled &&
          (state.learned_interval_ms <= 0 || state.inferred_overload);
      learned_interval_ms = NextBackoffIntervalMs(
          pressure, state.learned_interval_ms, ms_since_last_success);
      state.learned_interval_ms = learned_interval_ms;
      state.success_streak = 0;
      state.next_allowed_ms = now + learned_interval_ms;
      result.retry_after_ms = state.next_allowed_ms;
      result.throttled = true;
    }
  }

  if (result.throttled) {
    std::cerr << "[formant] " << method_name
              << " throttled stream=" << stream
              << " target=" << target_
              << " grpc_code=" << result.grpc_code
              << " reason=" << PostPressureName(pressure)
              << " learned_interval_ms=" << learned_interval_ms
              << " msg=" << status.error_message() << std::endl;
    return result;
  }

  std::cerr << "[formant] " << method_name
            << " failed stream=" << stream
            << " target=" << target_
            << " grpc_code=" << result.grpc_code
            << " msg=" << status.error_message() << std::endl;
  return result;
}

FormantAgentClient::PostResult FormantAgentClient::PostImage(
    const std::string& stream, const std::string& content_type,
    const std::string& bytes, int timeout_ms) {
  PostResult result = BeginPost(stream);
  if (result.locally_suppressed) return result;

  v1::model::Datapoint dp;
  dp.set_stream(stream);
  dp.set_timestamp(NowMs());
  dp.mutable_image()->set_content_type(content_type);
  dp.mutable_image()->set_raw(bytes);

  grpc::ClientContext ctx;
  const int clamped_timeout_ms = std::max(25, timeout_ms);
  ctx.set_deadline(std::chrono::system_clock::now() +
                   std::chrono::milliseconds(clamped_timeout_ms));
  v1::agent::PostDataResponse resp;
  auto s = data_stub_->PostData(&ctx, dp, &resp);
  if (s.ok()) {
    RecordPostSuccess(stream);
    result.ok = true;
    return result;
  }
  return RecordPostFailure(stream, s, "PostImage");
}

FormantAgentClient::PostResult FormantAgentClient::PostNumeric(
    const std::string& stream, double value) {
  PostResult result = BeginPost(stream);
  if (result.locally_suppressed) return result;

  std::lock_guard<std::mutex> lk(post_mu_);
  v1::model::Datapoint dp;
  dp.set_stream(stream);
  dp.set_timestamp(NowMs());
  dp.mutable_numeric()->set_value(value);

  grpc::ClientContext ctx;
  ctx.set_deadline(std::chrono::system_clock::now() + std::chrono::milliseconds(500));
  v1::agent::PostDataResponse resp;
  auto s = data_stub_->PostData(&ctx, dp, &resp);
  if (s.ok()) {
    RecordPostSuccess(stream);
    result.ok = true;
    return result;
  }
  return RecordPostFailure(stream, s, "PostNumeric");
}

FormantAgentClient::PostResult FormantAgentClient::PostText(
    const std::string& stream, const std::string& value) {
  PostResult result = BeginPost(stream);
  if (result.locally_suppressed) return result;

  std::lock_guard<std::mutex> lk(post_mu_);
  v1::model::Datapoint dp;
  dp.set_stream(stream);
  dp.set_timestamp(NowMs());
  dp.mutable_text()->set_value(value);

  grpc::ClientContext ctx;
  ctx.set_deadline(std::chrono::system_clock::now() + std::chrono::milliseconds(500));
  v1::agent::PostDataResponse resp;
  auto s = data_stub_->PostData(&ctx, dp, &resp);
  if (s.ok()) {
    RecordPostSuccess(stream);
    result.ok = true;
    return result;
  }
  return RecordPostFailure(stream, s, "PostText");
}

FormantAgentClient::PostResult FormantAgentClient::PostLocalization(
    const std::string& stream, const v1::model::Localization& localization) {
  PostResult result = BeginPost(stream);
  if (result.locally_suppressed) return result;

  std::lock_guard<std::mutex> lk(post_mu_);
  v1::model::Datapoint dp;
  dp.set_stream(stream);
  dp.set_timestamp(NowMs());
  *dp.mutable_localization() = localization;

  grpc::ClientContext ctx;
  ctx.set_deadline(std::chrono::system_clock::now() + std::chrono::milliseconds(1000));
  v1::agent::PostDataResponse resp;
  auto s = data_stub_->PostData(&ctx, dp, &resp);
  if (s.ok()) {
    RecordPostSuccess(stream);
    result.ok = true;
    return result;
  }
  return RecordPostFailure(stream, s, "PostLocalization");
}

FormantAgentClient::PostResult FormantAgentClient::PostBitset(
    const std::string& stream, const std::vector<std::pair<std::string, bool>>& bits) {
  PostResult result = BeginPost(stream);
  if (result.locally_suppressed) return result;

  std::lock_guard<std::mutex> lk(post_mu_);
  v1::model::Datapoint dp;
  dp.set_stream(stream);
  dp.set_timestamp(NowMs());
  for (const auto& b : bits) {
    auto* bit = dp.mutable_bitset()->add_bits();
    bit->set_key(b.first);
    bit->set_value(b.second);
  }

  grpc::ClientContext ctx;
  ctx.set_deadline(std::chrono::system_clock::now() + std::chrono::milliseconds(500));
  v1::agent::PostDataResponse resp;
  auto s = data_stub_->PostData(&ctx, dp, &resp);
  if (s.ok()) {
    RecordPostSuccess(stream);
    result.ok = true;
    return result;
  }
  return RecordPostFailure(stream, s, "PostBitset");
}

bool FormantAgentClient::SendCommandResponse(const std::string& request_id, bool success) {
  // Deliberately not under post_mu_: a command response must not wait behind
  // queued telemetry posts, each of which can hold that lock for its full deadline.
  v1::agent::SendCommandResponseRequest req;
  req.mutable_response()->set_request_id(request_id);
  req.mutable_response()->set_success(success);

  grpc::ClientContext ctx;
  ctx.set_deadline(std::chrono::system_clock::now() + std::chrono::milliseconds(500));
  v1::agent::SendCommandResponseResponse resp;
  auto s = control_stub_->SendCommandResponse(&ctx, req, &resp);
  if (!s.ok()) {
    std::cerr << "[formant] SendCommandResponse failed target=" << target_
              << " grpc_code=" << static_cast<int>(s.error_code())
              << " msg=" << s.error_message() << std::endl;
  }
  return s.ok();
}

void FormantAgentClient::StartTeleopLoop(const std::vector<std::string>& stream_filter,
                                         std::function<void(const v1::model::ControlDatapoint&)> cb,
                                         std::function<void()> on_stream_connected) {
  stop_ = false;
  teleop_thread_ = std::thread([this, stream_filter, cb, on_stream_connected]() {
    int reconnect_attempt = 0;
    while (!stop_) {
      std::cerr << "[formant] opening teleop stream target=" << target_;
      if (reconnect_attempt > 0) {
        std::cerr << " reconnect_attempt=" << reconnect_attempt;
      }
      std::cerr << std::endl;
      auto ctx = std::make_shared<grpc::ClientContext>();
      {
        std::lock_guard<std::mutex> lk(ctx_mu_);
        teleop_ctx_ = ctx;
      }
      v1::agent::GetTeleopControlDataStreamRequest req;
      for (const auto& f : stream_filter) req.add_stream_filter(f);
      auto stream = control_stub_->GetTeleopControlDataStream(ctx.get(), req);
      if (on_stream_connected) on_stream_connected();

      v1::agent::GetTeleopControlDataStreamResponse msg;
      while (!stop_ && stream->Read(&msg)) {
        cb(msg.control_datapoint());
      }
      const auto status = stream->Finish();
      if (!stop_) {
        std::cerr << "[formant] teleop stream closed target=" << target_
                  << " grpc_code=" << static_cast<int>(status.error_code())
                  << " msg=" << status.error_message() << std::endl;
      }

      {
        std::lock_guard<std::mutex> lk(ctx_mu_);
        if (teleop_ctx_ == ctx) teleop_ctx_.reset();
      }
      if (stop_) break;
      ++reconnect_attempt;
      SleepWithBackoff(reconnect_attempt);
    }
  });
}

void FormantAgentClient::StartCommandLoop(const std::vector<std::string>& command_filter,
                                          std::function<void(const v1::model::CommandRequest&)> cb,
                                          std::function<void()> on_stream_connected) {
  stop_ = false;
  command_thread_ = std::thread([this, command_filter, cb, on_stream_connected]() {
    int reconnect_attempt = 0;
    while (!stop_) {
      std::cerr << "[formant] opening command stream target=" << target_;
      if (reconnect_attempt > 0) {
        std::cerr << " reconnect_attempt=" << reconnect_attempt;
      }
      std::cerr << std::endl;
      auto ctx = std::make_shared<grpc::ClientContext>();
      {
        std::lock_guard<std::mutex> lk(ctx_mu_);
        command_ctx_ = ctx;
      }
      v1::agent::GetCommandRequestStreamRequest req;
      for (const auto& f : command_filter) req.add_command_filter(f);
      auto stream = control_stub_->GetCommandRequestStream(ctx.get(), req);
      if (on_stream_connected) on_stream_connected();

      v1::agent::GetCommandRequestStreamResponse msg;
      while (!stop_ && stream->Read(&msg)) {
        cb(msg.request());
      }
      const auto status = stream->Finish();
      if (!stop_) {
        std::cerr << "[formant] command stream closed target=" << target_
                  << " grpc_code=" << static_cast<int>(status.error_code())
                  << " msg=" << status.error_message() << std::endl;
      }

      {
        std::lock_guard<std::mutex> lk(ctx_mu_);
        if (command_ctx_ == ctx) command_ctx_.reset();
      }
      if (stop_) break;
      ++reconnect_attempt;
      SleepWithBackoff(reconnect_attempt);
    }
  });
}

void FormantAgentClient::StartHeartbeatLoop(
    std::function<void(const v1::agent::GetTeleopHeartbeatStreamResponse&)> cb,
    std::function<void()> on_stream_connected) {
  stop_ = false;
  heartbeat_thread_ = std::thread([this, cb, on_stream_connected]() {
    int reconnect_attempt = 0;
    while (!stop_) {
      std::cerr << "[formant] opening heartbeat stream target=" << target_;
      if (reconnect_attempt > 0) {
        std::cerr << " reconnect_attempt=" << reconnect_attempt;
      }
      std::cerr << std::endl;
      auto ctx = std::make_shared<grpc::ClientContext>();
      {
        std::lock_guard<std::mutex> lk(ctx_mu_);
        heartbeat_ctx_ = ctx;
      }
      v1::agent::GetTeleopHeartbeatStreamRequest req;
      auto stream = control_stub_->GetTeleopHeartbeatStream(ctx.get(), req);
      if (on_stream_connected) on_stream_connected();

      v1::agent::GetTeleopHeartbeatStreamResponse msg;
      while (!stop_ && stream->Read(&msg)) {
        cb(msg);
      }
      const auto status = stream->Finish();
      if (!stop_) {
        std::cerr << "[formant] heartbeat stream closed target=" << target_
                  << " grpc_code=" << static_cast<int>(status.error_code())
                  << " msg=" << status.error_message() << std::endl;
      }

      {
        std::lock_guard<std::mutex> lk(ctx_mu_);
        if (heartbeat_ctx_ == ctx) heartbeat_ctx_.reset();
      }
      if (stop_) break;
      ++reconnect_attempt;
      SleepWithBackoff(reconnect_attempt);
    }
  });
}

void FormantAgentClient::StopLoops() {
  stop_ = true;
  {
    std::lock_guard<std::mutex> lk(ctx_mu_);
    if (teleop_ctx_) teleop_ctx_->TryCancel();
    if (command_ctx_) command_ctx_->TryCancel();
    if (heartbeat_ctx_) heartbeat_ctx_->TryCancel();
  }
  if (teleop_thread_.joinable()) teleop_thread_.join();
  if (command_thread_.joinable()) command_thread_.join();
  if (heartbeat_thread_.joinable()) heartbeat_thread_.join();
}

bool FormantAgentClient::GetAppConfigMap(std::unordered_map<std::string, std::string>* out) {
  if (!out) return false;
  std::lock_guard<std::mutex> lk(post_mu_);
  grpc::ClientContext ctx;
  ctx.set_deadline(std::chrono::system_clock::now() + std::chrono::milliseconds(200));
  v1::agent::GetApplicationConfigurationRequest req;
  v1::agent::GetApplicationConfigurationResponse resp;
  auto s = data_stub_->GetApplicationConfiguration(&ctx, req, &resp);
  if (!s.ok()) return false;

  const auto& m = resp.configuration().configuration_map();
  out->clear();
  for (const auto& entry : m) {
    (*out)[entry.first] = entry.second;
  }
  return true;
}

std::string FormantAgentClient::GetAppConfig(const std::string& key, const std::string& default_value) {
  std::unordered_map<std::string, std::string> values;
  if (!GetAppConfigMap(&values)) return default_value;
  const auto& m = values;
  auto it = m.find(key);
  return it == m.end() ? default_value : it->second;
}

}  // namespace fsa
