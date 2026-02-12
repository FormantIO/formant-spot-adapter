#include "formant_spot_adapter/formant_agent_client.hpp"

#include <chrono>
#include <iostream>

namespace fsa {

FormantAgentClient::FormantAgentClient(const std::string& target)
    : target_(target),
      stub_(v1::agent::Agent::NewStub(grpc::CreateChannel(target, grpc::InsecureChannelCredentials()))) {
  std::cerr << "[formant] target=" << target_ << std::endl;
}

long long FormantAgentClient::NowMs() const {
  using namespace std::chrono;
  return duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count();
}

bool FormantAgentClient::PostImage(const std::string& stream, const std::string& content_type, const std::string& bytes) {
  v1::model::Datapoint dp;
  dp.set_stream(stream);
  dp.set_timestamp(NowMs());
  dp.mutable_image()->set_content_type(content_type);
  dp.mutable_image()->set_raw(bytes);

  grpc::ClientContext ctx;
  ctx.set_deadline(std::chrono::system_clock::now() + std::chrono::milliseconds(500));
  v1::agent::PostDataResponse resp;
  auto s = stub_->PostData(&ctx, dp, &resp);
  if (!s.ok()) {
    std::cerr << "[formant] PostImage failed stream=" << stream
              << " target=" << target_
              << " grpc_code=" << static_cast<int>(s.error_code())
              << " msg=" << s.error_message() << std::endl;
  }
  return s.ok();
}

bool FormantAgentClient::PostBitset(const std::string& stream, const std::vector<std::pair<std::string, bool>>& bits) {
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
  auto s = stub_->PostData(&ctx, dp, &resp);
  if (!s.ok()) {
    std::cerr << "[formant] PostBitset failed stream=" << stream
              << " target=" << target_
              << " grpc_code=" << static_cast<int>(s.error_code())
              << " msg=" << s.error_message() << std::endl;
  }
  return s.ok();
}

bool FormantAgentClient::SendCommandResponse(const std::string& request_id, bool success) {
  v1::agent::SendCommandResponseRequest req;
  req.mutable_response()->set_request_id(request_id);
  req.mutable_response()->set_success(success);

  grpc::ClientContext ctx;
  ctx.set_deadline(std::chrono::system_clock::now() + std::chrono::milliseconds(500));
  v1::agent::SendCommandResponseResponse resp;
  auto s = stub_->SendCommandResponse(&ctx, req, &resp);
  if (!s.ok()) {
    std::cerr << "[formant] SendCommandResponse failed target=" << target_
              << " grpc_code=" << static_cast<int>(s.error_code())
              << " msg=" << s.error_message() << std::endl;
  }
  return s.ok();
}

void FormantAgentClient::StartTeleopLoop(const std::vector<std::string>& stream_filter,
                                         std::function<void(const v1::model::ControlDatapoint&)> cb) {
  stop_ = false;
  teleop_thread_ = std::thread([this, stream_filter, cb]() {
    std::cerr << "[formant] opening teleop stream target=" << target_ << std::endl;
    auto ctx = std::make_shared<grpc::ClientContext>();
    {
      std::lock_guard<std::mutex> lk(ctx_mu_);
      teleop_ctx_ = ctx;
    }
    v1::agent::GetTeleopControlDataStreamRequest req;
    for (const auto& f : stream_filter) req.add_stream_filter(f);
    auto stream = stub_->GetTeleopControlDataStream(ctx.get(), req);

    v1::agent::GetTeleopControlDataStreamResponse msg;
    while (!stop_ && stream->Read(&msg)) {
      cb(msg.control_datapoint());
    }
    const auto status = stream->Finish();
    std::cerr << "[formant] teleop stream closed target=" << target_
              << " grpc_code=" << static_cast<int>(status.error_code())
              << " msg=" << status.error_message() << std::endl;

    {
      std::lock_guard<std::mutex> lk(ctx_mu_);
      teleop_ctx_.reset();
    }
  });
}

void FormantAgentClient::StartCommandLoop(const std::vector<std::string>& command_filter,
                                          std::function<void(const v1::model::CommandRequest&)> cb) {
  stop_ = false;
  command_thread_ = std::thread([this, command_filter, cb]() {
    std::cerr << "[formant] opening command stream target=" << target_ << std::endl;
    auto ctx = std::make_shared<grpc::ClientContext>();
    {
      std::lock_guard<std::mutex> lk(ctx_mu_);
      command_ctx_ = ctx;
    }
    v1::agent::GetCommandRequestStreamRequest req;
    for (const auto& f : command_filter) req.add_command_filter(f);
    auto stream = stub_->GetCommandRequestStream(ctx.get(), req);

    v1::agent::GetCommandRequestStreamResponse msg;
    while (!stop_ && stream->Read(&msg)) {
      cb(msg.request());
    }
    const auto status = stream->Finish();
    std::cerr << "[formant] command stream closed target=" << target_
              << " grpc_code=" << static_cast<int>(status.error_code())
              << " msg=" << status.error_message() << std::endl;

    {
      std::lock_guard<std::mutex> lk(ctx_mu_);
      command_ctx_.reset();
    }
  });
}

void FormantAgentClient::StartHeartbeatLoop(std::function<void(const v1::agent::GetTeleopHeartbeatStreamResponse&)> cb) {
  stop_ = false;
  heartbeat_thread_ = std::thread([this, cb]() {
    std::cerr << "[formant] opening heartbeat stream target=" << target_ << std::endl;
    auto ctx = std::make_shared<grpc::ClientContext>();
    {
      std::lock_guard<std::mutex> lk(ctx_mu_);
      heartbeat_ctx_ = ctx;
    }
    v1::agent::GetTeleopHeartbeatStreamRequest req;
    auto stream = stub_->GetTeleopHeartbeatStream(ctx.get(), req);

    v1::agent::GetTeleopHeartbeatStreamResponse msg;
    while (!stop_ && stream->Read(&msg)) {
      cb(msg);
    }
    const auto status = stream->Finish();
    std::cerr << "[formant] heartbeat stream closed target=" << target_
              << " grpc_code=" << static_cast<int>(status.error_code())
              << " msg=" << status.error_message() << std::endl;

    {
      std::lock_guard<std::mutex> lk(ctx_mu_);
      heartbeat_ctx_.reset();
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

std::string FormantAgentClient::GetAppConfig(const std::string& key, const std::string& default_value) {
  grpc::ClientContext ctx;
  v1::agent::GetApplicationConfigurationRequest req;
  v1::agent::GetApplicationConfigurationResponse resp;
  auto s = stub_->GetApplicationConfiguration(&ctx, req, &resp);
  if (!s.ok()) return default_value;

  const auto& m = resp.configuration().configuration_map();
  auto it = m.find(key);
  return it == m.end() ? default_value : it->second;
}

}  // namespace fsa
