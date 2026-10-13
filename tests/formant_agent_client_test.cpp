// Exercises FormantAgentClient against an in-process fake Formant agent, so the
// backpressure and channel behavior is tested over real gRPC without a robot.

#include <chrono>
#include <condition_variable>
#include <deque>
#include <iostream>
#include <memory>
#include <mutex>
#include <set>
#include <string>
#include <thread>
#include <unordered_map>

#include <grpcpp/grpcpp.h>

#include "formant_spot_adapter/formant_agent_client.hpp"
#include "formant_spot_adapter/post_backoff.hpp"

namespace {

int failures = 0;

void check(bool condition, const char* message) {
  if (condition) return;
  ++failures;
  std::cerr << "formant_agent_client_test failed: " << message << std::endl;
}

long long now_ms() {
  using namespace std::chrono;
  return duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count();
}

// What the fake agent does with the next PostData for a stream. Steps are
// consumed in order; once a stream's script is empty every post succeeds.
struct PostStep {
  grpc::StatusCode code{grpc::StatusCode::OK};
  std::string message;
  int delay_ms{0};
};

class FakeAgent final : public v1::agent::Agent::Service {
 public:
  void Script(const std::string& stream, std::deque<PostStep> steps) {
    std::lock_guard<std::mutex> lk(mu_);
    scripts_[stream] = std::move(steps);
  }

  int PostCount(const std::string& stream) {
    std::lock_guard<std::mutex> lk(mu_);
    return post_counts_[stream];
  }

  std::set<std::string> DataPeers() {
    std::lock_guard<std::mutex> lk(mu_);
    return data_peers_;
  }

  std::set<std::string> ControlPeers() {
    std::lock_guard<std::mutex> lk(mu_);
    return control_peers_;
  }

  std::string LastResponseRequestId() {
    std::lock_guard<std::mutex> lk(mu_);
    return last_response_request_id_;
  }

  grpc::Status PostData(grpc::ServerContext* ctx, const v1::model::Datapoint* dp,
                        v1::agent::PostDataResponse*) override {
    PostStep step;
    {
      std::lock_guard<std::mutex> lk(mu_);
      data_peers_.insert(ctx->peer());
      post_counts_[dp->stream()] += 1;
      auto it = scripts_.find(dp->stream());
      if (it != scripts_.end() && !it->second.empty()) {
        step = it->second.front();
        it->second.pop_front();
      }
    }
    if (step.delay_ms > 0) {
      std::this_thread::sleep_for(std::chrono::milliseconds(step.delay_ms));
    }
    if (step.code == grpc::StatusCode::OK) return grpc::Status::OK;
    return grpc::Status(step.code, step.message);
  }

  grpc::Status GetCommandRequestStream(
      grpc::ServerContext* ctx, const v1::agent::GetCommandRequestStreamRequest*,
      grpc::ServerWriter<v1::agent::GetCommandRequestStreamResponse>* writer) override {
    {
      std::lock_guard<std::mutex> lk(mu_);
      control_peers_.insert(ctx->peer());
    }
    v1::agent::GetCommandRequestStreamResponse msg;
    msg.mutable_request()->set_id("req-1");
    msg.mutable_request()->set_command("spot.stand");
    writer->Write(msg);
    while (!ctx->IsCancelled()) {
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    return grpc::Status::OK;
  }

  grpc::Status SendCommandResponse(grpc::ServerContext* ctx,
                                   const v1::agent::SendCommandResponseRequest* req,
                                   v1::agent::SendCommandResponseResponse*) override {
    std::lock_guard<std::mutex> lk(mu_);
    control_peers_.insert(ctx->peer());
    last_response_request_id_ = req->response().request_id();
    return grpc::Status::OK;
  }

 private:
  std::mutex mu_;
  std::unordered_map<std::string, std::deque<PostStep>> scripts_;
  std::unordered_map<std::string, int> post_counts_;
  std::set<std::string> data_peers_;
  std::set<std::string> control_peers_;
  std::string last_response_request_id_;
};

void test_classification() {
  using fsa::ClassifyPostFailure;
  using fsa::PostPressure;
  check(ClassifyPostFailure(grpc::StatusCode::RESOURCE_EXHAUSTED, "") == PostPressure::kThrottled,
        "RESOURCE_EXHAUSTED should classify as throttled");
  check(ClassifyPostFailure(grpc::StatusCode::UNKNOWN, "stream x is being throttled") ==
            PostPressure::kThrottled,
        "throttle message should classify as throttled");
  check(ClassifyPostFailure(grpc::StatusCode::UNKNOWN, "rate limit hit") ==
            PostPressure::kThrottled,
        "rate limit message should classify as throttled");
  check(ClassifyPostFailure(grpc::StatusCode::DEADLINE_EXCEEDED, "Deadline Exceeded") ==
            PostPressure::kDeadlineExceeded,
        "DEADLINE_EXCEEDED should classify as deadline pressure");
  check(ClassifyPostFailure(grpc::StatusCode::UNAVAILABLE, "Connect Failed") ==
            PostPressure::kUnavailable,
        "UNAVAILABLE should classify as unavailable");
  check(ClassifyPostFailure(grpc::StatusCode::INVALID_ARGUMENT, "bad stream") ==
            PostPressure::kNone,
        "INVALID_ARGUMENT should not be a load signal");
  check(ClassifyPostFailure(grpc::StatusCode::CANCELLED, "") == PostPressure::kNone,
        "CANCELLED should not be a load signal");
}

void test_backoff_intervals() {
  using fsa::NextBackoffIntervalMs;
  using fsa::PostPressure;

  check(NextBackoffIntervalMs(PostPressure::kDeadlineExceeded, 0, 0) ==
            fsa::kThrottleInitialBackoffMs,
        "first deadline failure should start at the initial backoff");
  check(NextBackoffIntervalMs(PostPressure::kNone, 1234, 99999) == 1234,
        "a non-load failure should leave the interval alone");

  int interval_ms = 0;
  int previous_ms = 0;
  for (int i = 0; i < 40; ++i) {
    interval_ms = NextBackoffIntervalMs(PostPressure::kDeadlineExceeded, interval_ms, 0);
    check(interval_ms >= previous_ms, "deadline backoff should never shrink");
    previous_ms = interval_ms;
  }
  check(interval_ms == fsa::kOverloadMaxBackoffMs,
        "repeated deadline failures should cap at the overload maximum");

  interval_ms = 0;
  for (int i = 0; i < 40; ++i) {
    interval_ms = NextBackoffIntervalMs(PostPressure::kThrottled, interval_ms, 0);
  }
  check(interval_ms == fsa::kThrottleMaxBackoffMs,
        "repeated explicit throttles should cap at the throttle maximum");

  check(NextBackoffIntervalMs(PostPressure::kDeadlineExceeded, 20000, 0) == 20000,
        "a deadline failure must not shrink an interval learned from an explicit throttle");
  check(NextBackoffIntervalMs(PostPressure::kDeadlineExceeded, 0, 60000) ==
            fsa::kThrottleInitialBackoffMs,
        "a timeout on an idle stream should not jump straight to the cap");
  check(NextBackoffIntervalMs(PostPressure::kUnavailable, 0, 1000000000000LL) ==
            fsa::kThrottleInitialBackoffMs,
        "a very long outage should start at the initial backoff");
  check(NextBackoffIntervalMs(PostPressure::kThrottled, 0, 1000000000000LL) ==
            fsa::kThrottleMaxBackoffMs,
        "a very long gap before a throttle should clamp without overflowing");

  check(fsa::OverloadRecoveryIntervalMs(5000) == 2500, "recovery should halve the interval");
  check(fsa::OverloadRecoveryIntervalMs(500) == 250, "recovery should halve the initial backoff");
  check(fsa::OverloadRecoveryIntervalMs(150) == 0, "recovery should drop a small interval");
  check(fsa::OverloadRecoveryIntervalMs(0) == 0, "recovery of no interval is no interval");
  check(NextBackoffIntervalMs(PostPressure::kThrottled, 0, 2000) == 2500,
        "an explicit throttle should learn the observed accept interval plus padding");
}

void sleep_until(long long deadline_ms) {
  while (now_ms() < deadline_ms) {
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
}

// The first RPC on a channel pays for the TCP connect. Do that up front so the
// short-deadline posts below only time out because the fake agent is slow.
void warm_up_data_channel(fsa::FormantAgentClient& client) {
  bool connected = false;
  for (int i = 0; i < 20 && !connected; ++i) {
    connected = client.PostNumeric("warmup." + std::to_string(i), 0.0).ok;
  }
  check(connected, "data channel should connect to the fake agent");
}

void test_deadline_exceeded_backs_off(fsa::FormantAgentClient& client, FakeAgent& agent) {
  agent.Script("slow.image", {{grpc::StatusCode::OK, "", 400}});

  const long long before = now_ms();
  const auto first = client.PostImage("slow.image", "image/jpeg", "jpeg-bytes", 40);
  check(!first.ok, "post to a stalled agent should fail");
  check(first.grpc_code == static_cast<int>(grpc::StatusCode::DEADLINE_EXCEEDED),
        "stalled post should surface DEADLINE_EXCEEDED");
  check(first.throttled, "DEADLINE_EXCEEDED should engage the stream backoff");
  check(first.retry_after_ms >= before + fsa::kThrottleInitialBackoffMs,
        "DEADLINE_EXCEEDED should return a retry time at least the initial backoff away");

  const auto second = client.PostImage("slow.image", "image/jpeg", "jpeg-bytes", 40);
  check(second.locally_suppressed, "the next post inside the backoff should be suppressed locally");
  check(agent.PostCount("slow.image") == 1,
        "a suppressed post must not reach the agent at all");

  const auto other = client.PostNumeric("healthy.numeric", 1.0);
  check(other.ok, "backoff on one stream must not affect another stream");

  // Each accepted post halves the interval, so a single timeout is forgotten
  // after a few good posts: 500 ms, then 250, then 125, then none.
  sleep_until(first.retry_after_ms + 20);
  const auto recovered = client.PostImage("slow.image", "image/jpeg", "jpeg-bytes", 500);
  check(recovered.ok, "stream should post again after the backoff elapses");
  const auto still_paced = client.PostImage("slow.image", "image/jpeg", "jpeg-bytes", 500);
  check(still_paced.locally_suppressed, "the first good post should not clear the backoff");
  check(still_paced.retry_after_ms - now_ms() <= fsa::kThrottleInitialBackoffMs / 2,
        "the first good post should halve the backoff");

  sleep_until(still_paced.retry_after_ms + 20);
  check(client.PostImage("slow.image", "image/jpeg", "jpeg-bytes", 500).ok,
        "second good post after the halved backoff");
  sleep_until(now_ms() + fsa::kThrottleInitialBackoffMs / 4 + 20);
  check(client.PostImage("slow.image", "image/jpeg", "jpeg-bytes", 500).ok,
        "third good post after the quartered backoff");
  const auto full_rate_a = client.PostImage("slow.image", "image/jpeg", "jpeg-bytes", 500);
  const auto full_rate_b = client.PostImage("slow.image", "image/jpeg", "jpeg-bytes", 500);
  check(full_rate_a.ok && full_rate_b.ok,
        "after three good posts the stream should be back at full rate");
}

void test_unavailable_must_be_sustained(fsa::FormantAgentClient& client, FakeAgent& agent) {
  const PostStep down{grpc::StatusCode::UNAVAILABLE, "agent restarting", 0};

  agent.Script("down.text", {down, down, down});
  const auto first = client.PostText("down.text", "a");
  const auto second = client.PostText("down.text", "b");
  check(!first.throttled && !second.throttled,
        "the first UNAVAILABLE failures are a blip and should not back off");
  const auto third = client.PostText("down.text", "c");
  check(third.throttled, "sustained UNAVAILABLE should engage the stream backoff");
  check(third.grpc_code == static_cast<int>(grpc::StatusCode::UNAVAILABLE),
        "sustained UNAVAILABLE should keep its grpc code");

  agent.Script("flaky.text", {down, down, PostStep{}, down, down});
  bool any_throttled = false;
  for (int i = 0; i < 5; ++i) {
    any_throttled = client.PostText("flaky.text", "x").throttled || any_throttled;
  }
  check(!any_throttled, "a success should reset the UNAVAILABLE streak");
  check(agent.PostCount("flaky.text") == 5, "no flaky post should have been suppressed");
}

void test_other_failures_do_not_back_off(fsa::FormantAgentClient& client, FakeAgent& agent) {
  const PostStep bad{grpc::StatusCode::INVALID_ARGUMENT, "unknown stream", 0};
  agent.Script("bad.numeric", {bad, bad, bad, bad, bad});
  bool any_throttled = false;
  for (int i = 0; i < 5; ++i) {
    any_throttled = client.PostNumeric("bad.numeric", 1.0).throttled || any_throttled;
  }
  check(!any_throttled, "INVALID_ARGUMENT should never back off a stream");
  check(agent.PostCount("bad.numeric") == 5, "no INVALID_ARGUMENT post should be suppressed");
}

void test_explicit_throttle_still_backs_off(fsa::FormantAgentClient& client, FakeAgent& agent) {
  agent.Script("throttled.numeric",
               {{grpc::StatusCode::RESOURCE_EXHAUSTED,
                 "stream throttled.numeric is being throttled", 0}});
  const auto first = client.PostNumeric("throttled.numeric", 1.0);
  check(first.throttled, "RESOURCE_EXHAUSTED should still engage the stream backoff");
  const auto second = client.PostNumeric("throttled.numeric", 2.0);
  check(second.locally_suppressed, "post after an explicit throttle should be suppressed locally");

  // An explicit throttle keeps its slow recovery: one accepted post does not
  // shorten the interval the agent asked for.
  sleep_until(first.retry_after_ms + 20);
  check(client.PostNumeric("throttled.numeric", 3.0).ok, "post after the throttle interval");
  const auto paced = client.PostNumeric("throttled.numeric", 4.0);
  check(paced.locally_suppressed, "an explicit throttle should still pace the next post");
  check(paced.retry_after_ms - now_ms() > fsa::kThrottleInitialBackoffMs / 2,
        "one good post must not halve an explicit throttle interval");
}

void test_control_uses_its_own_connection(fsa::FormantAgentClient& client, FakeAgent& agent) {
  std::mutex mu;
  std::condition_variable cv;
  std::string received_id;

  client.StartCommandLoop({}, [&](const v1::model::CommandRequest& request) {
    std::lock_guard<std::mutex> lk(mu);
    received_id = request.id();
    cv.notify_all();
  });
  {
    std::unique_lock<std::mutex> lk(mu);
    cv.wait_for(lk, std::chrono::seconds(5), [&] { return !received_id.empty(); });
  }
  check(received_id == "req-1", "command stream should deliver the command");
  check(client.SendCommandResponse("req-1", true), "command response should be accepted");
  check(agent.LastResponseRequestId() == "req-1", "command response should reach the agent");
  client.StopLoops();

  const auto data_peers = agent.DataPeers();
  const auto control_peers = agent.ControlPeers();
  check(!data_peers.empty(), "telemetry posts should have reached the agent");
  check(control_peers.size() == 1, "command stream and response should share one connection");
  bool shared = false;
  for (const auto& peer : control_peers) {
    shared = shared || data_peers.count(peer) > 0;
  }
  check(!shared, "control traffic must not share a connection with telemetry posts");
}

}  // namespace

int main() {
  test_classification();
  test_backoff_intervals();

  FakeAgent agent;
  int port = 0;
  grpc::ServerBuilder builder;
  builder.AddListeningPort("127.0.0.1:0", grpc::InsecureServerCredentials(), &port);
  builder.RegisterService(&agent);
  std::unique_ptr<grpc::Server> server = builder.BuildAndStart();
  if (!server || port == 0) {
    std::cerr << "formant_agent_client_test failed: could not start fake agent" << std::endl;
    return 1;
  }

  {
    fsa::FormantAgentClient client("127.0.0.1:" + std::to_string(port));
    warm_up_data_channel(client);
    test_deadline_exceeded_backs_off(client, agent);
    test_unavailable_must_be_sustained(client, agent);
    test_other_failures_do_not_back_off(client, agent);
    test_explicit_throttle_still_backs_off(client, agent);
    test_control_uses_its_own_connection(client, agent);
  }

  server->Shutdown(std::chrono::system_clock::now() + std::chrono::seconds(2));

  if (failures > 0) {
    std::cerr << "formant_agent_client_test: " << failures << " check(s) failed" << std::endl;
    return 1;
  }
  std::cout << "formant_agent_client_test: all checks passed" << std::endl;
  return 0;
}
