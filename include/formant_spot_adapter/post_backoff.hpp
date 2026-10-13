#pragma once

#include <string>

#include <grpcpp/grpcpp.h>

namespace fsa {

// Why a failed post to the Formant agent should slow its stream down.
enum class PostPressure {
  // Not a load signal (bad argument, cancelled, ...). The stream keeps its rate.
  kNone,
  // The agent said so explicitly: RESOURCE_EXHAUSTED or a throttle message.
  kThrottled,
  // The agent did not answer inside the post deadline. This is what a saturated
  // agent actually returns, so it has to count as backpressure.
  kDeadlineExceeded,
  // The agent could not be reached. Only counts once it is sustained, see
  // kUnavailableFailuresBeforeBackoff.
  kUnavailable,
};

constexpr int kThrottleInitialBackoffMs = 500;
constexpr int kThrottleMaxBackoffMs = 30000;
constexpr int kThrottlePaddingMinMs = 100;
// Deadline and unavailable failures are inferred load signals, not an explicit
// rate from the agent, so they are capped much lower than a real throttle and
// they recover fast: a stream backed off this way still posts at least every 5
// seconds, and each accepted post halves its interval (see
// OverloadRecoveryIntervalMs).
constexpr int kOverloadMaxBackoffMs = 5000;
// One UNAVAILABLE is a blip (agent restart, a dropped connection). Back off only
// after this many in a row on the same stream.
constexpr int kUnavailableFailuresBeforeBackoff = 3;

PostPressure ClassifyPostFailure(grpc::StatusCode code, const std::string& message);

const char* PostPressureName(PostPressure pressure);

// The interval a stream should wait between posts after a failure that counts as
// backpressure. `current_interval_ms` is the interval already learned for the
// stream (0 if none). `ms_since_last_success` is how long ago the stream last
// posted successfully (0 if never); it is only used for an explicit throttle,
// where that gap approximates the rate the agent enforces. For a timeout it
// would only measure how idle the stream was. The result never shrinks the
// interval.
int NextBackoffIntervalMs(PostPressure pressure, int current_interval_ms,
                          long long ms_since_last_success);

// The interval after an accepted post, for a stream whose interval was learned
// only from deadline or unavailable failures. Halves it and drops it entirely
// once it is small, so one timeout costs a stream about a second of reduced
// rate rather than minutes.
int OverloadRecoveryIntervalMs(int current_interval_ms);

}  // namespace fsa
