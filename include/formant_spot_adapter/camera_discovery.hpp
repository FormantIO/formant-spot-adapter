#pragma once

#include <string>
#include <vector>

namespace fsa {

// After a power-on the adapter can start before Spot's services are up, so the
// first ListImageSources fails, and a body camera with a fault can be listed at
// 0x0. Before DEV-17754 the first case left every camera stream unchecked and
// the second disabled the stream until the adapter restarted. CameraDiscovery
// keeps asking until each configured stream's sources are listed with a real
// size, then hands the stream over to be started.

struct ImageSourceSize {
  std::string name;
  int cols{0};
  int rows{0};
};

// A camera stream waiting for its Spot image sources. The stitched front
// stream needs two; the others need one.
struct CameraWant {
  std::string stream;
  std::vector<std::string> sources;
};

struct CameraDiscoveryTiming {
  // While Spot does not answer ListImageSources (not connected, services not
  // registered yet): 1 s, 2 s, 4 s ... up to 30 s between attempts.
  long long unanswered_initial_ms{1000};
  long long unanswered_max_ms{30000};
  // Once Spot answers, a source that is missing or 0x0 is checked again every
  // 30 s for 10 minutes, then every 5 minutes. The arm cameras normally come up
  // within a couple of minutes of the body; a camera fault can last longer.
  long long missing_fast_ms{30000};
  long long missing_fast_window_ms{10 * 60 * 1000};
  long long missing_slow_ms{5 * 60 * 1000};
  // If Spot has never answered this long after startup, start every pending
  // stream anyway with its configured sources, as the adapter did before
  // DEV-17754: a camera loop waits while Spot is unreachable and publishes once
  // GetImage works, so a ListImageSources that never succeeds must not keep
  // every camera dark. 0 or less turns this off.
  long long unanswered_fallback_ms{2 * 60 * 1000};
};

class CameraDiscovery {
 public:
  CameraDiscovery(std::vector<CameraWant> wants, long long now_ms,
                  CameraDiscoveryTiming timing = CameraDiscoveryTiming());

  bool Due(long long now_ms) const { return !Done() && now_ms >= next_check_ms_; }
  bool Done() const { return pending_.empty(); }
  long long next_check_ms() const { return next_check_ms_; }
  const std::vector<CameraWant>& pending() const { return pending_; }
  // Consecutive attempts Spot has not answered (0 after an answer).
  int unanswered_streak() const { return unanswered_streak_; }
  // Listings Spot has answered since the last reconnect.
  int answers() const { return answers_; }

  // Spot did not answer: not connected, or the RPC failed. Returns the wants to
  // start without a source list when the fallback is due (see
  // CameraDiscoveryTiming::unanswered_fallback_ms), otherwise nothing.
  std::vector<CameraWant> OnListFailed(long long now_ms);
  // True once the fallback has started streams without a source list.
  bool fell_back() const { return fell_back_; }
  // Spot answered. Returns the wants whose sources are all listed with a
  // non-zero size, in configured order; they leave the pending set.
  std::vector<CameraWant> OnListed(const std::vector<ImageSourceSize>& listing, long long now_ms);
  // The connection to Spot came back (possibly after a robot reboot): ask
  // again right away and restart both schedules.
  void OnReconnected(long long now_ms);

  // Why a want is not ready in this listing, for the log; "" when it is ready.
  // e.g. "source=right_fisheye_image size=0x0" or "source=hand_color_image missing".
  static std::string Unready(const CameraWant& want, const std::vector<ImageSourceSize>& listing);

 private:
  std::vector<CameraWant> pending_;
  CameraDiscoveryTiming timing_;
  long long next_check_ms_{0};
  long long created_ms_{0};
  long long first_answer_ms_{-1};
  bool ever_answered_{false};
  bool fell_back_{false};
  int unanswered_streak_{0};
  int answers_{0};
};

}  // namespace fsa
