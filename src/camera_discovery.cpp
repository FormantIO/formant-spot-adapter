#include "formant_spot_adapter/camera_discovery.hpp"

#include <algorithm>
#include <utility>

#include "formant_spot_adapter/retry_backoff.hpp"

namespace fsa {

namespace {

const ImageSourceSize* find_source(const std::vector<ImageSourceSize>& listing,
                                   const std::string& name) {
  for (const auto& source : listing) {
    if (source.name == name) return &source;
  }
  return nullptr;
}

}  // namespace

CameraDiscovery::CameraDiscovery(std::vector<CameraWant> wants, long long now_ms,
                                 CameraDiscoveryTiming timing)
    : timing_(timing), next_check_ms_(now_ms), created_ms_(now_ms) {
  for (auto& want : wants) {
    if (want.stream.empty() || want.sources.empty()) continue;
    pending_.push_back(std::move(want));
  }
}

std::vector<CameraWant> CameraDiscovery::OnListFailed(long long now_ms) {
  ++unanswered_streak_;
  next_check_ms_ = now_ms + backoff_delay_ms(unanswered_streak_, timing_.unanswered_initial_ms,
                                             timing_.unanswered_max_ms);
  std::vector<CameraWant> fallback;
  if (!ever_answered_ && timing_.unanswered_fallback_ms > 0 &&
      now_ms - created_ms_ >= timing_.unanswered_fallback_ms) {
    fallback = std::move(pending_);
    pending_.clear();
    fell_back_ = true;
  }
  return fallback;
}

std::vector<CameraWant> CameraDiscovery::OnListed(const std::vector<ImageSourceSize>& listing,
                                                  long long now_ms) {
  unanswered_streak_ = 0;
  ++answers_;
  ever_answered_ = true;
  if (first_answer_ms_ < 0) first_answer_ms_ = now_ms;

  std::vector<CameraWant> ready;
  std::vector<CameraWant> still_pending;
  for (auto& want : pending_) {
    if (Unready(want, listing).empty()) {
      ready.push_back(std::move(want));
    } else {
      still_pending.push_back(std::move(want));
    }
  }
  pending_ = std::move(still_pending);

  const bool fast = (now_ms - first_answer_ms_) < timing_.missing_fast_window_ms;
  next_check_ms_ = now_ms + std::max(1LL, fast ? timing_.missing_fast_ms : timing_.missing_slow_ms);
  return ready;
}

void CameraDiscovery::OnReconnected(long long now_ms) {
  unanswered_streak_ = 0;
  answers_ = 0;
  first_answer_ms_ = -1;
  next_check_ms_ = now_ms;
}

std::string CameraDiscovery::Unready(const CameraWant& want,
                                     const std::vector<ImageSourceSize>& listing) {
  std::string why;
  for (const auto& name : want.sources) {
    const ImageSourceSize* source = find_source(listing, name);
    std::string part;
    if (!source) {
      part = "source=" + name + " missing";
    } else if (source->cols <= 0 || source->rows <= 0) {
      part = "source=" + name + " size=" + std::to_string(source->cols) + "x" +
             std::to_string(source->rows);
    }
    if (part.empty()) continue;
    if (!why.empty()) why += " ";
    why += part;
  }
  return why;
}

}  // namespace fsa
