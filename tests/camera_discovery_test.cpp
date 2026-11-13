#include <iostream>
#include <string>
#include <vector>

#include "formant_spot_adapter/camera_discovery.hpp"
#include "formant_spot_adapter/retry_backoff.hpp"

namespace {

int failures = 0;

void check(bool condition, const std::string& message) {
  if (condition) return;
  ++failures;
  std::cerr << "camera_discovery_test failed: " << message << std::endl;
}

void expect_eq(const std::string& got, const std::string& want, const std::string& message) {
  if (got == want) return;
  ++failures;
  std::cerr << "camera_discovery_test failed: " << message << ": got \"" << got << "\" want \""
            << want << "\"" << std::endl;
}

std::vector<fsa::CameraWant> walt_wants() {
  return {
      {"spot.hand.image", {"hand_color_image"}},
      {"spot.left.image", {"left_fisheye_image"}},
      {"spot.right.image", {"right_fisheye_image"}},
      {"spot.back.image", {"back_fisheye_image"}},
      {"spot.front.image", {"frontleft_fisheye_image", "frontright_fisheye_image"}},
  };
}

std::vector<fsa::ImageSourceSize> body_cameras(int right_cols, int right_rows) {
  return {
      {"left_fisheye_image", 640, 480},
      {"right_fisheye_image", right_cols, right_rows},
      {"back_fisheye_image", 640, 480},
      {"frontleft_fisheye_image", 640, 480},
      {"frontright_fisheye_image", 640, 480},
  };
}

std::vector<std::string> streams(const std::vector<fsa::CameraWant>& wants) {
  std::vector<std::string> out;
  for (const auto& w : wants) out.push_back(w.stream);
  return out;
}

std::string join(const std::vector<std::string>& v) {
  std::string out;
  for (const auto& s : v) out += (out.empty() ? "" : ",") + s;
  return out;
}

// A fake Spot for the power-on sequence on Walt (Nov 12): services answer only
// after services_up_ms; the arm's hand camera is listed only after arm_up_ms;
// the right camera reports 0x0 until right_ok_ms.
struct FakeSpot {
  long long services_up_ms{0};
  long long arm_up_ms{0};
  long long right_ok_ms{0};

  bool List(long long now, std::vector<fsa::ImageSourceSize>* out) const {
    out->clear();
    if (now < services_up_ms) return false;
    *out = body_cameras(now < right_ok_ms ? 0 : 640, now < right_ok_ms ? 0 : 480);
    if (now >= arm_up_ms) out->push_back({"hand_color_image", 640, 480});
    return true;
  }
};

}  // namespace

int main() {
  using fsa::backoff_delay_ms;
  using fsa::CameraDiscovery;

  // Backoff helper.
  check(backoff_delay_ms(0, 1000, 30000) == 0, "no failures, no wait");
  check(backoff_delay_ms(1, 1000, 30000) == 1000, "first failure waits the initial delay");
  check(backoff_delay_ms(2, 1000, 30000) == 2000, "doubles");
  check(backoff_delay_ms(5, 1000, 30000) == 16000, "16 s after 5");
  check(backoff_delay_ms(6, 1000, 30000) == 30000, "capped at 30 s");
  check(backoff_delay_ms(1000, 1000, 30000) == 30000, "no overflow after many failures");
  check(backoff_delay_ms(3, 1000, 5000) == 4000, "restore schedule: 1, 2, 4");
  check(backoff_delay_ms(4, 1000, 5000) == 5000, "restore schedule capped at the old fixed 5 s");

  // Unready text.
  {
    const fsa::CameraWant front{"spot.front.image", {"frontleft_fisheye_image", "frontright_fisheye_image"}};
    expect_eq(CameraDiscovery::Unready(front, body_cameras(640, 480)), "", "front ready");
    std::vector<fsa::ImageSourceSize> no_right = {{"frontleft_fisheye_image", 0, 0}};
    expect_eq(CameraDiscovery::Unready(front, no_right),
              "source=frontleft_fisheye_image size=0x0 source=frontright_fisheye_image missing",
              "front unready lists each source");
  }

  // Empty wants are dropped; nothing to do means Done.
  {
    CameraDiscovery d({{"", {"x"}}, {"spot.hand.image", {}}}, 0);
    check(d.Done() && !d.Due(0), "empty wants are ignored");
  }

  // Spot answers at once with everything valid: all ready on the first check.
  {
    CameraDiscovery d(walt_wants(), 1000);
    check(d.Due(1000), "first check is immediate");
    std::vector<fsa::ImageSourceSize> all = body_cameras(640, 480);
    all.push_back({"hand_color_image", 640, 480});
    const auto ready = d.OnListed(all, 1000);
    expect_eq(join(streams(ready)),
              "spot.hand.image,spot.left.image,spot.right.image,spot.back.image,spot.front.image",
              "all ready in configured order");
    check(d.Done(), "nothing pending");
  }

  // Not answering: 1, 2, 4, 8, 16, 30, 30 s between attempts.
  {
    fsa::CameraDiscoveryTiming no_fallback;
    no_fallback.unanswered_fallback_ms = 0;
    CameraDiscovery d(walt_wants(), 0, no_fallback);
    long long now = 0;
    const long long expected[] = {1000, 2000, 4000, 8000, 16000, 30000, 30000};
    for (long long gap : expected) {
      d.OnListFailed(now);
      check(d.next_check_ms() - now == gap, "unanswered gap " + std::to_string(gap) + " got " +
                                                std::to_string(d.next_check_ms() - now));
      check(!d.Due(now + gap - 1) && d.Due(now + gap), "due exactly at the gap");
      now += gap;
    }
    check(d.unanswered_streak() == 7, "streak counted");
  }

  // Spot never answers: after 2 minutes every stream starts anyway with its
  // configured sources (the behaviour before this change), once.
  {
    CameraDiscovery d(walt_wants(), 0);
    long long now = 0;
    std::vector<fsa::CameraWant> started;
    while (!d.Done() && now <= 10 * 60 * 1000) {
      if (d.Due(now)) {
        for (auto& w : d.OnListFailed(now)) started.push_back(w);
      }
      now += 100;
    }
    check(d.Done() && d.fell_back(), "fallback started everything");
    check(started.size() == 5, "all five streams started by the fallback");
    check(now - 100 >= 120000 && now - 100 <= 150000,
          "fallback at the first attempt after 2 min, at " + std::to_string(now - 100));
  }
  // Once Spot has answered, the fallback never applies.
  {
    CameraDiscovery d({{"spot.right.image", {"right_fisheye_image"}}}, 0);
    (void)d.OnListed(body_cameras(0, 0), 0);
    check(d.OnListFailed(10 * 60 * 1000).empty() && !d.Done(), "no fallback after an answer");
  }

  // Missing after an answer: every 30 s for 10 minutes, then every 5 minutes.
  {
    CameraDiscovery d({{"spot.right.image", {"right_fisheye_image"}}}, 0);
    long long now = 0;
    const auto listing = body_cameras(0, 0);
    (void)d.OnListed(listing, now);
    check(d.next_check_ms() == 30000, "first recheck in 30 s");
    now = 9 * 60 * 1000 + 59000;
    (void)d.OnListed(listing, now);
    check(d.next_check_ms() - now == 30000, "still fast just inside 10 min");
    now = 10 * 60 * 1000;
    (void)d.OnListed(listing, now);
    check(d.next_check_ms() - now == 5 * 60 * 1000, "slow from 10 min");
    // A failed listing after answers uses the fast unanswered backoff again.
    d.OnListFailed(now);
    check(d.next_check_ms() - now == 1000, "lost answers back to 1 s");
    // A reconnect checks right away and restarts the fast window.
    d.OnReconnected(now + 50);
    check(d.Due(now + 50) && d.unanswered_streak() == 0 && d.answers() == 0, "reconnect resets");
    (void)d.OnListed(listing, now + 50);
    check(d.next_check_ms() - (now + 50) == 30000, "fast window restarted after reconnect");
  }

  // The Nov 12 power-on on Walt: Spot's services come up 40 s after the
  // adapter, the right camera is 0x0 until 95 s, the arm's hand camera is
  // listed at 150 s. Every stream must end up started without a restart, and
  // the body cameras must not wait for the slow ones.
  {
    FakeSpot spot;
    spot.services_up_ms = 40000;
    spot.right_ok_ms = 95000;
    spot.arm_up_ms = 150000;
    CameraDiscovery d(walt_wants(), 0);
    std::vector<std::pair<long long, std::string>> started;
    int list_calls = 0;
    for (long long now = 0; now <= 20 * 60 * 1000 && !d.Done(); now += 100) {
      if (!d.Due(now)) continue;
      ++list_calls;
      std::vector<fsa::ImageSourceSize> listing;
      if (!spot.List(now, &listing)) {
        d.OnListFailed(now);
        continue;
      }
      for (const auto& want : d.OnListed(listing, now)) started.push_back({now, want.stream});
    }
    check(d.Done(), "every configured stream started");
    check(started.size() == 5, "five streams started, got " + std::to_string(started.size()));
    long long first_answer = -1;
    for (const auto& s : started) {
      if (s.second == "spot.left.image") first_answer = s.first;
    }
    // Attempts at 0, 1, 3, 7, 15, 31, 61 s (the gap caps at 30 s): the first after
    // services are up is 61 s.
    check(first_answer == 61000, "body cameras start on the first answer, at " +
                                     std::to_string(first_answer));
    for (const auto& s : started) {
      if (s.second == "spot.back.image" || s.second == "spot.front.image") {
        check(s.first == first_answer, s.second + " started with the first answer");
      }
      if (s.second == "spot.right.image") {
        check(s.first >= 95000 && s.first <= 95000 + 30000, "right starts within 30 s of recovering, at " +
                                                                 std::to_string(s.first));
      }
      if (s.second == "spot.hand.image") {
        check(s.first >= 150000 && s.first <= 150000 + 30000, "hand starts within 30 s of the arm, at " +
                                                                  std::to_string(s.first));
      }
    }
    check(list_calls < 20, "bounded number of ListImageSources calls: " + std::to_string(list_calls));
  }

  if (failures != 0) {
    std::cerr << failures << " check(s) failed" << std::endl;
    return 1;
  }
  std::cout << "camera_discovery_test passed" << std::endl;
  return 0;
}
