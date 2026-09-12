#pragma once
#include <deque>
#include <utility>
#include "uniubi_media_driver/msg/audio_frame.hpp"
namespace uniubi_media_driver {
// Executor-owned bounded queue. No retries/replay of failed or reset frames.
class AudioPlaybackQueue {
public:
  using Frame = msg::AudioFrame;
  static bool valid(const Frame & f) {
    return f.sample_rate == 16000 && f.sample_format == 16 &&
      f.channel_count == 1 && f.data.size() == 1280;
  }
  bool push(const Frame & f) {
    if (!valid(f) || frames_.size() >= 2) return false;
    frames_.push_back(f); return true;
  }
  bool pop(Frame & f) {
    if (frames_.empty()) return false;
    f = std::move(frames_.front()); frames_.pop_front(); return true;
  }
  void clear() {frames_.clear();}
private:
  std::deque<Frame> frames_;
};
}
