#include <gtest/gtest.h>
#include "audio_playback_queue.hpp"
using uniubi_media_driver::AudioPlaybackQueue;
TEST(AudioQueue, FormatAndBoundedBacklog) {
  AudioPlaybackQueue q; AudioPlaybackQueue::Frame f, out;
  f.sample_rate=16000; f.sample_format=16; f.channel_count=1; f.data.resize(1280);
  auto invalid=f; invalid.sample_rate=48000; EXPECT_FALSE(q.push(invalid));
  invalid=f; invalid.sample_format=32; EXPECT_FALSE(q.push(invalid));
  invalid=f; invalid.channel_count=2; EXPECT_FALSE(q.push(invalid));
  invalid=f; invalid.data.resize(1279); EXPECT_FALSE(q.push(invalid));
  f.sequence=1; ASSERT_TRUE(q.push(f)); f.sequence=2; ASSERT_TRUE(q.push(f));
  f.sequence=3; EXPECT_FALSE(q.push(f));
  ASSERT_TRUE(q.pop(out)); EXPECT_EQ(out.sequence,1u);
  ASSERT_TRUE(q.pop(out)); EXPECT_EQ(out.sequence,2u); EXPECT_FALSE(q.pop(out));
}
TEST(AudioQueue, ResetDropsPendingFrames) {
  AudioPlaybackQueue q; AudioPlaybackQueue::Frame f,out;
  f.sample_rate=16000; f.sample_format=16; f.channel_count=1; f.data.resize(1280);
  ASSERT_TRUE(q.push(f)); q.clear(); EXPECT_FALSE(q.pop(out));
  ASSERT_TRUE(q.push(f)); EXPECT_TRUE(q.pop(out));
}
