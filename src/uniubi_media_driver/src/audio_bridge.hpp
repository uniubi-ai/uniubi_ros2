#pragma once
#include <chrono>
#include <thread>
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "uniubi/robot_sdk/MediaBusClient.h"
#include "audio_playback_queue.hpp"
namespace uniubi_media_driver {
class AudioBridge {
public:
  explicit AudioBridge(rclcpp::Node & node) : node_(node) {
    rcl_interfaces::msg::ParameterDescriptor fixed; fixed.read_only = true;
    capture_ = node_.declare_parameter("audio_capture", false, fixed);
    playback_ = node_.declare_parameter("audio_playback", false, fixed);
    channel_ = node_.declare_parameter("audio_channel", 0, fixed);
    frame_id_ = node_.declare_parameter<std::string>("audio_frame_id", "microphone", fixed);
    volume_ = node_.declare_parameter("audio_volume", 20);
    if (channel_ < 0 || volume_ < 0 || volume_ > 100) {
      throw std::runtime_error("audio_channel must be nonnegative; audio_volume must be 0..100");
    }
    if (capture_) publisher_ = node_.create_publisher<msg::AudioFrame>(
      "audio/capture", rclcpp::SensorDataQoS().keep_last(5));
  }
  ~AudioBridge() {stop();}
  void start(const uniubi::RobotSdk::IMediaBusClient::Ptr & media, int timeout_ms) {
    media_ = media;
    if (playback_) {
      raw_ = media_->createAudioRawBack();
      if (!raw_ || !raw_->setup()) throw std::runtime_error("RawBack setup failed");
      auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms);
      while (!raw_->ready()) {
        if (!rclcpp::ok() || std::chrono::steady_clock::now() >= deadline)
          throw std::runtime_error("RawBack ready timeout");
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
      }
      if (!raw_->setVolume(volume_)) throw std::runtime_error("RawBack volume failed");
      subscription_ = node_.create_subscription<msg::AudioFrame>(
        "audio/playback", rclcpp::SensorDataQoS().keep_last(2),
        [this](msg::AudioFrame::ConstSharedPtr f) {
          if (!raw_->ready() || !queue_.push(*f)) {
            RCLCPP_WARN_THROTTLE(node_.get_logger(), *node_.get_clock(), 2000,
              "Dropping playback frame: not ready, invalid PCM format/size, or queue full");
          }
        });
      timer_ = node_.create_wall_timer(std::chrono::milliseconds(40), [this]() {
        msg::AudioFrame f;
        if (!raw_->ready()) {queue_.clear(); return;}
        if (!queue_.pop(f)) return;
        uniubi::RobotSdk::AudioFrame frame(&f.data[0], f.data.size());
        auto & info = frame.getAudioFrameInfo();
        info.sampleRate = 16000; info.sampleFormat = 16; info.channelCount = 1;
        info.dataType = 0; info.sequence = f.sequence;
        info.timestamp = std::chrono::duration_cast<std::chrono::microseconds>(
          std::chrono::steady_clock::now().time_since_epoch()).count();
        if (!raw_->write(frame)) {
          queue_.clear();
          RCLCPP_ERROR(node_.get_logger(), "RawBack write failed: %d", raw_->getLastError());
        }
      });
      reset_ = node_.create_service<std_srvs::srv::Trigger>("audio/reset",
        [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
          std::shared_ptr<std_srvs::srv::Trigger::Response> out) {
          queue_.clear(); out->success = raw_->reset();
          out->message = out->success ? "Playback buffers cleared" : "RawBack reset failed";
        });
    }
    parameters_ = node_.add_on_set_parameters_callback([this](const auto & params) {
      rcl_interfaces::msg::SetParametersResult result; result.successful = true;
      int volume = volume_;
      for (const auto & p : params) if (p.get_name() == "audio_volume") {
        if (p.get_type() != rclcpp::ParameterType::PARAMETER_INTEGER ||
          p.as_int() < 0 || p.as_int() > 100) {
          result.successful = false; result.reason = "audio_volume must be 0..100"; return result;
        }
        volume = static_cast<int>(p.as_int());
      }
      if (volume != volume_ && raw_ && !raw_->setVolume(volume)) {
        result.successful = false; result.reason = "RawBack setVolume failed"; return result;
      }
      volume_ = volume; return result;
    });
    if (capture_) {
      active_ = media_->startRawAudioFrame(channel_, [this](int32_t channel,
        const uniubi::RobotSdk::AudioFrame & frame) {
        try {
          if (!frame.valid() || frame.size() <= 0 || frame.size() > 65536 || !frame.data()) return;
          const auto & info = frame.getFrameInfo();
          msg::AudioFrame out; out.header.stamp = node_.now(); out.header.frame_id = frame_id_;
          out.device_timestamp_us = info.timestamp; out.sequence = info.sequence;
          out.sample_rate = info.sampleRate; out.sample_format = info.sampleFormat;
          out.channel_count = info.channelCount; out.channel = channel;
          out.data.assign(frame.data(), frame.data() + frame.size());
          publisher_->publish(out);
        } catch (const std::exception & e) {
          RCLCPP_ERROR(node_.get_logger(), "Audio capture publish failed: %s", e.what());
        }
      });
      if (!active_) throw std::runtime_error("Audio capture start failed");
    }
  }
  void stop() {
    if (timer_) timer_->cancel();
    parameters_.reset(); subscription_.reset(); reset_.reset();
    queue_.clear();
    if (media_ && active_) {media_->stopRawAudioFrame(channel_); active_ = false;}
    if (raw_) {raw_->shutdown(); raw_.reset();}
    media_.reset();
  }
private:
  rclcpp::Node & node_;
  bool capture_, playback_, active_ = false;
  int channel_, volume_;
  std::string frame_id_;
  AudioPlaybackQueue queue_;
  uniubi::RobotSdk::IMediaBusClient::Ptr media_;
  uniubi::RobotSdk::IAudioRawBackStream::Ptr raw_;
  rclcpp::Publisher<msg::AudioFrame>::SharedPtr publisher_;
  rclcpp::Subscription<msg::AudioFrame>::SharedPtr subscription_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameters_;
};
}
