#include <algorithm>
#include <chrono>
#include <cstdint>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"

#include "uniubi/robot_sdk/Media/Define.h"
#include "uniubi/robot_sdk/Media/FrameInfo.h"
#include "uniubi/robot_sdk/MediaBusClient.h"
#include "uniubi/robot_sdk/MotionLowLevelClient.h"
#include "uniubi/robot_sdk/MotionSdkService.h"

using namespace std::chrono_literals;

namespace uniubi_media_driver {

namespace {

std::string join_topic(const std::string & prefix, const std::string & camera_name)
{
  std::string normalized = prefix;
  while (normalized.size() > 1 && normalized.back() == '/') {
    normalized.pop_back();
  }
  if (normalized.empty() || normalized == "/") {
    return "/" + camera_name + "/image_raw/compressed";
  }
  return normalized + "/" + camera_name + "/image_raw/compressed";
}

const Uface::Stream::FrameInfo * frame_info(
  const uniubi::RobotSdk::EncodedVideoFrame & frame)
{
  return reinterpret_cast<const Uface::Stream::FrameInfo *>(frame.getExtraData());
}

bool is_jpeg(const Uface::Stream::FrameInfo * info)
{
  if (info == nullptr) {
    return false;
  }
  return info->detail.video.encode == Uface::Media::videoJpeg ||
         info->detail.video.encode == Uface::Media::videoMJpeg;
}

}  // namespace

class MediaDriverNode final : public rclcpp::Node
{
public:
  MediaDriverNode()
  : Node("uniubi_media_driver")
  {
    sdk_config_file_ = declare_parameter<std::string>("sdk_config_file", "");
    client_id_ = declare_parameter<std::string>("client_id", "uniubiMediaDriver");
    sdk_init_timeout_s_ = declare_parameter<int>("sdk_init_timeout_s", 30);
    connect_timeout_ms_ = declare_parameter<int>("connect_timeout_ms", 5000);
    topic_prefix_ = declare_parameter<std::string>("topic_prefix", "");
    camera_names_ = declare_parameter<std::vector<std::string>>(
      "camera_names", std::vector<std::string>{"front_camera_0", "front_camera_1"});
    camera_channels_ = declare_parameter<std::vector<int64_t>>(
      "camera_channels", std::vector<int64_t>{0, 1});
    frame_ids_ = declare_parameter<std::vector<std::string>>(
      "frame_ids",
      std::vector<std::string>{
        "front_camera_0_optical_frame", "front_camera_1_optical_frame"});
    lazy_subscription_ = declare_parameter<bool>("lazy_subscription", true);

    validate_parameters();
    create_publishers();
    initialize_media_bus();

    reconcile_timer_ = create_wall_timer(500ms, [this]() {reconcile_subscriptions();});
    reconcile_subscriptions();
  }

  ~MediaDriverNode() override
  {
    if (reconcile_timer_) {
      reconcile_timer_->cancel();
    }

    std::lock_guard<std::mutex> lock(media_mutex_);
    if (media_) {
      for (std::size_t i = 0; i < streams_.size(); ++i) {
        if (streams_[i].active) {
          media_->stopEncodedVideoFrame(streams_[i].channel);
          streams_[i].active = false;
        }
      }
      media_->shutdown();
      media_.reset();
    }
    if (client_) {
      client_->disconnect();
      client_.reset();
    }
    if (service_) {
      service_->setLogCallback(nullptr);
      service_->shutdown();
      service_ = nullptr;
    }
  }

private:
  struct CameraStream
  {
    int32_t channel = 0;
    std::string name;
    std::string frame_id;
    std::string topic;
    rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr publisher;
    bool active = false;
    uint64_t frames = 0;
  };

  void validate_parameters()
  {
    if (camera_names_.empty()) {
      throw std::runtime_error("camera_names must not be empty");
    }
    if (camera_names_.size() != camera_channels_.size() ||
      camera_names_.size() != frame_ids_.size())
    {
      throw std::runtime_error(
              "camera_names, camera_channels, and frame_ids must have the same length");
    }
    if (sdk_init_timeout_s_ < 1 || connect_timeout_ms_ < 1) {
      throw std::runtime_error("SDK and connection timeouts must be positive");
    }

    std::vector<int64_t> unique_channels;
    for (const auto channel : camera_channels_) {
      if (channel < 0 || channel > INT32_MAX) {
        throw std::runtime_error("camera_channels contains an invalid channel");
      }
      if (std::find(unique_channels.begin(), unique_channels.end(), channel) !=
        unique_channels.end())
      {
        throw std::runtime_error("camera_channels must not contain duplicates");
      }
      unique_channels.push_back(channel);
    }
  }

  void create_publishers()
  {
    auto qos = rclcpp::SensorDataQoS().keep_last(1).best_effort().durability_volatile();
    streams_.reserve(camera_names_.size());
    for (std::size_t i = 0; i < camera_names_.size(); ++i) {
      CameraStream stream;
      stream.channel = static_cast<int32_t>(camera_channels_[i]);
      stream.name = camera_names_[i];
      stream.frame_id = frame_ids_[i];
      stream.topic = join_topic(topic_prefix_, stream.name);
      stream.publisher = create_publisher<sensor_msgs::msg::CompressedImage>(stream.topic, qos);
      RCLCPP_INFO(
        get_logger(), "camera channel %d -> %s", stream.channel, stream.topic.c_str());
      streams_.push_back(std::move(stream));
    }
  }

  void initialize_media_bus()
  {
    service_ = uniubi::RobotSdk::IMotionSdkService::instance();
    service_->setLogCallback(
      [this](uniubi::RobotSdk::IMotionSdkService::LogLevel level, const char * msg, int32_t length) {
        if (level >= uniubi::RobotSdk::IMotionSdkService::kLogWarn) {
          RCLCPP_WARN(get_logger(), "SDK: %.*s", length, msg);
        }
      });

    const char * config = sdk_config_file_.empty() ? nullptr : sdk_config_file_.c_str();
    if (!service_->initialService(
        config, client_id_.c_str(), static_cast<uint32_t>(sdk_init_timeout_s_)))
    {
      throw std::runtime_error("Motion SDK initialization failed");
    }
    if (service_->isMultiDevice()) {
      throw std::runtime_error(
              "MediaBus is available only in local on-board deployment, not multi-device mode");
    }

    client_ = uniubi::RobotSdk::IMotionLowLevelClient::create();
    if (!client_ || !client_->connect()) {
      throw std::runtime_error("failed to start local SDK client connection");
    }

    const auto deadline = std::chrono::steady_clock::now() +
      std::chrono::milliseconds(connect_timeout_ms_);
    while (client_->getState() != uniubi::RobotSdk::IMotionLowLevelClient::kConnected) {
      if (std::chrono::steady_clock::now() >= deadline) {
        throw std::runtime_error("timed out waiting for local SDK client connection");
      }
      std::this_thread::sleep_for(50ms);
    }

    media_ = client_->createMediaBusClient();
    if (!media_ || !media_->setup()) {
      const auto error = media_ ? media_->getLastError() : -1;
      throw std::runtime_error("MediaBus setup failed, error=" + std::to_string(error));
    }

    uniubi::RobotSdk::MediaLayout layout = {};
    if (!media_->getMediaLayout(layout)) {
      throw std::runtime_error("failed to query MediaBus layout");
    }
    for (const auto & stream : streams_) {
      if (stream.channel < 0 || static_cast<uint32_t>(stream.channel) >= layout.videoEncoderNum) {
        throw std::runtime_error(
                "camera channel " + std::to_string(stream.channel) +
                " is unavailable; MediaBus reports " +
                std::to_string(layout.videoEncoderNum) + " video encoders");
      }
    }
    RCLCPP_INFO(
      get_logger(), "MediaBus ready: cameras=%u encoders=%u microphones=%u",
      layout.cameraNum, layout.videoEncoderNum, layout.micNum);
  }

  void reconcile_subscriptions()
  {
    std::lock_guard<std::mutex> lock(media_mutex_);
    if (!media_) {
      return;
    }

    for (std::size_t i = 0; i < streams_.size(); ++i) {
      const bool wanted = !lazy_subscription_ ||
        streams_[i].publisher->get_subscription_count() > 0 ||
        streams_[i].publisher->get_intra_process_subscription_count() > 0;
      if (wanted && !streams_[i].active) {
        start_stream(i);
      } else if (!wanted && streams_[i].active) {
        media_->stopEncodedVideoFrame(streams_[i].channel);
        streams_[i].active = false;
        RCLCPP_INFO(
          get_logger(), "stopped camera channel %d (no ROS subscribers)", streams_[i].channel);
      }
    }
  }

  void start_stream(std::size_t index)
  {
    const int32_t channel = streams_[index].channel;
    const bool started = media_->startEncodedVideoFrame(
      channel,
      [this, index](int32_t callback_channel,
      const uniubi::RobotSdk::EncodedVideoFrame & frame) {
        publish_frame(index, callback_channel, frame);
      });
    if (!started) {
      RCLCPP_ERROR(
        get_logger(), "failed to start camera channel %d, MediaBus error=%d",
        channel, media_->getLastError());
      return;
    }
    streams_[index].active = true;
    RCLCPP_INFO(get_logger(), "started camera channel %d", channel);
  }

  void publish_frame(
    std::size_t index, int32_t callback_channel,
    const uniubi::RobotSdk::EncodedVideoFrame & frame)
  {
    if (index >= streams_.size() || callback_channel != streams_[index].channel) {
      RCLCPP_WARN(get_logger(), "received a frame for an unexpected camera channel");
      return;
    }

    const auto * info = frame_info(frame);
    if (!is_jpeg(info)) {
      const int codec = info == nullptr ? -1 : static_cast<int>(info->detail.video.encode);
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "dropping camera channel %d codec=%d; CompressedImage output accepts JPEG only",
        callback_channel, codec);
      return;
    }

    const auto size = frame.size();
    const auto * data = frame.getBuffer();
    if (size <= 0 || data == nullptr) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "dropping an empty JPEG frame from camera channel %d", callback_channel);
      return;
    }

    auto message = std::make_unique<sensor_msgs::msg::CompressedImage>();
    message->header.stamp = now();
    message->header.frame_id = streams_[index].frame_id;
    message->format = "jpeg";
    message->data.assign(data, data + size);
    streams_[index].publisher->publish(std::move(message));
    ++streams_[index].frames;
  }

  std::string sdk_config_file_;
  std::string client_id_;
  int sdk_init_timeout_s_ = 30;
  int connect_timeout_ms_ = 5000;
  std::string topic_prefix_;
  std::vector<std::string> camera_names_;
  std::vector<int64_t> camera_channels_;
  std::vector<std::string> frame_ids_;
  bool lazy_subscription_ = true;

  uniubi::RobotSdk::IMotionSdkService * service_ = nullptr;
  std::shared_ptr<uniubi::RobotSdk::IMotionLowLevelClient> client_;
  uniubi::RobotSdk::IMediaBusClient::Ptr media_;
  std::vector<CameraStream> streams_;
  rclcpp::TimerBase::SharedPtr reconcile_timer_;
  std::mutex media_mutex_;
};

}  // namespace uniubi_media_driver

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<uniubi_media_driver::MediaDriverNode>();
    rclcpp::spin(node);
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("uniubi_media_driver"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
