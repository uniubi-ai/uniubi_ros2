#include <algorithm>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

extern "C" {
#include <libavcodec/avcodec.h>
#include <libavformat/avformat.h>
#include <libavutil/imgutils.h>
#include <libswscale/swscale.h>
}

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "rtsp_config.hpp"

using namespace std::chrono_literals;

namespace uniubi_rtsp_driver {
namespace {

struct FormatCloser {
  void operator()(AVFormatContext * p) const { if (p) avformat_close_input(&p); }
};
struct CodecCloser {
  void operator()(AVCodecContext * p) const { if (p) avcodec_free_context(&p); }
};
struct FrameCloser {
  void operator()(AVFrame * p) const { if (p) av_frame_free(&p); }
};
struct PacketCloser {
  void operator()(AVPacket * p) const { if (p) av_packet_free(&p); }
};
struct SwsCloser {
  void operator()(SwsContext * p) const { if (p) sws_freeContext(p); }
};
using FormatPtr = std::unique_ptr<AVFormatContext, FormatCloser>;
using CodecPtr = std::unique_ptr<AVCodecContext, CodecCloser>;
using FramePtr = std::unique_ptr<AVFrame, FrameCloser>;
using PacketPtr = std::unique_ptr<AVPacket, PacketCloser>;
using SwsPtr = std::unique_ptr<SwsContext, SwsCloser>;

void set_option(AVDictionary ** opts, const char * key, const char * value)
{
  av_dict_set(opts, key, value, 0);
}

}  // namespace

class RtspDriverNode final : public rclcpp::Node {
public:
  RtspDriverNode() : Node("uniubi_rtsp_driver")
  {
    av_log_set_level(AV_LOG_QUIET);  // FFmpeg errors may include URL credentials.
    rcl_interfaces::msg::ParameterDescriptor fixed;
    fixed.read_only = true;
    urls_ = declare_parameter<std::vector<std::string>>("rtsp_urls", std::vector<std::string>{}, fixed);
    names_ = declare_parameter<std::vector<std::string>>(
      "camera_names", {"front_camera_0", "front_camera_1"}, fixed);
    frame_ids_ = declare_parameter<std::vector<std::string>>(
      "frame_ids", {"front_camera_0_optical_frame", "front_camera_1_optical_frame"}, fixed);
    prefix_ = declare_parameter<std::string>("topic_prefix", "", fixed);
    open_timeout_ms_ = declare_parameter<int>("open_timeout_ms", 5000, fixed);
    read_timeout_ms_ = declare_parameter<int>("read_timeout_ms", 2000, fixed);
    reconnect_delay_ms_ = declare_parameter<int>("reconnect_delay_ms", 1000, fixed);
    publish_compressed_ = declare_parameter<bool>("publish_compressed", false, fixed);
    jpeg_quality_ = declare_parameter<int>("jpeg_quality", 80, fixed);
    validate_config(urls_, names_, frame_ids_, prefix_, open_timeout_ms_, read_timeout_ms_,
      reconnect_delay_ms_, jpeg_quality_);
    const auto qos = rclcpp::SensorDataQoS().keep_last(1).best_effort().durability_volatile();
    for (size_t i = 0; i < urls_.size(); ++i) {
      auto camera = std::make_unique<Camera>();
      camera->index = i;
      camera->topic = image_topic(prefix_, names_[i]);
      camera->raw = create_publisher<sensor_msgs::msg::Image>(camera->topic, qos);
      if (publish_compressed_) {
        camera->compressed = create_publisher<sensor_msgs::msg::CompressedImage>(
          camera->topic + "/compressed", qos);
      }
      RCLCPP_INFO(get_logger(), "camera %s -> %s", names_[i].c_str(), camera->topic.c_str());
      cameras_.push_back(std::move(camera));
    }
  }

  void start()
  {
    for (auto & camera : cameras_) {
      camera->publisher_thread = std::thread([this, c = camera.get()] {
        try {publish_loop(*c);} catch (const std::exception & e) {
          RCLCPP_ERROR(get_logger(), "camera %s publisher failed: %s", names_[c->index].c_str(), e.what());
          fail_worker();
        } catch (...) {
          RCLCPP_ERROR(get_logger(), "camera %s publisher failed", names_[c->index].c_str());
          fail_worker();
        }
      });
      camera->decoder_thread = std::thread([this, c = camera.get()] {
        try {decode_loop(*c);} catch (const std::exception & e) {
          RCLCPP_ERROR(get_logger(), "camera %s decoder failed: %s", names_[c->index].c_str(), e.what());
          fail_worker();
        } catch (...) {
          RCLCPP_ERROR(get_logger(), "camera %s decoder failed", names_[c->index].c_str());
          fail_worker();
        }
      });
    }
  }

  ~RtspDriverNode() override
  {
    stop_.store(true);
    for (auto & camera : cameras_) camera->cv.notify_all();
    for (auto & camera : cameras_) {
      if (camera->decoder_thread.joinable()) camera->decoder_thread.join();
      if (camera->publisher_thread.joinable()) camera->publisher_thread.join();
    }
  }

  bool failed() const {return fatal_.load();}

private:
  void fail_worker()
  {
    if (fatal_.exchange(true)) return;
    stop_.store(true);
    for (auto & camera : cameras_) camera->cv.notify_all();
    rclcpp::shutdown();
  }

  struct Camera {
    size_t index = 0;
    std::string topic;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr raw;
    rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr compressed;
    std::mutex mutex;
    std::condition_variable cv;
    sensor_msgs::msg::Image::UniquePtr latest;
    std::thread decoder_thread;
    std::thread publisher_thread;
  };

  // Each decoder has its own callback deadline; the callback opaque is a camera-local context.
  struct InterruptState {
    std::atomic<bool> * stop;
    std::chrono::steady_clock::time_point deadline;
  };
  static int interrupt_camera(void * opaque)
  {
    auto * state = static_cast<InterruptState *>(opaque);
    return state->stop->load() || std::chrono::steady_clock::now() >= state->deadline;
  }

  bool wait_reconnect()
  {
    auto remaining = reconnect_delay_ms_;
    while (!stop_.load() && rclcpp::ok() && remaining > 0) {
      const auto slice = std::min(remaining, 100);
      std::this_thread::sleep_for(std::chrono::milliseconds(slice));
      remaining -= slice;
    }
    return !stop_.load() && rclcpp::ok();
  }

  void decode_loop(Camera & camera)
  {
    auto next_warning = std::chrono::steady_clock::time_point::min();
    do {
      if (!decode_session(camera) && !stop_.load() && rclcpp::ok()) {
        const auto current = std::chrono::steady_clock::now();
        if (current >= next_warning) {
          RCLCPP_WARN(get_logger(), "camera %s stream unavailable; reconnecting",
            names_[camera.index].c_str());
          next_warning = current + 5s;
        }
      }
    } while (wait_reconnect());
  }

  bool decode_session(Camera & camera)
  {
    InterruptState state{&stop_, std::chrono::steady_clock::now() +
      std::chrono::milliseconds(open_timeout_ms_)};
    AVFormatContext * raw_format = avformat_alloc_context();
    if (!raw_format) return false;
    raw_format->interrupt_callback = {interrupt_camera, &state};
    AVDictionary * opts = nullptr;
    set_option(&opts, "rtsp_transport", "tcp");
    const int open_result = avformat_open_input(&raw_format, urls_[camera.index].c_str(), nullptr, &opts);
    av_dict_free(&opts);
    FormatPtr format(raw_format);
    if (open_result < 0 || !format) return false;
    state.deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(open_timeout_ms_);
    int video = av_find_best_stream(format.get(), AVMEDIA_TYPE_VIDEO, -1, -1, nullptr, 0);
    // RTSP SDP normally supplies the codec; avoid probing a live source for a full second.
    if (video < 0 || format->streams[video]->codecpar->codec_id == AV_CODEC_ID_NONE) {
      format->max_analyze_duration = 500000;
      if (avformat_find_stream_info(format.get(), nullptr) < 0 ||
        std::chrono::steady_clock::now() >= state.deadline) return false;
      video = av_find_best_stream(format.get(), AVMEDIA_TYPE_VIDEO, -1, -1, nullptr, 0);
    }
    if (video < 0) return false;
    AVStream * stream = format->streams[video];
    const AVCodec * codec = avcodec_find_decoder(stream->codecpar->codec_id);
    if (!codec) return false;
    CodecPtr decoder(avcodec_alloc_context3(codec));
    if (!decoder || avcodec_parameters_to_context(decoder.get(), stream->codecpar) < 0) return false;
    decoder->thread_count = 1;
    decoder->flags |= AV_CODEC_FLAG_LOW_DELAY;
    if (avcodec_open2(decoder.get(), codec, nullptr) < 0) return false;
    FramePtr frame(av_frame_alloc());
    PacketPtr packet(av_packet_alloc());
    if (!frame || !packet) return false;
    SwsPtr converter;
    int source_width = 0, source_height = 0;
    AVPixelFormat source_format = AV_PIX_FMT_NONE;
    RCLCPP_INFO(get_logger(), "camera %s connected", names_[camera.index].c_str());
    while (!stop_.load() && rclcpp::ok()) {
      state.deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(read_timeout_ms_);
      const int read_result = av_read_frame(format.get(), packet.get());
      if (read_result < 0) break;
      if (packet->stream_index != video) {av_packet_unref(packet.get()); continue;}
      int send_result = avcodec_send_packet(decoder.get(), packet.get());
      av_packet_unref(packet.get());
      if (send_result < 0) continue;
      while (avcodec_receive_frame(decoder.get(), frame.get()) == 0) {
        if (frame->width < 1 || frame->height < 1 || frame->width > 8192 || frame->height > 8192) {
          av_frame_unref(frame.get()); continue;
        }
        const auto pixel_format = static_cast<AVPixelFormat>(frame->format);
        if (!converter || frame->width != source_width || frame->height != source_height ||
          pixel_format != source_format) {
          converter.reset(sws_getContext(frame->width, frame->height, pixel_format,
            frame->width, frame->height, AV_PIX_FMT_BGR24, SWS_FAST_BILINEAR,
            nullptr, nullptr, nullptr));
          source_width = frame->width; source_height = frame->height; source_format = pixel_format;
        }
        if (converter) {
          auto image = std::make_unique<sensor_msgs::msg::Image>();
          image->header.stamp = now();  // ROS reception/decode time; RTSP source clock is unspecified.
          image->header.frame_id = frame_ids_[camera.index];
          image->height = static_cast<uint32_t>(frame->height);
          image->width = static_cast<uint32_t>(frame->width);
          image->encoding = "bgr8";
          image->is_bigendian = false;
          image->step = static_cast<uint32_t>(frame->width * 3);
          image->data.resize(static_cast<size_t>(image->step) * image->height);
          uint8_t * dst[4] = {image->data.data(), nullptr, nullptr, nullptr};
          int dst_stride[4] = {static_cast<int>(image->step), 0, 0, 0};
          if (sws_scale(converter.get(), frame->data, frame->linesize, 0, frame->height,
            dst, dst_stride) == frame->height) {
            std::lock_guard<std::mutex> lock(camera.mutex);
            camera.latest = std::move(image);  // Overwrite the sole pending frame.
            camera.cv.notify_one();
          }
        }
        av_frame_unref(frame.get());
      }
    }
    return stop_.load() || !rclcpp::ok();
  }

  void publish_loop(Camera & camera)
  {
    CodecPtr encoder;
    FramePtr jpeg_frame;
    PacketPtr jpeg_packet(av_packet_alloc());
    SwsPtr jpeg_converter;
    int jpeg_width = 0, jpeg_height = 0;
    int64_t jpeg_pts = 0;
    while (!stop_.load() && rclcpp::ok()) {
      sensor_msgs::msg::Image::UniquePtr image;
      {
        std::unique_lock<std::mutex> lock(camera.mutex);
        camera.cv.wait_for(lock, 100ms, [&] {return stop_.load() || camera.latest != nullptr;});
        if (stop_.load() || !rclcpp::ok()) break;
        image = std::move(camera.latest);
      }
      if (!image) continue;
      if (camera.compressed && jpeg_packet) {
        if (!encoder || !jpeg_frame || !jpeg_converter ||
          jpeg_width != static_cast<int>(image->width) ||
          jpeg_height != static_cast<int>(image->height)) {
          encoder.reset(); jpeg_frame.reset(); jpeg_converter.reset();
          const AVCodec * codec = avcodec_find_encoder(AV_CODEC_ID_MJPEG);
          if (codec) encoder.reset(avcodec_alloc_context3(codec));
          if (encoder) {
            encoder->width = static_cast<int>(image->width);
            encoder->height = static_cast<int>(image->height);
            encoder->pix_fmt = AV_PIX_FMT_YUVJ420P;
            encoder->time_base = AVRational{1, 30};
            encoder->flags |= AV_CODEC_FLAG_QSCALE;
            encoder->global_quality = (2 + (100 - jpeg_quality_) * 29 / 99) * FF_QP2LAMBDA;
            if (avcodec_open2(encoder.get(), codec, nullptr) < 0) encoder.reset();
          }
          if (encoder) {
            jpeg_frame.reset(av_frame_alloc());
            if (jpeg_frame) {
              jpeg_frame->format = encoder->pix_fmt;
              jpeg_frame->width = encoder->width;
              jpeg_frame->height = encoder->height;
              if (av_frame_get_buffer(jpeg_frame.get(), 32) < 0) jpeg_frame.reset();
            }
            if (encoder) {
              jpeg_converter.reset(sws_getContext(encoder->width, encoder->height,
                AV_PIX_FMT_BGR24, encoder->width, encoder->height, encoder->pix_fmt,
                SWS_FAST_BILINEAR, nullptr, nullptr, nullptr));
            }
          }
          jpeg_width = static_cast<int>(image->width);
          jpeg_height = static_cast<int>(image->height);
        }
        if (!encoder || !jpeg_frame || !jpeg_converter) {
          RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
            "camera %s JPEG encoder unavailable", names_[camera.index].c_str());
        }
        if (encoder && jpeg_frame && jpeg_converter && av_frame_make_writable(jpeg_frame.get()) >= 0) {
          const uint8_t * src[4] = {image->data.data(), nullptr, nullptr, nullptr};
          int src_stride[4] = {static_cast<int>(image->step), 0, 0, 0};
          jpeg_frame->quality = encoder->global_quality;
          jpeg_frame->pts = ++jpeg_pts;
          if (sws_scale(jpeg_converter.get(), src, src_stride, 0, jpeg_height,
            jpeg_frame->data, jpeg_frame->linesize) == jpeg_height &&
            avcodec_send_frame(encoder.get(), jpeg_frame.get()) >= 0 &&
            avcodec_receive_packet(encoder.get(), jpeg_packet.get()) == 0) {
            sensor_msgs::msg::CompressedImage compressed;
            compressed.header = image->header;
            compressed.format = "jpeg";
            compressed.data.assign(jpeg_packet->data, jpeg_packet->data + jpeg_packet->size);
            camera.compressed->publish(compressed);
            av_packet_unref(jpeg_packet.get());
          } else {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
              "camera %s JPEG encoding failed", names_[camera.index].c_str());
          }
        }
      }
      camera.raw->publish(std::move(image));
    }
  }

  std::vector<std::string> urls_, names_, frame_ids_;
  std::string prefix_;
  int open_timeout_ms_, read_timeout_ms_, reconnect_delay_ms_, jpeg_quality_;
  bool publish_compressed_;
  std::atomic<bool> stop_{false};
  std::atomic<bool> fatal_{false};
  std::vector<std::unique_ptr<Camera>> cameras_;
};

}  // namespace uniubi_rtsp_driver

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<uniubi_rtsp_driver::RtspDriverNode>();
    node->start();
    rclcpp::spin(node);
    const bool failed = node->failed();
    rclcpp::shutdown();
    return failed ? 1 : 0;
  } catch (const std::exception & e) {
    RCLCPP_ERROR(rclcpp::get_logger("uniubi_rtsp_driver"), "%s", e.what());
    rclcpp::shutdown();
    return 1;
  }
}
