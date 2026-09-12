#include <gtest/gtest.h>
#include "audio_bridge.hpp"
using namespace uniubi_media_driver;
using namespace uniubi::RobotSdk;
class FakeRaw : public IAudioRawBackStream {
public:
  bool reset() override {++resets; return true;}
  bool setup() override {return true;}
  void shutdown() override {++shutdowns;}
  bool ready() const override {return true;}
  int32_t getLastError() const override {return 0;}
  bool setVolume(int32_t v) override {volume=v; return true;}
  bool write(const AudioFrame & f) override {++writes; bytes=f.size(); rate=f.getFrameInfo().sampleRate; return true;}
  int resets=0,shutdowns=0,writes=0,volume=-1,bytes=0,rate=0;
};
class FakeMedia : public IMediaBusClient {
public:
  std::shared_ptr<FakeRaw> raw=std::make_shared<FakeRaw>();
  RawAudioFrameCallback callback;
  void shutdown() override {}
  int32_t getLastError() const override {return 0;}
  void stopRawVideoFrame(int32_t) override {}
  void stopRawAudioFrame(int32_t) override {callback=nullptr;}
  bool getMediaLayout(MediaLayout &) override {return false;}
  bool setup(std::string) override {return true;}
  IAudioRawBackStream::Ptr createAudioRawBack() override {return raw;}
  void stopEncodedVideoFrame(int32_t) override {}
  bool startRawVideoFrame(int32_t,RawVideoFrameCallback) override {return false;}
  bool startRawAudioFrame(int32_t,RawAudioFrameCallback cb) override {callback=cb;return true;}
  bool startEncodedVideoFrame(int32_t,EncodedVideoFrameCallback) override {return false;}
};
TEST(AudioBridge, RosCapturePlaybackVolumeResetAndCleanup) {
  rclcpp::init(0,nullptr);
  {
    rclcpp::NodeOptions options;
    options.parameter_overrides({rclcpp::Parameter("audio_capture",true),rclcpp::Parameter("audio_playback",true)});
    auto node=std::make_shared<rclcpp::Node>("audio_bridge_test",options);
    auto backend=std::make_shared<FakeMedia>();
    AudioBridge bridge(*node); bridge.start(backend,100);
    rclcpp::executors::SingleThreadedExecutor executor; executor.add_node(node);
    auto pump=[&](int ms) {
      const auto end=std::chrono::steady_clock::now()+std::chrono::milliseconds(ms);
      while(std::chrono::steady_clock::now()<end) {executor.spin_some();std::this_thread::sleep_for(std::chrono::milliseconds(5));}
    };
    msg::AudioFrame received; bool captured=false;
    auto sub=node->create_subscription<msg::AudioFrame>("audio/capture",rclcpp::SensorDataQoS(),
      [&](msg::AudioFrame::ConstSharedPtr f){received=*f;captured=true;});
    auto pub=node->create_publisher<msg::AudioFrame>("audio/playback",rclcpp::SensorDataQoS());
    auto reset=node->create_client<std_srvs::srv::Trigger>("audio/reset");
    pump(500);
    std::vector<uint8_t> pcm(1280,42); AudioFrame audio(pcm.data(),pcm.size());
    auto & info=audio.getAudioFrameInfo();info.sampleRate=16000;info.sampleFormat=16;
    info.channelCount=1;info.sequence=73;info.timestamp=123456;
    ASSERT_TRUE(backend->callback);backend->callback(0,audio);pump(200);
    EXPECT_TRUE(captured); EXPECT_EQ(received.sequence,73u);
    EXPECT_EQ(received.device_timestamp_us,123456u);EXPECT_EQ(received.data.size(),1280u);
    ASSERT_FALSE(received.data.empty());EXPECT_EQ(received.data[0],42u);
    pub->publish(received);pump(200);
    EXPECT_EQ(backend->raw->writes,1);EXPECT_EQ(backend->raw->bytes,1280);EXPECT_EQ(backend->raw->rate,16000);
    received.sample_rate=48000;pub->publish(received);pump(100);EXPECT_EQ(backend->raw->writes,1);
    EXPECT_TRUE(node->set_parameter(rclcpp::Parameter("audio_volume",35)).successful);
    EXPECT_EQ(backend->raw->volume,35);
    EXPECT_FALSE(node->set_parameter(rclcpp::Parameter("audio_volume",101)).successful);
    EXPECT_EQ(backend->raw->volume,35);
    ASSERT_TRUE(reset->wait_for_service(std::chrono::seconds(1)));
    auto future=reset->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>());
    ASSERT_EQ(executor.spin_until_future_complete(future,std::chrono::seconds(2)),rclcpp::FutureReturnCode::SUCCESS);
    EXPECT_TRUE(future.get()->success);EXPECT_EQ(backend->raw->resets,1);
    bridge.stop();EXPECT_FALSE(backend->callback);EXPECT_EQ(backend->raw->shutdowns,1);
  }
  rclcpp::shutdown();
}
