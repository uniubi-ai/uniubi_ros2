#include <chrono>
#include <cstdlib>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "uniubi_interface_test/motion_high_level_client.hpp"

/**
 * Motion 高级控制客户端手动测试程序。
 *
 * 测试流程：
 * 1. 创建 ROS 2 节点，并为 robotServer 创建 MotionHighLevelClient。
 * 2. 调用 connect() 初始化客户端并订阅 robotServer 事件，此时还不会获取控制权。
 * 3. 先执行只读查询接口，用于在取控制权前确认基础 RPC 链路是否正常。
 * 4. 调用 startControl() 获取高级运动控制权；MotionHighLevelClient 内部会启动租约续约。
 * 5. 依次测试动作、原始 TRC、音频和运动数据记录接口。
 * 6. 测试结束时调用 releaseControl() 和 disconnect() 收尾。
 *
 * 该程序可能控制真实机器人，因此没有注册为 ament 自动测试。
 * 运行时参数可通过以下环境变量覆盖：
 * - UNIUBI_TEST_ROS_DOMAIN_ID
 * - UNIUBI_TEST_SERVICE_NAME
 * - UNIUBI_TEST_EVENT_TOPIC
 * - UNIUBI_TEST_DEVICE_ID（必须显式配置；示例不内置设备 SN）
 */
namespace
{

using namespace std::chrono_literals;
using MotionHighLevelClient = uniubi_interface_test::MotionHighLevelClient;
using HLState = MotionHighLevelClient::HighLevelState;
using HLError = MotionHighLevelClient::HighLevelError;

constexpr const char * kRosDomainId = "42";
constexpr const char * kRosServiceName = "robotServer";
constexpr const char * kRosEventTopic = "/robotServer/Event";
constexpr int32_t kControlTimeoutMs = 10000;
constexpr bool kEnableMotionActionDemo = false;

std::string get_env_or_default(const char * name, const char * default_value)
{
  const auto * value = std::getenv(name);
  if (value == nullptr || value[0] == '\0') {
    return default_value;
  }
  return value;
}

void require_true(bool condition, const std::string & message)
{
  if (!condition) {
    throw std::runtime_error(message);
  }
}

class MotionHighLevelClientTestNode : public rclcpp::Node
{
public:
  MotionHighLevelClientTestNode()
  : rclcpp::Node("uniubi_motion_highlevel_client_test"),
    service_name_(get_env_or_default("UNIUBI_TEST_SERVICE_NAME", kRosServiceName)),
    event_topic_(get_env_or_default("UNIUBI_TEST_EVENT_TOPIC", kRosEventTopic)),
    device_id_(get_env_or_default("UNIUBI_TEST_DEVICE_ID", ""))
  {
  }

  void initialize(rclcpp::Executor & executor)
  {
    // MotionHighLevelClient 需要节点的 shared_ptr，因此必须等 main() 中用 shared_ptr 持有本节点后再初始化。
    executor_ = &executor;
    client_ = std::make_unique<MotionHighLevelClient>(
      shared_from_this(),
      executor,
      service_name_,
      device_id_,
      event_topic_);
    client_->setConnectCallback(
      [this](HLState state, HLError error) {
        on_connect(state, error);
      });
    client_->setEventCallback(
      [this](const std::string & topic, const std::string & payload) {
        on_event(topic, payload);
      });
  }

  void run(const std::string & domain_id)
  {
    require_true(client_ != nullptr, "MotionHighLevelClientTestNode is not initialized");
    require_true(!device_id_.empty(), "UNIUBI_TEST_DEVICE_ID must be set explicitly");

    std::cout << "ROS_DOMAIN_ID=" << domain_id
              << ", service=" << service_name_
              << ", event_topic=" << event_topic_
              << ", device_id=" << device_id_
              << ", client_id=" << client_->client_id()
              << std::endl;

    // connect() 只创建 ROS 服务客户端和事件订阅状态，不会获取机器人控制权。
    // 控制权需要通过 startControl() 显式申请。
    require_true(client_->connect(), "connect failed: err=" + std::to_string(client_->getLastError()));
    spin_for(500ms);

    // connect() 后即可调用只读 RPC；先查询能力和系统状态，便于在取控制权前确认 robotServer 链路。
    std::string output;
    print_query_result("capabilities", client_->queryCapabilities(output), output);
    spin_for(200ms);

    output.clear();
    print_query_result("system", client_->querySystemStatus(output), output);
    spin_for(200ms);

    require_true(
      client_->startControl(kControlTimeoutMs),
      "startControl failed: err=" + std::to_string(client_->getLastError()));
    wait_until_controlled();

    if (kEnableMotionActionDemo) {
      // 真实运动动作默认关闭；确认目标设备、调试场地和人工接管条件后，再打开该开关做动作调试。
      // 按 SDK 示例，startAction 不带速度参数时只切入动作姿态，不应视为连续速度控制命令。
      if (!client_->startAction("walking", "")) {
        std::cout << "startAction skipped/failed: err=" << client_->getLastError() << std::endl;
      }
      spin_for(5s);

      if (!client_->stopAction()) {
        std::cout << "stopAction failed: err=" << client_->getLastError() << std::endl;
      }
      spin_for(200ms);

      // 原始 TRC 控制使用 takeMotionControl() 返回的 rawActionId，而不是字符串 controller token。
      // 如果服务端没有下发 rawActionId，MotionHighLevelClient 会返回 kActionRejected。
      MotionHighLevelClient::TRCStickFrame frame;
      frame.valid = 1;
      frame.buttons[MotionHighLevelClient::buttonBack] = 1;  // Stand
      frame.buttons[MotionHighLevelClient::buttonA] = 1;  // Stand + A = Lie Down（内部动作 laying）
      if (!client_->setRawControlCmd(frame)) {
        std::cout << "setRawControlCmd skipped/failed: err=" << client_->getLastError() << std::endl;
      }
      spin_for(200ms);
    } else {
      std::cout << "motion action demo skipped; set kEnableMotionActionDemo=true for walking/TRC debug"
                << std::endl;
    }

    // 这里按 SDK 示例固定播放音频 id "1"，前提是目标设备上存在该音频。
    // 如果后续 queryAudioPlayList() 返回 customVoice 为空，该调用通常会以 kActionRejected 失败。
    if (!client_->startAudioPlay(R"({"list":[{"id":"1"}],"volume":50,"repeat":1})")) {
      std::cout << "startAudioPlay failed: err=" << client_->getLastError() << std::endl;
    }
    spin_for(2s);

    if (!client_->pauseAudioPlay()) {
      std::cout << "pauseAudioPlay failed: err=" << client_->getLastError() << std::endl;
    }
    spin_for(1s);

    if (!client_->stopAudioPlay()) {
      std::cout << "stopAudioPlay failed: err=" << client_->getLastError() << std::endl;
    }
    spin_for(200ms);

    output.clear();
    print_query_result("audio detail", client_->queryAudioPlayDetail(output), output);

    output.clear();
    print_query_result(
      "audio list",
      client_->queryAudioPlayList(output, R"({"type":"customVoice"})"),
      output);

    // setMotionObservedEnable 只切换服务端运控观测量推送行为。
    // 事件回调和租约续约回调仍然依赖 executor 持续 spin。
    if (!client_->setMotionObservedEnable(true)) {
      std::cout << "setMotionObservedEnable(true) failed: err=" << client_->getLastError() << std::endl;
    }
    spin_for(2s);

    if (!client_->setMotionObservedEnable(false)) {
      std::cout << "setMotionObservedEnable(false) failed: err=" << client_->getLastError() << std::endl;
    }
    spin_for(200ms);

    if (!client_->releaseControl()) {
      std::cout << "releaseControl failed: err=" << client_->getLastError() << std::endl;
    }
    client_->disconnect();

    std::cout << "uniubi ROS 2 MotionHighLevelClient high-level test finished" << std::endl;
  }

private:
  void spin_for(std::chrono::nanoseconds duration)
  {
    // MotionHighLevelClient 内部使用 ROS timer、服务响应回调和订阅回调。
    // 测试程序在同步 SDK 风格接口之间等待时，需要持续 spin executor，
    // 这样租约续约和事件处理才能正常执行。
    const auto deadline = std::chrono::steady_clock::now() + duration;
    while (rclcpp::ok() && std::chrono::steady_clock::now() < deadline) {
      executor_->spin_some();
      std::this_thread::sleep_for(20ms);
    }
  }

  void wait_until_controlled()
  {
    // 当前 ROS 封装通常会在 startControl() 返回前同步进入 kControlled。
    // 这里保留等待循环，是为了兼容 SDK 风格的异步状态切换，并给事件回调执行机会。
    const auto deadline = std::chrono::steady_clock::now() + 10s;
    while (rclcpp::ok() && client_->getState() != MotionHighLevelClient::kControlled) {
      if (std::chrono::steady_clock::now() >= deadline) {
        throw std::runtime_error("wait controlled timeout");
      }
      spin_for(50ms);
    }
  }

  void print_query_result(
    const std::string & title,
    bool ok,
    const std::string & output)
  {
    if (ok) {
      std::cout << title << ": " << output << std::endl;
    } else {
      std::cout << title << " failed: err=" << client_->getLastError() << std::endl;
    }
  }

  void on_connect(HLState state, HLError error)
  {
    // 控制状态回调保持和 SDK 高级客户端一致：
    // kControlled 表示已获取控制权；kConnected 携带错误码表示控制权释放、拒绝、超时或被抢占。
    switch (state) {
      case MotionHighLevelClient::kControlled:
        std::cout << "[high] control acquired" << std::endl;
        break;
      case MotionHighLevelClient::kConnected:
        if (error == MotionHighLevelClient::kSessionExpired) {
          std::cout << "[high] lease expired" << std::endl;
        } else if (error == MotionHighLevelClient::kSessionRevoked) {
          std::cout << "[high] preempted by others" << std::endl;
        } else if (error == MotionHighLevelClient::kRpcAcquireRejected) {
          std::cout << "[high] startControl rejected/timeout" << std::endl;
        } else {
          std::cout << "[high] control released" << std::endl;
        }
        break;
      default:
        break;
    }
  }

  void on_event(const std::string & topic, const std::string & payload)
  {
    // MotionHighLevelClient 会解包 robotServer.host.event，并把内层业务 topic 传到这里。
    if (topic == "statistics/play_list") {
      std::cout << "[evt] play: " << payload << std::endl;
    } else if (topic == "statistics/device_status") {
      std::cout << "[evt] dev:  " << payload << std::endl;
    } else {
      std::cout << "[evt] " << topic << ": " << payload << std::endl;
    }
  }

  rclcpp::Executor * executor_ = nullptr;
  std::unique_ptr<MotionHighLevelClient> client_;
  std::string service_name_;
  std::string event_topic_;
  std::string device_id_;
};

}  // namespace

int main(int argc, char ** argv)
{
  const auto domain_id = get_env_or_default("UNIUBI_TEST_ROS_DOMAIN_ID", kRosDomainId);
  // 使用默认 context 时，ROS_DOMAIN_ID 必须在 rclcpp::init() 之前设置。
  setenv("ROS_DOMAIN_ID", domain_id.c_str(), 1);
  rclcpp::init(argc, argv);

  try {
    rclcpp::executors::SingleThreadedExecutor executor;
    auto node = std::make_shared<MotionHighLevelClientTestNode>();
    executor.add_node(node);
    node->initialize(executor);
    node->run(domain_id);
  } catch (const std::exception & e) {
    std::cerr << "uniubi ROS 2 MotionHighLevelClient high-level test failed: " << e.what() << std::endl;
    rclcpp::shutdown();
    return 1;
  }

  rclcpp::shutdown();
  return 0;
}
