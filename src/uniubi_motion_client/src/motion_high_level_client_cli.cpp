#include <chrono>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <memory>
#include <poll.h>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <unistd.h>

#include "rclcpp/rclcpp.hpp"
#include "uniubi_motion_client/motion_high_level_client.hpp"

namespace
{

using namespace std::chrono_literals;
using Client = uniubi_motion_client::MotionHighLevelClient;

constexpr const char * kDefaultDomainId = "42";
constexpr const char * kDefaultService = "robotServer";
constexpr const char * kDefaultEventTopic = "/robotServer/Event";
constexpr const char * kDefaultSensorTopic = "/sensor/observed";
constexpr const char * kStopVelocity =
  R"({"lineVelocityX":0.0,"lineVelocityY":0.0,"velocity":0.0})";

std::string env_or(const char * name, const char * fallback)
{
  const char * value = std::getenv(name);
  return value != nullptr && value[0] != '\0' ? value : fallback;
}

std::string trim_left(std::string value)
{
  const auto first = value.find_first_not_of(" \t");
  return first == std::string::npos ? std::string() : value.substr(first);
}

class HighLevelCli
{
public:
  HighLevelCli(
    rclcpp::Node::SharedPtr node,
    rclcpp::Executor & executor,
    const std::string & device_id)
  : executor_(executor)
  {
    client_ = std::make_unique<Client>(
      std::move(node), executor_, env_or("UNIUBI_TEST_SERVICE_NAME", kDefaultService), device_id,
      env_or("UNIUBI_TEST_EVENT_TOPIC", kDefaultEventTopic),
      env_or("UNIUBI_TEST_SENSOR_OBSERVED_TOPIC", kDefaultSensorTopic));

    client_->setConnectCallback(
      [](Client::HighLevelState state, Client::HighLevelError error) {
        if (state == Client::kControlled) {
          std::cout << "\n[INFO] HighLevel control acquired\n";
        } else if (state == Client::kConnected && error != Client::kNone) {
          std::cout << "\n[WARN] control lost, error=" << error << "\n";
        }
      });
    client_->setEventCallback(
      [](const std::string & topic, const std::string & payload) {
        if (topic == "control.status") {
          std::cout << "\n[EVENT] " << topic << ": " << payload << "\n";
        }
      });
    client_->setSensorObservedCallback(
      [this](const uniubi::msg::SensorObserved & sensor) {
        latest_sensor_ = sensor;
        has_sensor_ = true;
        ++sensor_frames_;
      });
  }

  void connect()
  {
    if (!client_->connect()) {
      fail("connect");
    }
    if (!client_->setMotionObservedEnable(false, true)) {
      fail("enable SensorObserved");
    }
    if (!client_->startControl()) {
      fail("startControl");
    }
    std::cout << "[PASS] connected; SensorObserved enabled; HighLevel control acquired\n";
  }

  void run()
  {
    print_help();
    bool running = true;
    while (rclcpp::ok() && running) {
      std::cout << "ros2-highlevel> " << std::flush;
      std::string line;
      while (rclcpp::ok()) {
        executor_.spin_some();
        pollfd input{STDIN_FILENO, POLLIN, 0};
        if (::poll(&input, 1, 20) > 0 && (input.revents & (POLLIN | POLLHUP))) {
          if (!std::getline(std::cin, line)) {
            running = false;
          }
          break;
        }
        std::this_thread::sleep_for(10ms);
      }
      if (!running || !rclcpp::ok()) {
        break;
      }
      running = execute(trim_left(line));
    }
  }

  void close()
  {
    if (client_->getState() != Client::kDisconnected &&
      !client_->setMotionObservedEnable(false, false))
    {
      std::cerr << "[WARN] disable SensorObserved failed, error=" << client_->getLastError() << '\n';
    }
    if (client_->getState() == Client::kControlled && !client_->releaseControl()) {
      std::cerr << "[WARN] releaseControl failed, error=" << client_->getLastError() << '\n';
    }
    client_->disconnect();
  }

private:
  bool execute(const std::string & line)
  {
    if (line.empty()) {
      return true;
    }

    std::istringstream input(line);
    std::string command;
    input >> command;

    if (command == "help" || command == "?") {
      print_help();
    } else if (command == "capabilities") {
      query("capabilities", [this](std::string & out) {return client_->queryCapabilities(out);});
    } else if (command == "system") {
      query("system", [this](std::string & out) {return client_->querySystemStatus(out);});
    } else if (command == "state") {
      query("state", [this](std::string & out) {return client_->queryMotionState(out);});
    } else if (command == "motors") {
      query("motors", [this](std::string & out) {return client_->queryMotorLayout(out);});
    } else if (command == "start") {
      std::string action;
      input >> action;
      const std::string params = rest(input);
      if (action.empty()) {
        std::cout << "[FAIL] usage: start ACTION [JSON]\n";
      } else {
        result(client_->startAction(action, params), "started " + action);
      }
    } else if (command == "set") {
      const std::string params = rest(input);
      if (params.empty()) {
        std::cout << "[FAIL] usage: set JSON\n";
      } else {
        result(client_->setActionParams(params), "params set; command remains active");
      }
    } else if (command == "send") {
      double seconds = 0.0;
      input >> seconds;
      const std::string params = rest(input);
      if (seconds <= 0.0 || params.empty()) {
        std::cout << "[FAIL] usage: send SECONDS JSON\n";
      } else if (client_->setActionParams(params)) {
        spin_for(std::chrono::duration<double>(seconds));
        if (client_->setActionParams(kStopVelocity)) {
          std::cout << "[PASS] command active for " << seconds
                    << "s; velocity cleared; current action is still running\n";
        } else {
          print_failure("clear velocity");
        }
      } else {
        print_failure("set params");
      }
    } else if (command == "stop") {
      result(client_->stopAction(), "stop action requested");
    } else if (command == "estop") {
      result(client_->emergencyStop(), "emergency stop requested");
    } else if (command == "odom" || command == "sensor") {
      double seconds = command == "odom" ? 5.0 : 0.0;
      input >> seconds;
      observe(seconds, command == "odom");
    } else if (command == "quit" || command == "exit") {
      return false;
    } else {
      std::cout << "[FAIL] unknown command: " << command << " (use help)\n";
    }
    return true;
  }

  static std::string rest(std::istringstream & input)
  {
    std::string value;
    std::getline(input, value);
    return trim_left(value);
  }

  template<typename Rep, typename Period>
  void spin_for(std::chrono::duration<Rep, Period> duration)
  {
    const auto deadline = std::chrono::steady_clock::now() + duration;
    while (rclcpp::ok() && std::chrono::steady_clock::now() < deadline) {
      executor_.spin_some();
      std::this_thread::sleep_for(20ms);
    }
  }

  template<typename Query>
  void query(const std::string & name, Query call)
  {
    std::string output;
    if (call(output)) {
      std::cout << output << '\n';
    } else {
      print_failure(name);
    }
  }

  void result(bool ok, const std::string & success)
  {
    if (ok) {
      std::cout << "[PASS] " << success << '\n';
    } else {
      print_failure(success);
    }
  }

  void observe(double seconds, bool odom_only)
  {
    const auto first_frame = sensor_frames_;
    if (seconds > 0.0) {
      const auto deadline = std::chrono::steady_clock::now() + std::chrono::duration<double>(seconds);
      auto next_print = std::chrono::steady_clock::now();
      while (rclcpp::ok() && std::chrono::steady_clock::now() < deadline) {
        executor_.spin_some();
        if (has_sensor_ && std::chrono::steady_clock::now() >= next_print) {
          print_sensor(odom_only);
          next_print = std::chrono::steady_clock::now() + 200ms;
        }
        std::this_thread::sleep_for(20ms);
      }
    } else if (has_sensor_) {
      print_sensor(odom_only);
    }

    if (!has_sensor_ || (seconds > 0.0 && sensor_frames_ == first_frame)) {
      std::cout << "[WAIT] no new SensorObserved frame received\n";
    }
  }

  void print_sensor(bool odom_only) const
  {
    const auto & odom = latest_sensor_.odom;
    std::cout << std::fixed << std::setprecision(3);
    if (!odom_only) {
      std::cout << "sensor gps=" << static_cast<unsigned>(latest_sensor_.gps.valid)
                << " uwb=" << static_cast<unsigned>(latest_sensor_.uwb.valid) << ' ';
    }
    std::cout << "odom valid=" << static_cast<unsigned>(odom.valid)
              << " epoch=" << odom.epoch
              << " pos=(" << odom.position[0] << ',' << odom.position[1] << ',' << odom.position[2] << ')'
              << " yaw=" << odom.yaw
              << " vel=(" << odom.velocity[0] << ',' << odom.velocity[1] << ',' << odom.velocity[2] << ')'
              << " yawSpeed=" << odom.yaw_speed << '\n';
  }

  void print_failure(const std::string & operation)
  {
    std::cout << "[FAIL] " << operation << ", error=" << client_->getLastError() << '\n';
  }

  [[noreturn]] void fail(const std::string & operation)
  {
    const auto error = client_->getLastError();
    throw std::runtime_error(operation + " failed, error=" + std::to_string(error));
  }

  static void print_help()
  {
    std::cout <<
      "Commands:\n"
      "  capabilities                 list supported actions\n"
      "  system                       query robot system status\n"
      "  state                        query current motion state\n"
      "  motors                       query motor layout\n"
      "  start ACTION [JSON]          start an action\n"
      "  set JSON                     keep action parameters active\n"
      "  send SECONDS JSON            apply parameters, then clear walking velocity\n"
      "  stop                         stop the current RPC action\n"
      "  estop                        request emergency stop\n"
      "  odom [SECONDS]               print odometry at about 5 Hz (default 5s)\n"
      "  sensor [SECONDS]             print latest GPS/UWB/odometry observation\n"
      "  quit                         release control and exit\n";
  }

  rclcpp::Executor & executor_;
  std::unique_ptr<Client> client_;
  uniubi::msg::SensorObserved latest_sensor_{};
  bool has_sensor_ = false;
  std::uint64_t sensor_frames_ = 0;
};

}  // namespace

int main(int argc, char ** argv)
{
  const std::string domain_id = env_or("UNIUBI_TEST_ROS_DOMAIN_ID", kDefaultDomainId);
  const std::string device_id = env_or("UNIUBI_TEST_DEVICE_ID", "");
  if (device_id.empty()) {
    std::cerr << "UNIUBI_TEST_DEVICE_ID must be set\n";
    return 2;
  }

  setenv("ROS_DOMAIN_ID", domain_id.c_str(), 1);
  rclcpp::init(argc, argv);
  int status = 0;
  try {
    rclcpp::executors::SingleThreadedExecutor executor;
    auto node = std::make_shared<rclcpp::Node>("uniubi_motion_highlevel_cli");
    executor.add_node(node);
    HighLevelCli cli(node, executor, device_id);
    cli.connect();
    cli.run();
    cli.close();
  } catch (const std::exception & error) {
    std::cerr << "[FAIL] " << error.what() << '\n';
    status = 1;
  }
  rclcpp::shutdown();
  return status;
}
