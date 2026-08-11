#include <atomic>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <csignal>
#include <cstdint>
#include <deque>
#include <future>
#include <functional>
#include <iomanip>
#include <initializer_list>
#include <limits>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <sstream>
#include <string>
#include <thread>
#include <tuple>
#include <utility>
#include <vector>

#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/battery_state.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "uniubi_motion_bridge/msg/motion_status.hpp"
#include "uniubi_motion_bridge/srv/start_motion_action.hpp"
#include "uniubi_motion_client/motion_high_level_client.hpp"

namespace uniubi_motion_bridge
{

namespace
{

using MotionClient = uniubi_motion_client::MotionHighLevelClient;

constexpr auto kConnectReadinessTimeout = std::chrono::seconds(5);
constexpr auto kConnectReadinessRetryInterval = std::chrono::milliseconds(200);
constexpr int32_t kConnectReadinessRpcTimeoutMs = 500;

volatile std::sig_atomic_t shutdown_requested = 0;

void handle_signal(int)
{
  shutdown_requested = 1;
}

const char * state_name(int32_t state)
{
  switch (state) {
    case MotionClient::kConnected:
      return "connected";
    case MotionClient::kControlled:
      return "controlled";
    case MotionClient::kDisconnected:
    default:
      return "disconnected";
  }
}

}  // namespace

struct CommandResult
{
  bool success{false};
  int32_t error_code{MotionClient::kNone};
  std::string message;
};

struct VelocityCommand
{
  double linear_x{0.0};
  double linear_y{0.0};
  double angular_z{0.0};
  std::chrono::steady_clock::time_point received_at;
};

struct MotorKey
{
  std::uint32_t limb_no{0};
  std::uint32_t joint_no{0};

  bool operator<(const MotorKey & other) const
  {
    return std::tie(limb_no, joint_no) < std::tie(other.limb_no, other.joint_no);
  }
};

struct MotorLayoutEntry
{
  MotorKey key;
  std::string name;
};

class MotionBridgeNode final : public rclcpp::Node
{
public:
  MotionBridgeNode()
  : Node("uniubi_motion_bridge")
  {
    robot_service_name_ = declare_parameter<std::string>("robot_service_name", "robotServer");
    event_topic_ = declare_parameter<std::string>("event_topic", "/robotServer/Event");
    device_id_ = declare_parameter<std::string>("device_id", "");
    lease_ms_ = declare_parameter<int32_t>("lease_ms", 60000);
    auto_connect_ = declare_parameter<bool>("auto_connect", true);
    cmd_vel_topic_ = declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel");
    cmd_vel_timeout_ms_ = declare_parameter<int32_t>("cmd_vel_timeout_ms", 500);
    cmd_vel_rate_hz_ = declare_parameter<double>("cmd_vel_rate_hz", 50.0);
    odom_topic_ = declare_parameter<std::string>("odom_topic", "/odom");
    odom_frame_id_ = declare_parameter<std::string>("odom_frame_id", "odom");
    base_frame_id_ = declare_parameter<std::string>("base_frame_id", "base_link");
    publish_joint_states_ = declare_parameter<bool>("publish_joint_states", true);
    joint_states_topic_ = declare_parameter<std::string>("joint_states_topic", "/joint_states");
    fallback_limb_sizes_ = declare_parameter<std::vector<int64_t>>(
      "fallback_limb_sizes", std::vector<int64_t>{});
    fallback_joint_names_ = declare_parameter<std::vector<std::string>>(
      "fallback_joint_names", std::vector<std::string>{});
    imu_topic_ = declare_parameter<std::string>("imu_topic", "/imu/data");
    imu_frame_id_ = declare_parameter<std::string>("imu_frame_id", "imu_link");
    battery_topic_ = declare_parameter<std::string>("battery_topic", "/battery_state");
    battery_publish_rate_hz_ = declare_parameter<double>("battery_publish_rate_hz", 1.0);
    motion_status_topic_ = declare_parameter<std::string>(
      "motion_status_topic", "/motion/status");
    motion_status_rate_hz_ = declare_parameter<double>("motion_status_rate_hz", 10.0);

    if (cmd_vel_timeout_ms_ <= 0) {
      cmd_vel_timeout_ms_ = 500;
    }
    if (!std::isfinite(cmd_vel_rate_hz_) || cmd_vel_rate_hz_ <= 0.0) {
      cmd_vel_rate_hz_ = 50.0;
    }
    if (!std::isfinite(battery_publish_rate_hz_) || battery_publish_rate_hz_ <= 0.0) {
      battery_publish_rate_hz_ = 1.0;
    }
    if (!std::isfinite(motion_status_rate_hz_) || motion_status_rate_hz_ <= 0.0) {
      motion_status_rate_hz_ = 10.0;
    }
    battery_publish_period_ = std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::duration<double>(1.0 / battery_publish_rate_hz_));
    if (battery_publish_period_ < std::chrono::milliseconds(1)) {
      battery_publish_period_ = std::chrono::milliseconds(1);
    }
    motion_status_publish_period_ = std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::duration<double>(1.0 / motion_status_rate_hz_));
    if (motion_status_publish_period_ < std::chrono::milliseconds(1)) {
      motion_status_publish_period_ = std::chrono::milliseconds(1);
    }
    velocity_dispatch_period_ = std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::duration<double>(1.0 / cmd_vel_rate_hz_));
    if (velocity_dispatch_period_ < std::chrono::milliseconds(1)) {
      velocity_dispatch_period_ = std::chrono::milliseconds(1);
    }

    motion_status_publisher_ = create_publisher<uniubi_motion_bridge::msg::MotionStatus>(
      motion_status_topic_, rclcpp::QoS(1).reliable().transient_local());
    odometry_publisher_ = create_publisher<nav_msgs::msg::Odometry>(
      odom_topic_, rclcpp::QoS(1).best_effort().durability_volatile());
    joint_state_publisher_ = create_publisher<sensor_msgs::msg::JointState>(
      joint_states_topic_, rclcpp::QoS(1).best_effort().durability_volatile());
    imu_publisher_ = create_publisher<sensor_msgs::msg::Imu>(
      imu_topic_, rclcpp::SensorDataQoS());
    battery_publisher_ = create_publisher<sensor_msgs::msg::BatteryState>(
      battery_topic_, rclcpp::SensorDataQoS());
    cmd_vel_subscription_ = create_subscription<geometry_msgs::msg::Twist>(
      cmd_vel_topic_, rclcpp::QoS(1).best_effort().durability_volatile(),
      [this](const geometry_msgs::msg::Twist::SharedPtr message) {
        on_cmd_vel(*message);
      });

    release_control_service_ = create_service<std_srvs::srv::Trigger>(
      "/motion/release_control",
      [this](const std_srvs::srv::Trigger::Request::SharedPtr,
      std_srvs::srv::Trigger::Response::SharedPtr response) {
        set_response(response, submit([this]() {return release_control();}));
      });
    query_capabilities_service_ = create_service<std_srvs::srv::Trigger>(
      "/motion/query_capabilities",
      [this](const std_srvs::srv::Trigger::Request::SharedPtr,
      std_srvs::srv::Trigger::Response::SharedPtr response) {
        set_response(response, submit([this]() {return query_capabilities();}));
      });
    stop_action_service_ = create_service<std_srvs::srv::Trigger>(
      "/motion/stop_action",
      [this](const std_srvs::srv::Trigger::Request::SharedPtr,
      std_srvs::srv::Trigger::Response::SharedPtr response) {
        set_response(response, submit([this]() {return stop_action("service request");}));
      });
    emergency_stop_service_ = create_service<std_srvs::srv::Trigger>(
      "/motion/emergency_stop",
      [this](const std_srvs::srv::Trigger::Request::SharedPtr,
      std_srvs::srv::Trigger::Response::SharedPtr response) {
        set_response(response, submit([this]() {return emergency_stop();}));
      });
    start_action_service_ = create_service<uniubi_motion_bridge::srv::StartMotionAction>(
      "/motion/start_action",
      [this](const uniubi_motion_bridge::srv::StartMotionAction::Request::SharedPtr request,
      uniubi_motion_bridge::srv::StartMotionAction::Response::SharedPtr response) {
        const auto result = submit(
          [this, action = request->action, params = request->params_json]() {
            return start_action(action, params);
          });
        response->success = result.success;
        response->error_code = result.error_code;
        response->message = result.message;
      });
    client_node_ = std::make_shared<rclcpp::Node>("uniubi_motion_bridge_client");
    client_executor_ = std::make_unique<rclcpp::executors::SingleThreadedExecutor>();
    client_executor_->add_node(client_node_);
    motion_client_ = std::make_unique<MotionClient>(
      client_node_, *client_executor_, robot_service_name_, device_id_, event_topic_);
    motion_client_->setConnectCallback(
      [this](MotionClient::HighLevelState state, MotionClient::HighLevelError error) {
        if (state == MotionClient::kDisconnected) {
          current_action_.clear();
          current_linear_x_ = current_linear_y_ = current_angular_z_ = 0.0F;
          reset_velocity_tracking();
        }
        if (error != MotionClient::kNone) {
          set_status_error(error, "motion control state changed with an error");
        }
        publish_motion_status();
      });
    motion_client_->setEventCallback(
      [this](const std::string & topic, const std::string & payload_json) {
        RCLCPP_DEBUG(
          get_logger(), "robotServer event [%s]: %s", topic.c_str(), payload_json.c_str());
      });
    motion_client_->setSensorObservedCallback(
      [this](const uniubi::msg::SensorObserved & sensor) {
        publish_odometry(sensor.odom);
      });
    motion_client_->setMotionObservedCallback(
      [this](const uniubi::msg::MotionObserved & observed) {
        publish_motion_observed(observed);
      });

    worker_ = std::thread([this]() {worker_loop();});
    publish_motion_status();

    if (device_id_.empty()) {
      RCLCPP_WARN(
        get_logger(),
        "device_id is empty; set it explicitly when more than one robot shares the DDS domain");
    }
    if (auto_connect_) {
      enqueue([this]() {return connect_client();});
    }
  }

  ~MotionBridgeNode() override
  {
    stop_.store(true);
    queue_cv_.notify_all();
    if (worker_.joinable()) {
      worker_.join();
    }
    if (client_executor_ && client_node_) {
      client_executor_->remove_node(client_node_);
    }
  }

private:
  using TriggerResponse = std_srvs::srv::Trigger::Response;
  using Command = std::function<CommandResult()>;

  struct QueuedCommand
  {
    Command command;
    std::shared_ptr<std::promise<CommandResult>> promise;
  };

  void worker_loop()
  {
    while (!stop_.load()) {
      QueuedCommand queued;
      bool has_command = false;
      {
        std::unique_lock<std::mutex> lock(queue_mutex_);
        queue_cv_.wait_for(
          lock, std::chrono::milliseconds(20),
          [this]() {return stop_.load() || !commands_.empty();});
        if (!commands_.empty()) {
          queued = std::move(commands_.front());
          commands_.pop_front();
          has_command = true;
        }
      }

      if (has_command) {
        try {
          queued.promise->set_value(queued.command());
        } catch (const std::exception & error) {
          queued.promise->set_value({false, MotionClient::kRpcCallFailed, error.what()});
        }
      } else {
        dispatch_pending_velocity();
      }
      enforce_cmd_vel_watchdog();
      client_executor_->spin_some();
      if (rpc_ready_) {
        update_motion_status();
      }
    }

    if (motion_client_) {
      if (motion_client_->getState() == MotionClient::kControlled && !current_action_.empty()) {
        (void)stop_action("node shutdown");
      }
      disable_motion_observed();
      motion_client_->disconnect();
      rpc_ready_ = false;
      current_action_.clear();
      current_linear_x_ = current_linear_y_ = current_angular_z_ = 0.0F;
      publish_motion_status();
    }
  }

  std::future<CommandResult> enqueue(Command command)
  {
    auto promise = std::make_shared<std::promise<CommandResult>>();
    auto future = promise->get_future();
    {
      std::lock_guard<std::mutex> lock(queue_mutex_);
      commands_.push_back({std::move(command), promise});
    }
    queue_cv_.notify_one();
    return future;
  }

  CommandResult submit(Command command)
  {
    return enqueue(std::move(command)).get();
  }

  CommandResult connect_client()
  {
    if (motion_client_->getState() != MotionClient::kDisconnected && rpc_ready_) {
      return {true, MotionClient::kNone, state_name(motion_client_->getState())};
    }

    if (motion_client_->getState() == MotionClient::kDisconnected) {
      rpc_ready_ = false;
      if (!motion_client_->connect(lease_ms_)) {
        const auto error = motion_client_->getLastError();
        set_status_error(error, "connect failed");
        publish_motion_status();
        return {false, error, "connect failed"};
      }
    }

    if (!wait_for_rpc_ready()) {
      auto error = motion_client_->getLastError();
      if (error == MotionClient::kNone) {
        error = MotionClient::kRpcConnectFailed;
      }
      set_status_error(error, "connect readiness check timed out");
      publish_motion_status();
      return {false, error, "connect readiness check timed out"};
    }

    rpc_ready_ = true;
    configure_joint_states();
    clear_status_error();
    next_motion_status_query_at_ = {};
    publish_motion_status();
    return {true, MotionClient::kNone, "connected"};
  }

  bool wait_for_rpc_ready()
  {
    const auto deadline = std::chrono::steady_clock::now() + kConnectReadinessTimeout;
    while (std::chrono::steady_clock::now() < deadline) {
      const auto remaining = std::chrono::duration_cast<std::chrono::milliseconds>(
        deadline - std::chrono::steady_clock::now());
      const auto rpc_timeout_ms = std::max<int32_t>(
        1, std::min<int32_t>(
          kConnectReadinessRpcTimeoutMs, static_cast<int32_t>(remaining.count())));

      std::string ignored;
      if (motion_client_->queryCapabilities(ignored, rpc_timeout_ms)) {
        return true;
      }

      const auto now = std::chrono::steady_clock::now();
      if (now >= deadline) {
        break;
      }
      std::this_thread::sleep_for(std::min(
        kConnectReadinessRetryInterval,
        std::chrono::duration_cast<std::chrono::milliseconds>(deadline - now)));
    }
    return false;
  }

  CommandResult acquire_control(bool & acquired_now)
  {
    acquired_now = false;
    const auto connected = connect_client();
    if (!connected.success) {
      return connected;
    }
    if (motion_client_->getState() == MotionClient::kControlled) {
      return {true, MotionClient::kNone, "already controlled"};
    }
    const bool success = motion_client_->startControl();
    const auto error = success ? MotionClient::kNone : motion_client_->getLastError();
    acquired_now = success;
    if (success) {
      clear_status_error();
    } else {
      set_status_error(error, "control acquire failed");
    }
    publish_motion_status();
    return {success, error, success ? "control acquired" : "control acquire failed"};
  }

  CommandResult release_control()
  {
    if (motion_client_->getState() == MotionClient::kDisconnected) {
      publish_motion_status();
      return {true, MotionClient::kNone, "already disconnected"};
    }
    if (motion_client_->getState() != MotionClient::kControlled) {
      publish_motion_status();
      return {true, MotionClient::kNone, "control already released"};
    }
    if (!current_action_.empty()) {
      const auto stopped = stop_action("release control");
      if (!stopped.success) {
        return {
          false, stopped.error_code,
          "control release aborted because the active action could not be stopped"};
      }
    }
    const bool success = motion_client_->releaseControl();
    const auto error = success ? MotionClient::kNone : motion_client_->getLastError();
    if (success) {
      current_action_.clear();
      current_linear_x_ = current_linear_y_ = current_angular_z_ = 0.0F;
      reset_velocity_tracking();
      clear_status_error();
    } else {
      set_status_error(error, "control release failed");
    }
    publish_motion_status();
    return {success, error, success ? "control released" : "control release failed"};
  }

  CommandResult stop_action(const std::string & reason)
  {
    if (motion_client_->getState() != MotionClient::kControlled) {
      return {false, MotionClient::kNotControlled, "control is not acquired"};
    }
    const bool success = motion_client_->stopAction();
    const auto error = success ? MotionClient::kNone : motion_client_->getLastError();
    if (success) {
      reset_velocity_tracking();
      clear_status_error();
      next_motion_status_query_at_ = {};
    } else {
      set_status_error(error, "stop action failed");
    }
    const CommandResult result{
      success, error, success ? "action stopped: " + reason : "stop action failed"};
    publish_motion_status();
    return result;
  }

  CommandResult emergency_stop()
  {
    if (motion_client_->getState() != MotionClient::kControlled) {
      return {false, MotionClient::kNotControlled, "control is not acquired"};
    }
    const bool success = motion_client_->emergencyStop();
    const auto error = success ? MotionClient::kNone : motion_client_->getLastError();
    if (success) {
      reset_velocity_tracking();
      clear_status_error();
      next_motion_status_query_at_ = {};
    } else {
      set_status_error(error, "emergency stop failed");
    }
    const CommandResult result{
      success, error, success ? "emergency stop sent" : "emergency stop failed"};
    publish_motion_status();
    return result;
  }

  CommandResult start_action(const std::string & action, const std::string & params_json)
  {
    if (action.empty()) {
      return {false, MotionClient::kActionRejected, "action must not be empty"};
    }

    const auto connected = connect_client();
    if (!connected.success) {
      return connected;
    }
    bool acquired_now = false;
    const auto acquired = acquire_control(acquired_now);
    if (!acquired.success) {
      return acquired;
    }

    const bool success = motion_client_->startAction(action, params_json);
    const auto error = success ? MotionClient::kNone : motion_client_->getLastError();
    if (success) {
      current_action_ = action;
      reset_velocity_tracking();
      clear_status_error();
      next_motion_status_query_at_ = {};
    } else {
      set_status_error(error, action + " failed");
    }
    bool rollback_released = true;
    if (!success && acquired_now) {
      rollback_released = motion_client_->releaseControl();
      if (!rollback_released) {
        (void)motion_client_->getLastError();
      }
      publish_motion_status();
    }
    const CommandResult result{
      success, error,
      success ? action + " started" :
      action + " failed" +
      (acquired_now ?
      (rollback_released ? "; newly acquired control was released" :
      "; newly acquired control could not be released") : "")};
    publish_motion_status();
    return result;
  }

  bool fetch_capabilities(std::string & capabilities)
  {
    return motion_client_->queryCapabilities(capabilities);
  }

  CommandResult query_capabilities()
  {
    const auto connected = connect_client();
    if (!connected.success) {
      return connected;
    }
    std::string capabilities;
    const bool success = fetch_capabilities(capabilities);
    const auto error = success ? MotionClient::kNone : motion_client_->getLastError();
    return {success, error, success ? capabilities : "capability query failed"};
  }

  void set_status_error(int32_t error_code, const std::string & message, bool from_query = false)
  {
    last_error_code_ = error_code;
    last_error_message_ = message;
    last_error_from_query_ = from_query;
  }

  void clear_status_error()
  {
    last_error_code_ = MotionClient::kNone;
    last_error_message_.clear();
    last_error_from_query_ = false;
  }

  void publish_motion_status()
  {
    uniubi_motion_bridge::msg::MotionStatus message;
    message.stamp = now();
    if (!rpc_ready_) {
      message.control_state = uniubi_motion_bridge::msg::MotionStatus::DISCONNECTED;
    } else {
      switch (motion_client_->getState()) {
        case MotionClient::kControlled:
          message.control_state = uniubi_motion_bridge::msg::MotionStatus::CONTROLLED;
          break;
        case MotionClient::kConnected:
          message.control_state = uniubi_motion_bridge::msg::MotionStatus::CONNECTED;
          break;
        case MotionClient::kDisconnected:
        default:
          message.control_state = uniubi_motion_bridge::msg::MotionStatus::DISCONNECTED;
          break;
      }
    }
    message.current_action = current_action_;
    message.line_velocity_x = current_linear_x_;
    // Both ROS REP-103 and the UniUbi motion interface define +Y as left.
    message.line_velocity_y = current_linear_y_;
    message.angular_velocity = current_angular_z_;
    message.last_error_code = last_error_code_;
    message.last_error_message = last_error_message_;
    motion_status_publisher_->publish(message);
  }

  void update_motion_status()
  {
    const auto now_steady = std::chrono::steady_clock::now();
    if (now_steady < next_motion_status_query_at_) {
      return;
    }
    next_motion_status_query_at_ = now_steady + motion_status_publish_period_;

    if (!rpc_ready_ || motion_client_->getState() == MotionClient::kDisconnected) {
      return;
    }

    std::string state_json;
    if (!motion_client_->queryMotionState(state_json, 100)) {
      set_status_error(
        motion_client_->getLastError(), "queryMotionState failed", true);
      publish_motion_status();
      return;
    }

    Json::CharReaderBuilder builder;
    Json::Value root;
    std::string parse_error;
    std::istringstream stream(state_json);
    if (!Json::parseFromStream(builder, stream, &root, &parse_error) || !root.isObject()) {
      set_status_error(
        MotionClient::kRpcCallFailed, "queryMotionState returned invalid JSON", true);
      publish_motion_status();
      return;
    }

    current_action_ = root.isMember("action") && root["action"].isString() ?
      root["action"].asString() : std::string();
    current_linear_x_ = root.isMember("lineVelocityX") && root["lineVelocityX"].isNumeric() ?
      root["lineVelocityX"].asFloat() : 0.0F;
    current_linear_y_ = root.isMember("lineVelocityY") && root["lineVelocityY"].isNumeric() ?
      root["lineVelocityY"].asFloat() : 0.0F;
    current_angular_z_ = root.isMember("velocity") && root["velocity"].isNumeric() ?
      root["velocity"].asFloat() : 0.0F;
    if (last_error_from_query_) {
      clear_status_error();
    }
    publish_motion_status();
  }

  void on_cmd_vel(const geometry_msgs::msg::Twist & message)
  {
    const auto x = message.linear.x;
    const auto y = message.linear.y;
    const auto yaw = message.angular.z;
    if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(yaw)) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "ignoring cmd_vel containing NaN or infinity");
      return;
    }

    VelocityCommand command;
    command.linear_x = x;
    // Both ROS REP-103 and the UniUbi motion interface define +Y as left.
    command.linear_y = y;
    command.angular_z = yaw;
    command.received_at = std::chrono::steady_clock::now();
    {
      std::lock_guard<std::mutex> lock(velocity_mutex_);
      pending_velocity_ = command;
      last_velocity_received_at_ = command.received_at;
    }
    queue_cv_.notify_one();
  }

  void dispatch_pending_velocity()
  {
    const auto now = std::chrono::steady_clock::now();
    std::optional<VelocityCommand> command;
    {
      std::lock_guard<std::mutex> lock(velocity_mutex_);
      if (!pending_velocity_.has_value() || now < next_velocity_dispatch_at_) {
        return;
      }
      command = pending_velocity_;
      pending_velocity_.reset();
      next_velocity_dispatch_at_ = now + velocity_dispatch_period_;
    }
    if (now - command->received_at > std::chrono::milliseconds(cmd_vel_timeout_ms_)) {
      return;
    }
    velocity_watchdog_fired_ = false;
    apply_velocity(*command);
  }

  void apply_velocity(const VelocityCommand & command)
  {
    constexpr double kZeroEpsilon = 1.0e-6;
    Json::Value params(Json::objectValue);
    params["lineVelocityX"] = command.linear_x;
    // No lateral sign conversion is required at the ROS/SDK boundary.
    params["lineVelocityY"] = command.linear_y;
    params["velocity"] = command.angular_z;
    const bool any_nonzero = std::abs(command.linear_x) > kZeroEpsilon ||
      std::abs(command.linear_y) > kZeroEpsilon ||
      std::abs(command.angular_z) > kZeroEpsilon;

    Json::StreamWriterBuilder builder;
    builder["indentation"] = "";
    const bool success = motion_client_->setActionParams(Json::writeString(builder, params));
    const auto error = success ? MotionClient::kNone : motion_client_->getLastError();
    if (success) {
      velocity_command_active_ = any_nonzero;
      clear_status_error();
    } else {
      set_status_error(error, "cmd_vel rejected");
    }
    publish_motion_status();
  }

  void enforce_cmd_vel_watchdog()
  {
    if (!velocity_command_active_ || velocity_watchdog_fired_ ||
      motion_client_->getState() != MotionClient::kControlled)
    {
      return;
    }
    std::chrono::steady_clock::time_point last_received;
    {
      std::lock_guard<std::mutex> lock(velocity_mutex_);
      last_received = last_velocity_received_at_;
    }
    if (last_received.time_since_epoch().count() == 0) {
      return;
    }
    const auto elapsed = std::chrono::steady_clock::now() - last_received;
    if (elapsed > std::chrono::milliseconds(cmd_vel_timeout_ms_)) {
      VelocityCommand zero;
      zero.received_at = std::chrono::steady_clock::now();
      apply_velocity(zero);
      velocity_watchdog_fired_ = true;
    }
  }

  void reset_velocity_tracking()
  {
    {
      std::lock_guard<std::mutex> lock(velocity_mutex_);
      pending_velocity_.reset();
      last_velocity_received_at_ = {};
    }
    next_velocity_dispatch_at_ = {};
    velocity_command_active_ = false;
    velocity_watchdog_fired_ = false;
  }

  void publish_odometry(const uniubi::msg::MotionOdometry & input)
  {
    if (!input.valid) {
      return;
    }
    nav_msgs::msg::Odometry output;
    output.header.stamp = now();
    output.header.frame_id = odom_frame_id_;
    output.child_frame_id = base_frame_id_;
    output.pose.pose.position.x = input.position[0];
    output.pose.pose.position.y = input.position[1];
    output.pose.pose.position.z = input.position[2];
    output.pose.pose.orientation.z = std::sin(static_cast<double>(input.yaw) * 0.5);
    output.pose.pose.orientation.w = std::cos(static_cast<double>(input.yaw) * 0.5);
    output.twist.twist.linear.x = input.velocity[0];
    output.twist.twist.linear.y = input.velocity[1];
    output.twist.twist.linear.z = input.velocity[2];
    output.twist.twist.angular.z = input.yaw_speed;
    output.pose.covariance[0] = -1.0;
    output.twist.covariance[0] = -1.0;
    odometry_publisher_->publish(output);
  }

  static bool read_uint_field(
    const Json::Value & object,
    std::initializer_list<const char *> names,
    std::uint32_t & output)
  {
    for (const auto * name : names) {
      if (object.isMember(name) && object[name].isUInt()) {
        output = object[name].asUInt();
        return true;
      }
    }
    return false;
  }

  static std::string read_string_field(
    const Json::Value & object,
    std::initializer_list<const char *> names)
  {
    for (const auto * name : names) {
      if (object.isMember(name) && object[name].isString()) {
        return object[name].asString();
      }
    }
    return {};
  }

  bool parse_motor_layout(const std::string & layout_json)
  {
    Json::CharReaderBuilder builder;
    Json::Value root;
    std::string error;
    std::istringstream stream(layout_json);
    if (!Json::parseFromStream(builder, stream, &root, &error)) {
      RCLCPP_WARN(get_logger(), "motor layout JSON parse failed: %s", error.c_str());
      return false;
    }

    const Json::Value * motors = nullptr;
    if (root.isArray()) {
      motors = &root;
    } else if (root.isObject() && root["motors"].isArray()) {
      motors = &root["motors"];
    } else if (root.isObject() && root["motor"].isArray()) {
      motors = &root["motor"];
    }
    if (!motors) {
      RCLCPP_WARN(get_logger(), "motor layout does not contain a motors array");
      return false;
    }

    std::vector<MotorLayoutEntry> parsed;
    std::map<MotorKey, std::string> keys;
    for (const auto & motor : *motors) {
      if (!motor.isObject()) {
        continue;
      }
      MotorLayoutEntry entry;
      if (!read_uint_field(motor, {"limbNo", "limbsNo", "limb_no", "limbs_no"},
        entry.key.limb_no) ||
        !read_uint_field(motor, {"jointNo", "joint_no"}, entry.key.joint_no))
      {
        continue;
      }
      entry.name = read_string_field(motor, {"name", "joint", "jointName", "joint_name"});
      if (entry.name.empty() || keys.count(entry.key) != 0) {
        continue;
      }
      keys.emplace(entry.key, entry.name);
      parsed.push_back(std::move(entry));
    }
    if (parsed.empty()) {
      RCLCPP_WARN(get_logger(), "motor layout contains no usable motor entries");
      return false;
    }

    motor_layout_ = std::move(parsed);
    RCLCPP_INFO(get_logger(), "configured joint_states for %zu motor(s)", motor_layout_.size());
    return true;
  }

  bool configure_fallback_motor_layout()
  {
    std::size_t expected_names = 0;
    for (const auto size : fallback_limb_sizes_) {
      if (size <= 0) {
        RCLCPP_WARN(get_logger(), "fallback_limb_sizes must contain positive values");
        return false;
      }
      expected_names += static_cast<std::size_t>(size);
    }
    if (expected_names == 0 || expected_names != fallback_joint_names_.size()) {
      RCLCPP_WARN(
        get_logger(), "fallback motor layout has %zu slot(s) but %zu joint name(s)",
        expected_names, fallback_joint_names_.size());
      return false;
    }

    std::vector<MotorLayoutEntry> parsed;
    parsed.reserve(expected_names);
    std::size_t name_index = 0;
    for (std::size_t limb = 0; limb < fallback_limb_sizes_.size(); ++limb) {
      for (int64_t joint = 0; joint < fallback_limb_sizes_[limb]; ++joint) {
        const auto & name = fallback_joint_names_[name_index++];
        if (name.empty()) {
          RCLCPP_WARN(get_logger(), "fallback motor layout contains an empty joint name");
          return false;
        }
        parsed.push_back({
          {static_cast<std::uint32_t>(limb), static_cast<std::uint32_t>(joint)}, name});
      }
    }
    motor_layout_ = std::move(parsed);
    RCLCPP_WARN(
      get_logger(), "using configured fallback joint layout for %zu motor(s)", motor_layout_.size());
    return true;
  }

  void configure_joint_states()
  {
    if (motion_observed_enabled_) {
      return;
    }

    if (publish_joint_states_) {
      std::string layout_json;
      const bool queried = motion_client_->queryMotorLayout(layout_json, 3000);
      if (!queried || !parse_motor_layout(layout_json)) {
        const auto error = motion_client_->getLastError();
        if (!configure_fallback_motor_layout()) {
          RCLCPP_WARN(
            get_logger(), "joint_states disabled because motor layout is unavailable (error=%d)", error);
          motor_layout_.clear();
        }
      }
    }
    if (!motion_client_->setMotionObservedEnable(true, true, 3000)) {
      const auto error = motion_client_->getLastError();
      RCLCPP_WARN(
        get_logger(), "motion observation topics disabled because observation could not be enabled (error=%d)",
        error);
      motor_layout_.clear();
      return;
    }
    motion_observed_enabled_ = true;
  }

  void disable_motion_observed()
  {
    if (!motion_observed_enabled_) {
      return;
    }
    (void)motion_client_->setMotionObservedEnable(false, false, 3000);
    motion_observed_enabled_ = false;
  }

  void publish_motion_observed(const uniubi::msg::MotionObserved & input)
  {
    if (!motion_observed_enabled_) {
      return;
    }
    const auto stamp = now();
    publish_joint_states(input, stamp);
    publish_imu(input, stamp);

    const auto steady_now = std::chrono::steady_clock::now();
    if (steady_now >= next_battery_publish_at_) {
      publish_battery(input, stamp);
      next_battery_publish_at_ = steady_now + battery_publish_period_;
    }
  }

  void publish_joint_states(
    const uniubi::msg::MotionObserved & input,
    const rclcpp::Time & stamp)
  {
    if (!publish_joint_states_ || motor_layout_.empty()) {
      return;
    }

    const auto motor_count = std::clamp<int32_t>(
      input.motor_num, 0, static_cast<int32_t>(input.motor.size()));
    std::map<MotorKey, const uniubi::msg::MotorObserved *> observed;
    for (int32_t index = 0; index < motor_count; ++index) {
      const auto & motor = input.motor[static_cast<std::size_t>(index)];
      observed[{motor.header.limbs_no, motor.header.joint_no}] = &motor;
    }

    sensor_msgs::msg::JointState output;
    output.header.stamp = stamp;
    output.name.reserve(motor_layout_.size());
    output.position.reserve(motor_layout_.size());
    output.velocity.reserve(motor_layout_.size());
    output.effort.reserve(motor_layout_.size());
    for (const auto & layout : motor_layout_) {
      const auto found = observed.find(layout.key);
      if (found == observed.end()) {
        continue;
      }
      output.name.push_back(layout.name);
      output.position.push_back(found->second->position);
      output.velocity.push_back(found->second->velocity);
      output.effort.push_back(found->second->torque);
    }
    if (!output.name.empty()) {
      joint_state_publisher_->publish(output);
    }
  }

  void publish_imu(const uniubi::msg::MotionObserved & input, const rclcpp::Time & stamp)
  {
    const auto & accel = input.imu.accel;
    const auto & gyro = input.imu.gyro;
    if (accel.error != 0 || gyro.error != 0 ||
      !std::isfinite(accel.x) || !std::isfinite(accel.y) || !std::isfinite(accel.z) ||
      !std::isfinite(gyro.x) || !std::isfinite(gyro.y) || !std::isfinite(gyro.z))
    {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000, "not publishing IMU because accel/gyro is invalid");
      return;
    }

    sensor_msgs::msg::Imu output;
    output.header.stamp = stamp;
    output.header.frame_id = imu_frame_id_;
    output.linear_acceleration.x = accel.x;
    output.linear_acceleration.y = accel.y;
    output.linear_acceleration.z = accel.z;
    output.angular_velocity.x = gyro.x;
    output.angular_velocity.y = gyro.y;
    output.angular_velocity.z = gyro.z;

    const auto & quaternion = input.imu.quaternion;
    const double norm = std::sqrt(
      static_cast<double>(quaternion.w) * quaternion.w +
      static_cast<double>(quaternion.x) * quaternion.x +
      static_cast<double>(quaternion.y) * quaternion.y +
      static_cast<double>(quaternion.z) * quaternion.z);
    if (quaternion.error == 0 && std::isfinite(norm) && norm > 1.0e-6) {
      output.orientation.w = quaternion.w / norm;
      output.orientation.x = quaternion.x / norm;
      output.orientation.y = quaternion.y / norm;
      output.orientation.z = quaternion.z / norm;
    } else {
      output.orientation_covariance[0] = -1.0;
    }
    imu_publisher_->publish(output);
  }

  void publish_battery(const uniubi::msg::MotionObserved & input, const rclcpp::Time & stamp)
  {
    const auto nan = std::numeric_limits<float>::quiet_NaN();
    sensor_msgs::msg::BatteryState output;
    output.header.stamp = stamp;
    output.voltage = std::isfinite(input.power.charge_voltage) ? input.power.charge_voltage : nan;
    output.temperature = std::isfinite(input.power.temper) ? input.power.temper : nan;
    output.current = std::isfinite(input.power.charge_current) ? input.power.charge_current : nan;
    output.charge = nan;
    output.capacity = nan;
    output.design_capacity = nan;
    output.percentage = std::isfinite(input.power.power) ?
      std::clamp(input.power.power / 100.0F, 0.0F, 1.0F) : nan;
    output.power_supply_status = sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_UNKNOWN;
    output.power_supply_health = sensor_msgs::msg::BatteryState::POWER_SUPPLY_HEALTH_UNKNOWN;
    output.power_supply_technology = sensor_msgs::msg::BatteryState::POWER_SUPPLY_TECHNOLOGY_UNKNOWN;
    output.present = std::isfinite(input.power.charge_voltage) && input.power.charge_voltage > 0.0F;
    battery_publisher_->publish(output);
  }

  static void set_response(
    const TriggerResponse::SharedPtr & response,
    const CommandResult & result)
  {
    response->success = result.success;
    response->message = result.message;
    if (!result.success) {
      response->message += " (error=" + std::to_string(result.error_code) + ")";
    }
  }

  std::string robot_service_name_;
  std::string event_topic_;
  std::string device_id_;
  int32_t lease_ms_{60000};
  bool auto_connect_{true};
  std::string cmd_vel_topic_;
  int32_t cmd_vel_timeout_ms_{500};
  double cmd_vel_rate_hz_{50.0};
  std::string odom_topic_;
  std::string odom_frame_id_;
  std::string base_frame_id_;
  bool publish_joint_states_{true};
  std::string joint_states_topic_;
  std::vector<int64_t> fallback_limb_sizes_;
  std::vector<std::string> fallback_joint_names_;
  std::string imu_topic_;
  std::string imu_frame_id_;
  std::string battery_topic_;
  double battery_publish_rate_hz_{1.0};
  std::string motion_status_topic_;
  double motion_status_rate_hz_{10.0};

  rclcpp::Node::SharedPtr client_node_;
  std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> client_executor_;
  std::unique_ptr<MotionClient> motion_client_;
  rclcpp::Publisher<uniubi_motion_bridge::msg::MotionStatus>::SharedPtr motion_status_publisher_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr battery_publisher_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_subscription_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr release_control_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr query_capabilities_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr stop_action_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr emergency_stop_service_;
  rclcpp::Service<uniubi_motion_bridge::srv::StartMotionAction>::SharedPtr start_action_service_;

  std::atomic<bool> stop_{false};
  std::thread worker_;
  std::mutex queue_mutex_;
  std::condition_variable queue_cv_;
  std::deque<QueuedCommand> commands_;
  std::mutex velocity_mutex_;
  std::optional<VelocityCommand> pending_velocity_;
  std::chrono::steady_clock::time_point last_velocity_received_at_{};
  std::chrono::steady_clock::time_point next_velocity_dispatch_at_{};
  std::chrono::milliseconds velocity_dispatch_period_{100};
  std::string current_action_;
  float current_linear_x_{0.0F};
  float current_linear_y_{0.0F};
  float current_angular_z_{0.0F};
  int32_t last_error_code_{MotionClient::kNone};
  std::string last_error_message_;
  bool last_error_from_query_{false};
  bool velocity_command_active_{false};
  bool velocity_watchdog_fired_{false};
  std::vector<MotorLayoutEntry> motor_layout_;
  bool motion_observed_enabled_{false};
  bool rpc_ready_{false};
  std::chrono::steady_clock::time_point next_battery_publish_at_{};
  std::chrono::milliseconds battery_publish_period_{1000};
  std::chrono::steady_clock::time_point next_motion_status_query_at_{};
  std::chrono::milliseconds motion_status_publish_period_{100};
};

}  // namespace uniubi_motion_bridge

int main(int argc, char ** argv)
{
  rclcpp::InitOptions init_options;
  rclcpp::init(argc, argv, init_options, rclcpp::SignalHandlerOptions::None);
  std::signal(SIGINT, uniubi_motion_bridge::handle_signal);
  std::signal(SIGTERM, uniubi_motion_bridge::handle_signal);
  auto node = std::make_shared<uniubi_motion_bridge::MotionBridgeNode>();
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  rclcpp::WallRate loop_rate(100.0);
  while (rclcpp::ok() && !uniubi_motion_bridge::shutdown_requested) {
    executor.spin_some();
    loop_rate.sleep();
  }
  executor.remove_node(node);
  node.reset();
  rclcpp::shutdown();
  return 0;
}
