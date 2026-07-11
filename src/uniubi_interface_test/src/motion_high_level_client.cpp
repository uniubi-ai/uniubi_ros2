#include "uniubi_interface_test/motion_high_level_client.hpp"

#include <algorithm>
#include <cstdint>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <utility>

#include "json/json.h"

namespace uniubi_interface_test
{

namespace
{

constexpr const char * kRobotAppService = "robotAppService";
constexpr const char * kRawControlTopic = "motion/trc";
constexpr const char * kHostEventTopic = "robotServer.host.event";
constexpr const char * kControlStatusTopic = "robotServer.control.status";
constexpr std::uint32_t kEventMagic = 0x53425645U;
constexpr int32_t kDefaultLeaseMs = 60000;
constexpr int32_t kRenewTimeoutMs = 3000;
constexpr int32_t kRenewTimerPeriodMs = 200;

Json::Value null_params()
{
  return Json::Value(Json::nullValue);
}

Json::Value empty_object()
{
  return Json::Value(Json::objectValue);
}

Json::Value bool_param(const std::string & name, bool value)
{
  Json::Value params(Json::objectValue);
  params[name] = value;
  return params;
}

std::string write_json(const Json::Value & value)
{
  Json::StreamWriterBuilder builder;
  builder["indentation"] = "";
  return Json::writeString(builder, value);
}

bool parse_json_no_throw(const std::string & json, Json::Value & value, std::string & error)
{
  Json::CharReaderBuilder builder;
  std::istringstream stream(json);
  return Json::parseFromStream(builder, stream, &value, &error);
}

bool read_controlled(const Json::Value & payload)
{
  if (!payload.isMember("controlled")) {
    return false;
  }

  const auto & value = payload["controlled"];
  if (value.isBool()) {
    return value.asBool();
  }
  if (value.isString()) {
    return value.asString() != "0";
  }
  return false;
}

}  // namespace

MotionHighLevelClient::MotionHighLevelClient(
  const rclcpp::Node::SharedPtr & node,
  rclcpp::Executor & executor,
  const std::string & ros_service_name,
  const std::string & device_id,
  const std::string & event_topic)
: SystemRpcClientBase(node, ros_service_name, device_id),
  node_(node),
  executor_(executor),
  pending_renew_request_id_(std::nullopt),
  pending_renew_deadline_(std::nullopt),
  last_renew_at_(std::chrono::steady_clock::now()),
  renew_sequence_(0),
  event_topic_(event_topic),
  raw_action_id_(0),
  raw_control_seq_(1),
  lease_ms_(kDefaultLeaseMs),
  state_(kDisconnected),
  last_error_(kNone)
{
}

MotionHighLevelClient::~MotionHighLevelClient()
{
  disconnect();
}

bool MotionHighLevelClient::connect(int32_t lease_ms)
{
  if (state_ != kDisconnected) {
    return true;
  }

  if (!wait_for_service(timeout_from_ms(5000))) {
    set_error(kRpcConnectFailed);
    return false;
  }

  lease_ms_ = lease_ms > 0 ? lease_ms : kDefaultLeaseMs;
  controller_.clear();
  raw_action_id_ = 0;
  raw_control_seq_ = 1;
  create_event_subscription();
  set_error(kNone);
  state_ = kConnected;
  return true;
}

void MotionHighLevelClient::disconnect()
{
  if (state_ == kControlled) {
    (void)releaseControl();
  }

  stop_renew_timer();
  destroy_event_subscription();
  raw_control_publisher_.reset();
  controller_.clear();
  raw_action_id_ = 0;
  state_ = kDisconnected;
}

bool MotionHighLevelClient::startControl(int32_t timeout_ms)
{
  if (!ensure_connected()) {
    return false;
  }

  if (state_ == kControlled) {
    return true;
  }

  Json::Value params(Json::objectValue);
  if (lease_ms_ > 0) {
    params["leaseTimeout"] = lease_ms_;
  }

  Json::Value ret;
  if (!rpc_call(
      "takeMotionControl",
      client_id(),
      params,
      ret,
      timeout_ms,
      "takeMotionControl"))
  {
    set_error(kRpcAcquireRejected);
    if (connect_callback_) {
      connect_callback_(kConnected, kRpcAcquireRejected);
    }
    return false;
  }

  if (!ret.isObject() || !ret.isMember("controller") || !ret["controller"].isString()) {
    set_error(kRpcAcquireRejected);
    if (connect_callback_) {
      connect_callback_(kConnected, kRpcAcquireRejected);
    }
    return false;
  }

  controller_ = ret["controller"].asString();
  if (ret.isMember("leaseTimeout") && ret["leaseTimeout"].isNumeric()) {
    lease_ms_ = ret["leaseTimeout"].asInt();
  }
  raw_action_id_ =
    ret.isMember("rawActionId") && ret["rawActionId"].isNumeric() ? ret["rawActionId"].asUInt64() : 0;
  raw_control_seq_ = 1;
  last_renew_at_ = std::chrono::steady_clock::now();
  state_ = kControlled;
  set_error(kNone);
  start_renew_timer();

  if (connect_callback_) {
    connect_callback_(kControlled, kNone);
  }
  return true;
}

bool MotionHighLevelClient::releaseControl()
{
  if (state_ != kControlled) {
    set_error(kNotControlled);
    return false;
  }

  if (!rpc_send_action(
      "releaseMotionControl",
      controller_,
      null_params(),
      5000,
      "releaseMotionControl"))
  {
    return false;
  }

  stop_renew_timer();
  controller_.clear();
  raw_action_id_ = 0;
  state_ = kConnected;
  set_error(kNone);

  if (connect_callback_) {
    connect_callback_(kConnected, kNone);
  }
  return true;
}

int32_t MotionHighLevelClient::getState() const
{
  return state_;
}

int32_t MotionHighLevelClient::getLastError() const
{
  const auto error = last_error_;
  last_error_ = kNone;
  return error;
}

void MotionHighLevelClient::setConnectCallback(ConnectCallback cb)
{
  if (state_ != kDisconnected) {
    return;
  }
  connect_callback_ = std::move(cb);
}

void MotionHighLevelClient::setEventCallback(EventCallback cb)
{
  if (state_ != kDisconnected) {
    return;
  }
  event_callback_ = std::move(cb);
}

bool MotionHighLevelClient::queryCapabilities(std::string & out, int32_t timeout_ms)
{
  if (!ensure_connected()) {
    return false;
  }

  Json::Value ret;
  if (!rpc_call(
      "getMotionCapabilities",
      client_id(),
      null_params(),
      ret,
      timeout_ms,
      "getMotionCapabilities"))
  {
    return false;
  }

  return response_to_output(ret, out);
}

bool MotionHighLevelClient::querySystemStatus(std::string & out, int32_t timeout_ms)
{
  if (!ensure_connected()) {
    return false;
  }

  Json::Value ret;
  if (!rpc_call("getSystemStatus", controller_.empty() ? client_id() : controller_, null_params(), ret, timeout_ms,
      "getSystemStatus"))
  {
    return false;
  }

  return response_to_output(ret, out);
}

bool MotionHighLevelClient::queryMotionState(std::string & out, int32_t timeout_ms)
{
  if (!ensure_connected()) {
    return false;
  }

  Json::Value ret;
  if (!rpc_call("queryMotionState", controller_.empty() ? client_id() : controller_, null_params(), ret, timeout_ms,
      "queryMotionState"))
  {
    return false;
  }

  return response_to_output(ret, out);
}

bool MotionHighLevelClient::startAction(
  const std::string & action,
  const std::string & params_json,
  int32_t timeout_ms)
{
  if (!ensure_controlled()) {
    return false;
  }

  Json::Value action_params;
  if (!parse_params_json(params_json, action_params, "startMotionAction")) {
    return false;
  }

  Json::Value params(Json::objectValue);
  params["action"] = action;
  if (!action_params.isNull()) {
    params["params"] = action_params;
  }

  Json::Value ret;
  return rpc_call("startMotionAction", controller_, params, ret, timeout_ms, "startMotionAction");
}

bool MotionHighLevelClient::stopAction(int32_t timeout_ms)
{
  if (!ensure_controlled()) {
    return false;
  }

  Json::Value ret;
  return rpc_call("stopMotionAction", controller_, null_params(), ret, timeout_ms, "stopMotionAction");
}

bool MotionHighLevelClient::setActionParams(
  const std::string & params_json,
  int32_t timeout_ms)
{
  if (!ensure_controlled()) {
    return false;
  }

  Json::Value action_params;
  if (!parse_params_json(params_json, action_params, "setMotionActionParams")) {
    return false;
  }

  Json::Value params(Json::objectValue);
  if (!action_params.isNull()) {
    params["params"] = action_params;
  }

  Json::Value ret;
  return rpc_call("setMotionActionParams", controller_, params, ret, timeout_ms, "setMotionActionParams");
}

bool MotionHighLevelClient::setRawControlCmd(const TRCStickFrame & frame)
{
  if (!ensure_controlled()) {
    return false;
  }

  if (raw_action_id_ == 0) {
    set_error(kActionRejected);
    return false;
  }

  if (!raw_control_publisher_) {
    raw_control_publisher_ = node_->create_publisher<RemoteControl>(kRawControlTopic, rclcpp::QoS(1));
  }

  RemoteControl message;
  message.controller = raw_action_id_;
  message.timestamp = raw_control_seq_++;
  if (frame.valid) {
    message.back = frame.buttons[buttonBack];
    message.start = frame.buttons[buttonStart];
    message.lb = frame.buttons[buttonLB];
    message.rb = frame.buttons[buttonRB];
    message.f1 = frame.buttons[buttonF1];
    message.f2 = frame.buttons[buttonF2];
    message.a = frame.buttons[buttonA];
    message.b = frame.buttons[buttonB];
    message.x = frame.buttons[buttonX];
    message.y = frame.buttons[buttonY];
    message.up = frame.buttons[buttonUp];
    message.down = frame.buttons[buttonDown];
    message.left = frame.buttons[buttonLeft];
    message.right = frame.buttons[buttonRight];
    message.ls = frame.buttons[buttonLS];
    message.rs = frame.buttons[buttonRS];
    message.stick_lx = frame.axes[axesLX];
    message.stick_ly = frame.axes[axesLY];
    message.stick_rx = frame.axes[axesRX];
    message.stick_ry = frame.axes[axesRY];
    message.trigger_l = frame.axes[axesLT];
    message.trigger_r = frame.axes[axesRT];
  }

  raw_control_publisher_->publish(message);
  set_error(kNone);
  return true;
}

bool MotionHighLevelClient::emergencyStop(int32_t timeout_ms)
{
  if (!ensure_controlled()) {
    return false;
  }

  Json::Value ret;
  return rpc_call("emergencyStopMotion", controller_, null_params(), ret, timeout_ms, "emergencyStopMotion");
}

bool MotionHighLevelClient::setMotionObservedEnable(bool motion_enable, bool sensor_enable, int32_t timeout_ms)
{
  if (!ensure_connected()) {
    return false;
  }

  Json::Value params(Json::objectValue);
  params["motionEnable"] = motion_enable;
  params["sensorEnable"] = sensor_enable;

  Json::Value ret;
  return rpc_call(
    "setMotionObservedEnable",
    controller_.empty() ? client_id() : controller_,
    params,
    ret,
    timeout_ms,
    "setMotionObservedEnable");
}

bool MotionHighLevelClient::startAudioPlay(
  const std::string & params_json,
  int32_t timeout_ms)
{
  if (!ensure_controlled()) {
    return false;
  }

  Json::Value params;
  if (!parse_params_json(params_json, params, "startPlayList")) {
    return false;
  }

  Json::Value ret;
  return rpc_call("startPlayList", controller_, params, ret, timeout_ms, "startPlayList");
}

bool MotionHighLevelClient::stopAudioPlay(int32_t timeout_ms)
{
  if (!ensure_controlled()) {
    return false;
  }

  Json::Value ret;
  return rpc_call("stopPlayList", controller_, null_params(), ret, timeout_ms, "stopPlayList");
}

bool MotionHighLevelClient::pauseAudioPlay(int32_t timeout_ms)
{
  if (!ensure_controlled()) {
    return false;
  }

  Json::Value ret;
  return rpc_call("stopPlayList", controller_, bool_param("pause", true), ret, timeout_ms, "pauseAudioPlay");
}

bool MotionHighLevelClient::queryAudioPlayDetail(std::string & out, int32_t timeout_ms)
{
  if (!ensure_connected()) {
    return false;
  }

  Json::Value ret;
  if (!rpc_call(
      "getAudioPlayDetail",
      controller_.empty() ? client_id() : controller_,
      null_params(),
      ret,
      timeout_ms,
      "getAudioPlayDetail"))
  {
    return false;
  }

  return response_to_output(ret, out);
}

bool MotionHighLevelClient::queryAudioPlayList(
  std::string & out,
  const std::string & params_json,
  int32_t timeout_ms)
{
  if (!ensure_connected()) {
    return false;
  }

  Json::Value params;
  if (!parse_params_json(params_json, params, "getAudioPlayList")) {
    return false;
  }

  Json::Value ret;
  if (!rpc_call(
      "getAudioPlayList",
      controller_.empty() ? client_id() : controller_,
      params,
      ret,
      timeout_ms,
      "getAudioPlayList"))
  {
    return false;
  }

  return response_to_output(ret, out);
}

bool MotionHighLevelClient::deleteAudioFile(
  const std::string & params_json,
  int32_t timeout_ms)
{
  if (!ensure_controlled()) {
    return false;
  }

  Json::Value params;
  if (!parse_params_json(params_json, params, "deleteAudioFile")) {
    return false;
  }

  Json::Value ret;
  return rpc_call("deleteAudioFile", controller_, params, ret, timeout_ms, "deleteAudioFile");
}

bool MotionHighLevelClient::setCameraLightBrightness(int32_t brightness, int32_t timeout_ms)
{
  if (!ensure_controlled()) {
    return false;
  }

  if (brightness < 0 || brightness > 100) {
    set_error(kActionRejected);
    return false;
  }

  Json::Value params(Json::objectValue);
  params["brightness"] = brightness;
  Json::Value ret;
  return rpc_call("setCameraLightBrightness", controller_, params, ret, timeout_ms, "setCameraLightBrightness");
}

bool MotionHighLevelClient::ensure_connected()
{
  if (state_ == kDisconnected) {
    set_error(kNotConnected);
    return false;
  }
  return true;
}

bool MotionHighLevelClient::ensure_controlled()
{
  if (state_ != kControlled) {
    set_error(kNotControlled);
    return false;
  }
  return true;
}

bool MotionHighLevelClient::parse_params_json(
  const std::string & params_json,
  Json::Value & params,
  const std::string & context)
{
  if (params_json.empty()) {
    params = Json::Value(Json::nullValue);
    return true;
  }

  std::string error;
  if (!parse_json_no_throw(params_json, params, error)) {
    std::cerr << context << " params JSON parse failed: " << error << std::endl;
    set_error(kActionRejected);
    return false;
  }

  if (params.isNull()) {
    return true;
  }

  if (!params.isObject() && !params.isArray()) {
    std::cerr << context << " params JSON must be an object or array" << std::endl;
    set_error(kActionRejected);
    return false;
  }

  return true;
}

bool MotionHighLevelClient::rpc_call(
  const std::string & method,
  const std::string & rpc_client_id,
  const Json::Value & params,
  Json::Value & out,
  int32_t timeout_ms,
  const std::string & context)
{
  try {
    const auto response = call(
      make_call(kRobotAppService, method, params, rpc_client_id),
      executor_,
      timeout_from_ms(timeout_ms));

    if (!response) {
      set_error(kRpcCallFailed);
      return false;
    }

    if (response->code != 0) {
      std::cerr << context << " response code=" << response->code << std::endl;
      set_error(kRpcCallFailed);
      return false;
    }

    Json::Value payload;
    std::string error;
    if (!parse_json_no_throw(response->payload, payload, error) || !payload.isObject()) {
      std::cerr << context << " response payload parse failed: " << error << std::endl;
      set_error(kRpcCallFailed);
      return false;
    }

    out = payload.isMember("params") ? payload["params"] : Json::Value(Json::nullValue);
    if (payload.isMember("result") && payload["result"].asBool()) {
      set_error(kNone);
      return true;
    }

    set_error(kActionRejected);
    return false;
  } catch (const std::exception & e) {
    std::cerr << context << " RPC failed: " << e.what() << std::endl;
    set_error(kRpcCallFailed);
    return false;
  }
}

bool MotionHighLevelClient::rpc_send_action(
  const std::string & method,
  const std::string & rpc_client_id,
  const Json::Value & params,
  int32_t timeout_ms,
  const std::string & context)
{
  Json::Value unused;
  return rpc_call(method, rpc_client_id, params, unused, timeout_ms, context);
}

bool MotionHighLevelClient::response_to_output(const Json::Value & payload, std::string & out)
{
  out = payload.isNull() ? "{}" : write_json(payload);
  return true;
}

void MotionHighLevelClient::set_error(HighLevelError error)
{
  last_error_ = error;
}

std::chrono::milliseconds MotionHighLevelClient::timeout_from_ms(int32_t timeout_ms) const
{
  return std::chrono::milliseconds(timeout_ms > 0 ? timeout_ms : 5000);
}

std::chrono::milliseconds MotionHighLevelClient::renew_interval() const
{
  const auto lease = std::chrono::milliseconds(lease_ms_ > 0 ? lease_ms_ : kDefaultLeaseMs);
  return std::clamp(lease / 3, std::chrono::milliseconds(200), std::chrono::milliseconds(10000));
}

void MotionHighLevelClient::start_renew_timer()
{
  stop_renew_timer();
  renew_timer_ = node_->create_wall_timer(
    std::chrono::milliseconds(kRenewTimerPeriodMs),
    [this]() {
      tick_renew();
    });
}

void MotionHighLevelClient::stop_renew_timer()
{
  if (renew_timer_) {
    renew_timer_->cancel();
    renew_timer_.reset();
  }

  if (pending_renew_request_id_.has_value()) {
    (void)remove_pending_request(*pending_renew_request_id_);
    pending_renew_request_id_.reset();
  }
  pending_renew_deadline_.reset();
}

void MotionHighLevelClient::tick_renew()
{
  if (state_ != kControlled || controller_.empty()) {
    stop_renew_timer();
    return;
  }

  const auto now = std::chrono::steady_clock::now();
  if (pending_renew_request_id_.has_value()) {
    if (pending_renew_deadline_.has_value() && now >= *pending_renew_deadline_) {
      (void)remove_pending_request(*pending_renew_request_id_);
      pending_renew_request_id_.reset();
      pending_renew_deadline_.reset();
      lose_control(kSessionExpired);
    }
    return;
  }

  if (now < last_renew_at_ + renew_interval()) {
    return;
  }

  const auto sequence = ++renew_sequence_;
  const auto controller = controller_;
  try {
    auto future = async_call(
      make_call(kRobotAppService, "renewMotionControl", null_params(), controller),
      [this, sequence, controller](SharedFuture future) {
        handle_renew_response(sequence, controller, future);
      });
    pending_renew_request_id_ = future.request_id;
    pending_renew_deadline_ = now + std::chrono::milliseconds(kRenewTimeoutMs);
  } catch (const std::exception & e) {
    std::cerr << "renewMotionControl async RPC failed: " << e.what() << std::endl;
    lose_control(kSessionExpired);
  }
}

void MotionHighLevelClient::handle_renew_response(
  std::uint64_t sequence,
  const std::string & controller,
  SharedFuture future)
{
  if (!pending_renew_request_id_.has_value() || sequence != renew_sequence_) {
    return;
  }
  pending_renew_request_id_.reset();
  pending_renew_deadline_.reset();

  if (state_ != kControlled || controller != controller_) {
    return;
  }

  try {
    const auto response = future.get();
    if (!response || response->code != 0) {
      lose_control(kSessionExpired);
      return;
    }

    Json::Value payload;
    std::string error;
    if (!parse_json_no_throw(response->payload, payload, error) || !payload.isObject() ||
      !payload.isMember("result") || !payload["result"].asBool())
    {
      lose_control(kSessionExpired);
      return;
    }

    if (payload.isMember("params") && payload["params"].isObject() &&
      payload["params"].isMember("leaseTimeout") && payload["params"]["leaseTimeout"].isNumeric())
    {
      lease_ms_ = payload["params"]["leaseTimeout"].asInt();
    }
    last_renew_at_ = std::chrono::steady_clock::now();
    set_error(kNone);
  } catch (const std::exception & e) {
    std::cerr << "renewMotionControl response failed: " << e.what() << std::endl;
    lose_control(kSessionExpired);
  }
}

void MotionHighLevelClient::lose_control(HighLevelError error)
{
  stop_renew_timer();
  controller_.clear();
  raw_action_id_ = 0;
  state_ = kConnected;
  set_error(error);
  if (connect_callback_) {
    connect_callback_(kConnected, error);
  }
}

void MotionHighLevelClient::create_event_subscription()
{
  if (event_subscription_ || event_topic_.empty()) {
    return;
  }

  event_subscription_ = node_->create_subscription<EventMessage>(
    event_topic_,
    rclcpp::QoS(10),
    [this](const EventMessage::SharedPtr message) {
      handle_event(*message);
    });
}

void MotionHighLevelClient::destroy_event_subscription()
{
  event_subscription_.reset();
}

void MotionHighLevelClient::handle_event(const EventMessage & event)
{
  if (event.magic != kEventMagic) {
    return;
  }

  Json::Value payload;
  std::string error;
  if (!parse_json_no_throw(event.payload, payload, error) || !payload.isObject()) {
    if (event_callback_) {
      event_callback_(event.topic, event.payload);
    }
    return;
  }

  if (event.topic == kControlStatusTopic) {
    if (state_ != kControlled) {
      return;
    }

    const auto controlled = read_controlled(payload);
    const auto current_controller =
      payload.isMember("controller") && payload["controller"].isString() ?
      payload["controller"].asString() : std::string();
    if (controlled && current_controller == controller_) {
      return;
    }

    lose_control(kSessionRevoked);
    return;
  }

  if (!event_callback_) {
    return;
  }

  if (event.topic == kHostEventTopic &&
    payload.isMember("event") && payload["event"].isString() &&
    payload.isMember("detail"))
  {
    event_callback_(payload["event"].asString(), write_json(payload["detail"]));
    return;
  }

  event_callback_(event.topic, event.payload);
}

}  // namespace uniubi_interface_test
