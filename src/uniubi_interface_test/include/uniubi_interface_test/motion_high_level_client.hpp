#ifndef UNIUBI_INTERFACE_TEST__MOTION_HIGH_LEVEL_CLIENT_HPP_
#define UNIUBI_INTERFACE_TEST__MOTION_HIGH_LEVEL_CLIENT_HPP_

#include <chrono>
#include <cstdint>
#include <functional>
#include <optional>
#include <string>

#include "json/json.h"
#include "rclcpp/rclcpp.hpp"
#include "uniubi/msg/event_message.hpp"
#include "uniubi/msg/remote_control.hpp"
#include "uniubi_interface_test/system_rpc_client_base.hpp"

namespace uniubi_interface_test
{

/**
 * @brief 基于 uniubi::srv::System 的高级运动控制客户端。
 *
 * 该类封装 robotServer 的 robotAppService RPC 接口，接口风格参考 MotionHighLevelClient SDK。
 * 使用流程：
 * 1. connect()：初始化客户端状态并订阅事件，不获取控制权。
 * 2. query*()：可在已连接状态下查询能力、系统状态、音频状态等只读信息。
 * 3. startControl()：申请高级运动控制权，成功后进入 kControlled。
 * 4. 动作/音频/灯光等控制类接口必须在 kControlled 状态下调用。
 * 5. releaseControl()/disconnect()：释放控制权并清理订阅/续约状态。
 *
 * 注意：MotionHighLevelClient 内部使用 ROS timer、service response callback 和 subscription callback，
 * 调用方必须持续 spin 传入的 executor，否则事件处理和控制权租约续约不会执行。
 */
class MotionHighLevelClient : public SystemRpcClientBase
{
public:
  /// 高级控制客户端状态。
  enum HighLevelState
  {
    kDisconnected = 0,  ///< 未连接，不能调用需要连接态的接口。
    kConnected,         ///< 已连接但未持有控制权，可调用只读查询接口。
    kControlled,        ///< 已持有高级运动控制权，可下发动作/音频/灯光等控制接口。
  };

  /// 最近一次失败原因，数值保持与 SDK 高级客户端语义一致。
  enum HighLevelError
  {
    kNone = 0,             ///< 无错误。
    kRpcConnectFailed,     ///< RPC 服务不可用或连接初始化失败。
    kRpcAcquireRejected,   ///< startControl() 获取控制权失败或被服务端拒绝。
    kRpcCallFailed,        ///< RPC 调用失败、超时或响应解析失败。
    kSessionExpired,       ///< 控制权租约续约失败或超时。
    kSessionRevoked,       ///< 控制权被其他客户端接管或服务端事件显示已失权。
    kNotConnected,         ///< 未 connect() 时调用了需要连接态的接口。
    kNotControlled,        ///< 未持有控制权时调用了控制类接口。
    kActionRejected,       ///< 服务端返回 result=false 或参数不合法。
  };

  /// 原始 TRC 控制帧中的按键索引。
  enum ButtonDefine
  {
    buttonBack = 0,
    buttonStart,
    buttonLB,
    buttonRB,
    buttonF1,
    buttonF2,
    buttonA,
    buttonB,
    buttonX,
    buttonY,
    buttonUp,
    buttonDown,
    buttonLeft,
    buttonRight,
    buttonLS,
    buttonRS,
    BUTTON_MAX,
  };

  /// 原始 TRC 控制帧中的摇杆/扳机索引。
  enum AxesDefine
  {
    axesLX = 0,
    axesLY,
    axesRX,
    axesRY,
    axesLT,
    axesRT,
    AXES_MAX,
  };

  /// 原始 TRC 控制帧。setRawControlCmd() 会把该结构映射为 uniubi::msg::RemoteControl。
  struct TRCStickFrame
  {
    std::uint32_t valid = 0;                 ///< 非 0 表示 buttons/axes 数据有效。
    std::uint8_t buttons[BUTTON_MAX] = {};   ///< 按键状态，索引见 ButtonDefine。
    float axes[AXES_MAX] = {};               ///< 摇杆和扳机值，索引见 AxesDefine。
    std::uint64_t control_id = 0;            ///< 保留字段；实际发送时使用服务端下发的 rawActionId。
  };

  /// 控制权状态变化回调。成功取权、释放、续约失效、被抢权时触发。
  using ConnectCallback = std::function<void(HighLevelState state, HighLevelError error)>;

  /// 业务事件回调。robotServer.host.event 会被解包后传入内层业务 topic 和 detail JSON。
  using EventCallback = std::function<void(const std::string & topic, const std::string & payload_json)>;

  /**
   * @brief 创建高级运动控制客户端。
   * @param node ROS 2 节点。
   * @param executor 用于同步 RPC、timer、service response 和 event subscription 的 executor。
   * @param ros_service_name ROS 2 System 服务名称，当前默认为 robotServer。
   * @param device_id 目标设备 ID，多设备场景必须传入。
   * @param event_topic robotServer 事件 topic。
   */
  MotionHighLevelClient(
    const rclcpp::Node::SharedPtr & node,
    rclcpp::Executor & executor,
    const std::string & ros_service_name,
    const std::string & device_id = "",
    const std::string & event_topic = "/robotServer/Event");

  ~MotionHighLevelClient() override;

  /**
   * @brief 进入高级客户端连接态。
   *
   * 该函数只等待 ROS service 可用并创建事件订阅，不申请控制权。
   * lease_ms 是后续 startControl() 申请控制权时传给服务端的期望租约时长；
   * <=0 时使用默认 60000ms。
   */
  bool connect(int32_t lease_ms = 0);

  /// 断开客户端；如果当前持有控制权，会先尝试 releaseControl()。
  void disconnect();

  /**
   * @brief 申请高级运动控制权。
   *
   * 成功后保存服务端返回的 controller/rawActionId，状态切为 kControlled，
   * 并启动内部定时器周期调用 renewMotionControl 续约。
   */
  bool startControl(int32_t timeout_ms = 10000);

  /// 释放高级运动控制权，成功后状态切回 kConnected，并停止续约。
  bool releaseControl();

  /// 获取当前 HighLevelState。
  int32_t getState() const;

  /// 获取最近一次失败原因；读取后会清零为 kNone。
  int32_t getLastError() const;

  /// 注册控制权状态变化回调。应在 connect() 前设置。
  void setConnectCallback(ConnectCallback cb);

  /// 注册业务事件回调。应在 connect() 前设置。
  void setEventCallback(EventCallback cb);

  /// 查询运动能力列表。已 connect 即可调用，不要求持有控制权。
  bool queryCapabilities(std::string & out, int32_t timeout_ms = 5000);

  /// 查询系统状态。已 connect 即可调用。
  bool querySystemStatus(std::string & out, int32_t timeout_ms = 5000);

  /// 查询当前运动状态。已 connect 即可调用。
  bool queryMotionState(std::string & out, int32_t timeout_ms = 5000);

  /**
   * @brief 启动高级动作。
   * @param action 动作名称，例如 walking。
   * @param params_json 动作参数 JSON，字段以 queryCapabilities() 返回为准。
   *
   * 必须先 startControl()。不带速度参数时通常只切入动作姿态。
   */
  bool startAction(
    const std::string & action,
    const std::string & params_json = "",
    int32_t timeout_ms = 5000);

  /// 停止当前高级动作。必须持有控制权。
  bool stopAction(int32_t timeout_ms = 5000);

  /// 修改当前动作参数。必须持有控制权。
  bool setActionParams(
    const std::string & params_json = "",
    int32_t timeout_ms = 5000);

  /**
   * @brief 发送原始 TRC 控制帧。
   *
   * 必须持有控制权，并且 takeMotionControl 响应中包含非 0 rawActionId。
   * 发送时 RemoteControl.controller 使用 rawActionId，不使用字符串 controller token。
   */
  bool setRawControlCmd(const TRCStickFrame & frame);

  /// 急停。必须持有控制权。
  bool emergencyStop(int32_t timeout_ms = 5000);

  /// 开关运控/传感器观测量推送。已 connect 即可调用，不强制要求持有控制权。
  bool setMotionObservedEnable(bool motion_enable, bool sensor_enable = false, int32_t timeout_ms = 5000);

  /**
   * @brief 启动、恢复或调整音频播放列表。
   *
   * params_json 示例：{"list":[{"id":"1"}],"volume":50,"repeat":1}
   * 必须持有控制权。
   */
  bool startAudioPlay(
    const std::string & params_json,
    int32_t timeout_ms = 5000);

  /// 停止音频播放。必须持有控制权。
  bool stopAudioPlay(int32_t timeout_ms = 5000);

  /// 暂停音频播放。必须持有控制权。
  bool pauseAudioPlay(int32_t timeout_ms = 5000);

  /// 查询当前音频播放详情。已 connect 即可调用。
  bool queryAudioPlayDetail(std::string & out, int32_t timeout_ms = 5000);

  /// 查询音频文件列表。params_json 可传 {"type":"customVoice"}。
  bool queryAudioPlayList(
    std::string & out,
    const std::string & params_json = "",
    int32_t timeout_ms = 5000);

  /// 删除音频文件。params_json 示例：{"id":"1"}。必须持有控制权。
  bool deleteAudioFile(
    const std::string & params_json,
    int32_t timeout_ms = 5000);

  /// 设置摄像头前灯亮度，取值 0-100。必须持有控制权。
  bool setCameraLightBrightness(int32_t brightness, int32_t timeout_ms = 5000);

private:
  using EventMessage = uniubi::msg::EventMessage;
  using RemoteControl = uniubi::msg::RemoteControl;

  bool ensure_connected();

  bool ensure_controlled();

  /// 解析用户传入的 JSON 参数；空字符串会被视为 JSON null。
  bool parse_params_json(
    const std::string & params_json,
    Json::Value & params,
    const std::string & context);

  /// 同步调用 robotAppService RPC，并返回响应 payload.params。
  bool rpc_call(
    const std::string & method,
    const std::string & client_id,
    const Json::Value & params,
    Json::Value & out,
    int32_t timeout_ms,
    const std::string & context);

  /// 当前实现复用同步 RPC 调用，保留该函数用于表达无需业务返回值的 action 语义。
  bool rpc_send_action(
    const std::string & method,
    const std::string & client_id,
    const Json::Value & params,
    int32_t timeout_ms,
    const std::string & context);

  /// 将 payload.params 转成输出 JSON 字符串；null 输出为 {}。
  bool response_to_output(const Json::Value & payload, std::string & out);

  void set_error(HighLevelError error);

  std::chrono::milliseconds timeout_from_ms(int32_t timeout_ms) const;

  /// 按协议计算续约周期：clamp(leaseTimeout / 3, 200ms, 10s)。
  std::chrono::milliseconds renew_interval() const;

  /// 启动控制权续约 timer。
  void start_renew_timer();

  /// 停止续约 timer，并清理在途续约请求。
  void stop_renew_timer();

  /// timer 回调：到达续约周期后异步发送 renewMotionControl。
  void tick_renew();

  /// 处理 renewMotionControl 异步响应。
  void handle_renew_response(
    std::uint64_t sequence,
    const std::string & controller,
    SharedFuture future);

  /// 统一处理失权：停止续约、清空 controller/rawActionId、切回 kConnected 并触发回调。
  void lose_control(HighLevelError error);

  /// 创建 robotServer 事件订阅。
  void create_event_subscription();

  /// 销毁事件订阅。
  void destroy_event_subscription();

  /// 处理 EventMessage，包含 magic 校验、host.event 解包和 control.status 失权处理。
  void handle_event(const EventMessage & event);

  rclcpp::Node::SharedPtr node_;
  rclcpp::Executor & executor_;
  rclcpp::Subscription<EventMessage>::SharedPtr event_subscription_;
  rclcpp::Publisher<RemoteControl>::SharedPtr raw_control_publisher_;
  rclcpp::TimerBase::SharedPtr renew_timer_;
  std::optional<std::int64_t> pending_renew_request_id_;
  std::optional<std::chrono::steady_clock::time_point> pending_renew_deadline_;
  std::chrono::steady_clock::time_point last_renew_at_;
  std::uint64_t renew_sequence_;
  std::string event_topic_;
  std::string controller_;
  std::uint64_t raw_action_id_;
  std::uint64_t raw_control_seq_;
  int32_t lease_ms_;
  HighLevelState state_;
  mutable HighLevelError last_error_;
  ConnectCallback connect_callback_;
  EventCallback event_callback_;
};

}  // namespace uniubi_interface_test

#endif  // UNIUBI_INTERFACE_TEST__MOTION_HIGH_LEVEL_CLIENT_HPP_
