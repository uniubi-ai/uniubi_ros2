#ifndef UNIUBI_INTERFACE_TEST__SYSTEM_RPC_CLIENT_BASE_HPP_
#define UNIUBI_INTERFACE_TEST__SYSTEM_RPC_CLIENT_BASE_HPP_

#include <chrono>
#include <cstdint>
#include <string>

#include "json/json.h"
#include "rclcpp/rclcpp.hpp"
#include "uniubi/srv/system.hpp"

namespace uniubi_interface_test
{

struct SystemRpcRequest
{
  SystemRpcRequest();

  SystemRpcRequest(
    std::uint64_t timestamp,
    const std::string & service,
    const std::string & device_id,
    const std::string & method,
    const std::string & payload);

  std::uint64_t timestamp;
  std::string service;
  std::string device_id;
  std::string method;
  std::string payload;
};

struct SystemRpcCall
{
  SystemRpcCall();

  SystemRpcCall(
    const std::string & service,
    const std::string & method,
    const Json::Value & params);

  SystemRpcCall(
    const std::string & service,
    const std::string & method,
    const Json::Value & params,
    const std::string & client_id,
    const std::string & device_id,
    std::uint64_t timestamp);

  std::string service;
  std::string method;
  Json::Value params;
  std::string client_id;
  std::string device_id;
  std::uint64_t timestamp;
};

class SystemRpcClientBase
{
public:
  using Service = uniubi::srv::System;
  using Request = Service::Request;
  using Response = Service::Response;
  using Client = rclcpp::Client<Service>;
  using FutureAndRequestId = Client::FutureAndRequestId;
  using SharedFuture = Client::SharedFuture;
  using SharedFutureAndRequestId = Client::SharedFutureAndRequestId;
  using SharedResponse = Client::SharedResponse;
  using ResponseCallback = Client::CallbackType;

  SystemRpcClientBase(
    const rclcpp::Node::SharedPtr & node,
    const std::string & ros_service_name,
    const std::string & device_id = "");

  virtual ~SystemRpcClientBase() = default;

  bool wait_for_service(std::chrono::nanoseconds timeout);

  FutureAndRequestId async_call(const SystemRpcCall & rpc_call);

  SharedFutureAndRequestId async_call(
    const SystemRpcCall & rpc_call,
    ResponseCallback callback);

  SharedResponse call(
    const SystemRpcCall & rpc_call,
    rclcpp::Executor & executor,
    std::chrono::nanoseconds timeout);

  FutureAndRequestId async_send_request(const SystemRpcRequest & rpc_request);

  FutureAndRequestId async_send_request(const Request & request);

  SharedFutureAndRequestId async_send_request(
    const SystemRpcRequest & rpc_request,
    ResponseCallback callback);

  SharedFutureAndRequestId async_send_request(
    const Request & request,
    ResponseCallback callback);

  bool remove_pending_request(std::int64_t request_id);

  SharedResponse send_request(
    const SystemRpcRequest & rpc_request,
    rclcpp::Executor & executor,
    std::chrono::nanoseconds timeout);

  SharedResponse send_request(
    const Request & request,
    rclcpp::Executor & executor,
    std::chrono::nanoseconds timeout);

  const std::string & ros_service_name() const;

  const std::string & device_id() const;

  void set_device_id(const std::string & device_id);

  const std::string & client_id() const;

  static Request make_request(const SystemRpcRequest & rpc_request);

  Request make_request(const SystemRpcCall & rpc_call) const;

  SystemRpcRequest make_rpc_request(const SystemRpcCall & rpc_call) const;

  static std::string make_payload(
    const std::string & client_id,
    const Json::Value & params);

  static std::uint64_t now_ms();

protected:
  SystemRpcCall make_call(
    const std::string & service,
    const std::string & method,
    const Json::Value & params,
    const std::string & client_id) const;

private:
  static std::string make_client_id(const rclcpp::Node::SharedPtr & node);

  rclcpp::Node::SharedPtr node_;
  Client::SharedPtr client_;
  std::string ros_service_name_;
  std::string device_id_;
  std::string client_id_;
};

}  // namespace uniubi_interface_test

#endif  // UNIUBI_INTERFACE_TEST__SYSTEM_RPC_CLIENT_BASE_HPP_
