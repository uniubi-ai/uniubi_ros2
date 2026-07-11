#include "uniubi_interface_test/system_rpc_client_base.hpp"

#include <atomic>
#include <chrono>
#include <iomanip>
#include <memory>
#include <sstream>
#include <stdexcept>

#include "json/json.h"

namespace uniubi_interface_test
{

namespace
{

std::atomic<std::uint64_t> g_client_sequence{1};

Json::Value empty_object_if_null(const Json::Value & value)
{
  return value.isNull() ? Json::Value(Json::objectValue) : value;
}

std::string write_json(const Json::Value & value)
{
  Json::StreamWriterBuilder builder;
  builder["indentation"] = "";
  return Json::writeString(builder, value);
}

}  // namespace

SystemRpcRequest::SystemRpcRequest()
: timestamp(0)
{
}

SystemRpcRequest::SystemRpcRequest(
  std::uint64_t timestamp,
  const std::string & service,
  const std::string & device_id,
  const std::string & method,
  const std::string & payload)
: timestamp(timestamp),
  service(service),
  device_id(device_id),
  method(method),
  payload(payload)
{
}

SystemRpcCall::SystemRpcCall()
: params(Json::objectValue),
  timestamp(0)
{
}

SystemRpcCall::SystemRpcCall(
  const std::string & service,
  const std::string & method,
  const Json::Value & params)
: service(service),
  method(method),
  params(empty_object_if_null(params)),
  timestamp(0)
{
}

SystemRpcCall::SystemRpcCall(
  const std::string & service,
  const std::string & method,
  const Json::Value & params,
  const std::string & client_id,
  const std::string & device_id,
  std::uint64_t timestamp)
: service(service),
  method(method),
  params(empty_object_if_null(params)),
  client_id(client_id),
  device_id(device_id),
  timestamp(timestamp)
{
}

SystemRpcClientBase::SystemRpcClientBase(
  const rclcpp::Node::SharedPtr & node,
  const std::string & ros_service_name,
  const std::string & device_id)
: node_(node),
  ros_service_name_(ros_service_name),
  device_id_(device_id),
  client_id_(make_client_id(node))
{
  if (!node_) {
    throw std::invalid_argument("SystemRpcClientBase requires a valid rclcpp node");
  }

  client_ = node_->create_client<Service>(ros_service_name_);
}

bool SystemRpcClientBase::wait_for_service(std::chrono::nanoseconds timeout)
{
  return client_->wait_for_service(timeout);
}

SystemRpcClientBase::FutureAndRequestId SystemRpcClientBase::async_call(
  const SystemRpcCall & rpc_call)
{
  return async_send_request(make_rpc_request(rpc_call));
}

SystemRpcClientBase::SharedFutureAndRequestId SystemRpcClientBase::async_call(
  const SystemRpcCall & rpc_call,
  ResponseCallback callback)
{
  return async_send_request(make_rpc_request(rpc_call), std::move(callback));
}

SystemRpcClientBase::SharedResponse SystemRpcClientBase::call(
  const SystemRpcCall & rpc_call,
  rclcpp::Executor & executor,
  std::chrono::nanoseconds timeout)
{
  return send_request(make_rpc_request(rpc_call), executor, timeout);
}

SystemRpcClientBase::FutureAndRequestId SystemRpcClientBase::async_send_request(
  const SystemRpcRequest & rpc_request)
{
  return async_send_request(make_request(rpc_request));
}

SystemRpcClientBase::FutureAndRequestId SystemRpcClientBase::async_send_request(
  const Request & request)
{
  auto request_ptr = std::make_shared<Request>(request);
  return client_->async_send_request(request_ptr);
}

SystemRpcClientBase::SharedFutureAndRequestId SystemRpcClientBase::async_send_request(
  const SystemRpcRequest & rpc_request,
  ResponseCallback callback)
{
  return async_send_request(make_request(rpc_request), std::move(callback));
}

SystemRpcClientBase::SharedFutureAndRequestId SystemRpcClientBase::async_send_request(
  const Request & request,
  ResponseCallback callback)
{
  auto request_ptr = std::make_shared<Request>(request);
  return client_->async_send_request(request_ptr, std::move(callback));
}

bool SystemRpcClientBase::remove_pending_request(std::int64_t request_id)
{
  return client_->remove_pending_request(request_id);
}

SystemRpcClientBase::SharedResponse SystemRpcClientBase::send_request(
  const SystemRpcRequest & rpc_request,
  rclcpp::Executor & executor,
  std::chrono::nanoseconds timeout)
{
  return send_request(make_request(rpc_request), executor, timeout);
}

SystemRpcClientBase::SharedResponse SystemRpcClientBase::send_request(
  const Request & request,
  rclcpp::Executor & executor,
  std::chrono::nanoseconds timeout)
{
  auto future = async_send_request(request);
  const auto result = executor.spin_until_future_complete(future, timeout);
  if (result == rclcpp::FutureReturnCode::SUCCESS) {
    return future.get();
  }

  client_->remove_pending_request(future);
  if (result == rclcpp::FutureReturnCode::TIMEOUT) {
    throw std::runtime_error("System RPC request timed out");
  }
  throw std::runtime_error("System RPC request was interrupted");
}

const std::string & SystemRpcClientBase::ros_service_name() const
{
  return ros_service_name_;
}

const std::string & SystemRpcClientBase::device_id() const
{
  return device_id_;
}

void SystemRpcClientBase::set_device_id(const std::string & device_id)
{
  device_id_ = device_id;
}

const std::string & SystemRpcClientBase::client_id() const
{
  return client_id_;
}

SystemRpcClientBase::Request SystemRpcClientBase::make_request(
  const SystemRpcRequest & rpc_request)
{
  Request request;
  request.timestamp = rpc_request.timestamp;
  request.service = rpc_request.service;
  request.device_id = rpc_request.device_id;
  request.method = rpc_request.method;
  request.payload = rpc_request.payload;
  return request;
}

SystemRpcClientBase::Request SystemRpcClientBase::make_request(
  const SystemRpcCall & rpc_call)
  const
{
  return make_request(make_rpc_request(rpc_call));
}

SystemRpcRequest SystemRpcClientBase::make_rpc_request(const SystemRpcCall & rpc_call)
  const
{
  const auto timestamp = rpc_call.timestamp == 0 ? now_ms() : rpc_call.timestamp;
  const auto & call_client_id = rpc_call.client_id.empty() ? client_id_ : rpc_call.client_id;
  return SystemRpcRequest(
    timestamp,
    rpc_call.service,
    rpc_call.device_id,
    rpc_call.method,
    make_payload(call_client_id, rpc_call.params));
}

std::string SystemRpcClientBase::make_payload(
  const std::string & client_id,
  const Json::Value & params)
{
  Json::Value payload(Json::objectValue);
  payload["call"]["clientId"] = client_id;
  payload["params"] = empty_object_if_null(params);
  return write_json(payload);
}

std::uint64_t SystemRpcClientBase::now_ms()
{
  const auto now = std::chrono::system_clock::now().time_since_epoch();
  return static_cast<std::uint64_t>(
    std::chrono::duration_cast<std::chrono::milliseconds>(now).count());
}

SystemRpcCall SystemRpcClientBase::make_call(
  const std::string & service,
  const std::string & method,
  const Json::Value & params,
  const std::string & client_id) const
{
  return SystemRpcCall(service, method, params, client_id, device_id_, now_ms());
}

std::string SystemRpcClientBase::make_client_id(const rclcpp::Node::SharedPtr & node)
{
  if (!node) {
    return {};
  }

  const auto now = std::chrono::system_clock::now().time_since_epoch();
  const auto now_ms =
    static_cast<std::uint64_t>(std::chrono::duration_cast<std::chrono::milliseconds>(now).count());
  const auto sequence = g_client_sequence.fetch_add(1, std::memory_order_relaxed);

  std::ostringstream stream;
  stream << "ros2-client-" << node->get_name() << '-' << std::hex << now_ms << '-' << sequence;
  return stream.str();
}

}  // namespace uniubi_interface_test
