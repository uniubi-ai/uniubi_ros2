#pragma once
#include <functional>
#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "uniubi/msg/sensor_observed.hpp"

namespace uniubi_motion_client {
// Reads the existing internal DDS type without changing its wire name or layout.
class CereSensorReader {
public:
  CereSensorReader(const rclcpp::Node::SharedPtr & node, const std::string & topic,
    std::function<void(const uniubi::msg::SensorObserved &)> callback);
  ~CereSensorReader();
  CereSensorReader(const CereSensorReader &) = delete;
  CereSensorReader & operator=(const CereSensorReader &) = delete;
private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};
}
