#include <cstdlib>
#include <memory>
#include <stdexcept>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "uniubi/msg/sensor_observed.hpp"

namespace
{

std::string sensor_observed_topic()
{
  const auto * value = std::getenv("UNIUBI_TEST_SENSOR_OBSERVED_TOPIC");
  if (value != nullptr && value[0] != '\0') {return value;}
  const auto * device_id = std::getenv("UNIUBI_TEST_DEVICE_ID");
  if (device_id == nullptr || device_id[0] == '\0') {
    throw std::invalid_argument("UNIUBI_TEST_DEVICE_ID is required unless UNIUBI_TEST_SENSOR_OBSERVED_TOPIC is set");
  }
  return std::string("/robot/") + device_id + "/sensor/observed";
}

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("uniubi_sensor_observed_subscriber");
  const auto topic = sensor_observed_topic();
  auto qos = rclcpp::QoS(rclcpp::KeepLast(1));
  qos.best_effort();
  qos.durability_volatile();

  auto subscription = node->create_subscription<uniubi::msg::SensorObserved>(
    topic, qos,
    [node](const uniubi::msg::SensorObserved::SharedPtr sensor) {
      const auto & odometry = sensor->odom;
      RCLCPP_INFO(
        node->get_logger(),
        "gps_valid=%u odom_epoch=%u odom_valid=%s position=(%.3f, %.3f) m "
        "velocity=(%.3f, %.3f) m/s yaw=%.3f rad yaw_speed=%.3f rad/s",
        static_cast<unsigned>(sensor->gps.valid),
        odometry.epoch,
        odometry.valid ? "true" : "false",
        odometry.position[0], odometry.position[1],
        odometry.velocity[0], odometry.velocity[1],
        odometry.yaw, odometry.yaw_speed);
    });

  RCLCPP_INFO(node->get_logger(), "subscribing to %s without requesting control", topic.c_str());
  rclcpp::spin(node);
  subscription.reset();
  rclcpp::shutdown();
  return 0;
}
