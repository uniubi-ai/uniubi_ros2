#include <cstdlib>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "uniubi/msg/motion_odometry.hpp"

namespace
{

constexpr const char * kDefaultOdometryTopic = "/motion/odometry";

std::string odometry_topic()
{
  const auto * value = std::getenv("UNIUBI_TEST_ODOMETRY_TOPIC");
  return value == nullptr || value[0] == '\0' ? kDefaultOdometryTopic : value;
}

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("uniubi_motion_odometry_subscriber");
  const auto topic = odometry_topic();
  auto qos = rclcpp::QoS(rclcpp::KeepLast(1));
  qos.best_effort();
  qos.durability_volatile();

  auto subscription = node->create_subscription<uniubi::msg::MotionOdometry>(
    topic, qos,
    [node](const uniubi::msg::MotionOdometry::SharedPtr odometry) {
      RCLCPP_INFO(
        node->get_logger(),
        "epoch=%u valid=%s position=(%.3f, %.3f) m velocity=(%.3f, %.3f) m/s "
        "yaw=%.3f rad yaw_speed=%.3f rad/s timestamp_us=%llu",
        odometry->epoch,
        odometry->valid ? "true" : "false",
        odometry->position[0], odometry->position[1],
        odometry->velocity[0], odometry->velocity[1],
        odometry->yaw, odometry->yaw_speed,
        static_cast<unsigned long long>(odometry->timestamp_us));
    });

  RCLCPP_INFO(node->get_logger(), "subscribing to %s without requesting control", topic.c_str());
  rclcpp::spin(node);
  subscription.reset();
  rclcpp::shutdown();
  return 0;
}
