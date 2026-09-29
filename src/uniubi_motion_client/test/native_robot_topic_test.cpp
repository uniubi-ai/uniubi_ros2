#include <chrono>
#include <stdexcept>
#include <thread>
#include "dds/dds.h"
#include "MotionObserved.h"
#include "SensorObserved.h"
#include "rclcpp/rclcpp.hpp"
#include "uniubi/msg/motion_observed.hpp"
#include "uniubi/msg/sensor_observed.hpp"

using namespace std::chrono_literals;
static void check(bool ok) { if (!ok) throw std::runtime_error("native robot topic isolation failed"); }

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("native_robot_topic_test");
  auto qos = rclcpp::QoS(1).best_effort().durability_volatile();
  int motion_a = 0, motion_b = 0, sensor_a = 0, sensor_b = 0;
  auto ma = node->create_subscription<uniubi::msg::MotionObserved>(
    "/robot/dog57/motion/observed", qos, [&](const uniubi::msg::MotionObserved::SharedPtr m) { if (m->timestamp == 57) ++motion_a; });
  auto mb = node->create_subscription<uniubi::msg::MotionObserved>(
    "/robot/dog58/motion/observed", qos, [&](const uniubi::msg::MotionObserved::SharedPtr m) { if (m->timestamp == 58) ++motion_b; });
  auto sa = node->create_subscription<uniubi::msg::SensorObserved>(
    "/robot/dog57/sensor/observed", qos, [&](const uniubi::msg::SensorObserved::SharedPtr m) { if (m->timestamp == 57) ++sensor_a; });
  auto sb = node->create_subscription<uniubi::msg::SensorObserved>(
    "/robot/dog58/sensor/observed", qos, [&](const uniubi::msg::SensorObserved::SharedPtr m) { if (m->timestamp == 58) ++sensor_b; });

  const auto domain = node->get_node_base_interface()->get_context()->get_domain_id();
  const auto participant = dds_create_participant(domain, nullptr, nullptr);
  check(participant > 0);
  const auto mta = dds_create_topic(participant, &uniubi_msg_dds__MotionObserved__desc,
    "rt/robot/dog57/motion/observed", nullptr, nullptr);
  const auto mtb = dds_create_topic(participant, &uniubi_msg_dds__MotionObserved__desc,
    "rt/robot/dog58/motion/observed", nullptr, nullptr);
  const auto sta = dds_create_topic(participant, &uniubi_msg_dds__SensorObserved__desc,
    "rt/robot/dog57/sensor/observed", nullptr, nullptr);
  const auto stb = dds_create_topic(participant, &uniubi_msg_dds__SensorObserved__desc,
    "rt/robot/dog58/sensor/observed", nullptr, nullptr);
  check(mta > 0 && mtb > 0 && sta > 0 && stb > 0);
  auto * q = dds_create_qos();
  dds_qset_reliability(q, DDS_RELIABILITY_BEST_EFFORT, 0);
  const auto mwa = dds_create_writer(participant, mta, q, nullptr);
  const auto mwb = dds_create_writer(participant, mtb, q, nullptr);
  const auto swa = dds_create_writer(participant, sta, q, nullptr);
  const auto swb = dds_create_writer(participant, stb, q, nullptr);
  dds_delete_qos(q);
  check(mwa > 0 && mwb > 0 && swa > 0 && swb > 0);
  auto spin = [&]() { rclcpp::spin_some(node); std::this_thread::sleep_for(10ms); };
  for (int i = 0; i < 300; ++i) {
    if (dds_get_matched_subscriptions(mwa, nullptr, 0) == 1 &&
        dds_get_matched_subscriptions(mwb, nullptr, 0) == 1 &&
        dds_get_matched_subscriptions(swa, nullptr, 0) == 1 &&
        dds_get_matched_subscriptions(swb, nullptr, 0) == 1) break;
    spin();
  }
  check(dds_get_matched_subscriptions(mwa, nullptr, 0) == 1);
  check(dds_get_matched_subscriptions(mwb, nullptr, 0) == 1);
  check(dds_get_matched_subscriptions(swa, nullptr, 0) == 1);
  check(dds_get_matched_subscriptions(swb, nullptr, 0) == 1);
  uniubi_msg_dds__MotionObserved_ motion{};
  uniubi_msg_dds__SensorObserved_ sensor{};
  motion.timestamp = sensor.timestamp = 57;
  check(dds_write(mwa, &motion) == 0 && dds_write(swa, &sensor) == 0);
  motion.timestamp = sensor.timestamp = 58;
  check(dds_write(mwb, &motion) == 0 && dds_write(swb, &sensor) == 0);
  for (int i = 0; i < 200 && (motion_a < 1 || motion_b < 1 || sensor_a < 1 || sensor_b < 1); ++i) spin();
  check(motion_a == 1 && motion_b == 1 && sensor_a == 1 && sensor_b == 1);
  dds_delete(participant);
  ma.reset(); mb.reset(); sa.reset(); sb.reset(); node.reset(); rclcpp::shutdown();
}
