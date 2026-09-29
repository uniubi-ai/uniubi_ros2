#include <chrono>
#include <limits>
#include <stdexcept>
#include <thread>

#include "dds/dds.h"
#include "BrainMotionState.h"
#include "uniubi_motion_client/cere_sensor_reader.hpp"

static void require(bool ok) {if (!ok) throw std::runtime_error("cere reader assertion failed");}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("cere_sensor_reader_test");
  int sensor_count = 0, motion_count = 0;
  uniubi::msg::SensorObserved sensor;
  uniubi::msg::MotionObserved motion;
  auto reader = std::make_unique<uniubi_motion_client::CereSensorReader>(
    node, "rt/cere/testSensor",
    [&](const uniubi::msg::SensorObserved & value) {++sensor_count; sensor = value;},
    [&](const uniubi::msg::MotionObserved & value) {++motion_count; motion = value;});
  const auto domain = node->get_node_base_interface()->get_context()->get_domain_id();
  const auto participant = dds_create_participant(domain, nullptr, nullptr); require(participant > 0);
  const auto topic = dds_create_topic(participant, &uniubi_dds__BrainMotionState_desc,
    "rt/cere/testSensor", nullptr, nullptr); require(topic > 0);
  auto * q = dds_create_qos(); dds_qset_reliability(q, DDS_RELIABILITY_BEST_EFFORT, 0);
  const auto writer = dds_create_writer(participant, topic, q, nullptr);
  dds_delete_qos(q); require(writer > 0);
  auto spin = [&](int ms) {
    auto end = std::chrono::steady_clock::now() + std::chrono::milliseconds(ms);
    while (std::chrono::steady_clock::now() < end) {
      rclcpp::spin_some(node); std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
  };
  for (int i = 0; i < 50 && dds_get_matched_subscriptions(writer, nullptr, 0) == 0; ++i) spin(100);
  require(dds_get_matched_subscriptions(writer, nullptr, 0) > 0);
  uniubi_dds__BrainMotionState sample{};
  sample.timestamp = 123456;
  require(dds_write(writer, &sample) == 0); spin(100);
  require(sensor_count == 0 && motion_count == 0);

  // Sensor-only frames must not require hasMotion.
  sample.hasSensor = 1;
  sample.gps.valid = 1; sample.gps.point.lat = 30.25; sample.gps.point.lng = 120.5;
  sample.gps.speed = 2.5; sample.gps.level = 2; sample.gps.rssi = -30;
  sample.uwb.valid = 1; sample.uwb.beaconId = 1234; sample.uwb.pairState = 2;
  sample.uwb.rssi = -40; sample.uwb.pitch = 12; sample.uwb.azimuth = 25;
  sample.uwb.distance = 375;
  sample.odom.valid = 1; sample.odom.epoch = 7; sample.odom.yaw = .5;
  sample.odom.yawSpeed = .25;
  for (int i = 0; i < 3; ++i) {
    sample.odom.position[i] = i + 1; sample.odom.velocity[i] = i + .5;
  }
  require(dds_write(writer, &sample) == 0); spin(100);
  require(sensor_count == 1 && motion_count == 0 && sensor.timestamp == 123456000);
  require(sensor.uwb.beacon_id == 1234 && sensor.gps.point.lat == 30.25);
  require(sensor.odom.epoch == 7 && sensor.odom.position[2] == 3);

  // Motion-only frames independently carry IMU and bounded motor array.
  sample.hasSensor = 0; sample.hasMotion = 1; sample.motorNum = 2;
  sample.imu.accel.x = 1.25F; sample.imu.gyro.z = -.75F;
  sample.imu.quaternion.w = .5F; sample.imu.quaternion.x = .25F;
  sample.motor[0].header.limbsNo = 1; sample.motor[0].header.jointNo = 2;
  sample.motor[0].position = 3.5F; sample.motor[0].velocity = 4.5F;
  sample.motor[0].torque = 5.5F; sample.motor[0].lossRate = 6.5F;
  sample.motor[1].header.limbsNo = 3; sample.motor[1].header.jointNo = 4;
  require(dds_write(writer, &sample) == 0); spin(100);
  require(sensor_count == 1 && motion_count == 1 && motion.timestamp == 123456000);
  require(motion.motor_num == 2 && motion.imu.accel.x == 1.25F && motion.imu.gyro.z == -.75F);
  require(motion.imu.quaternion.w == .5F && motion.motor[0].header.limbs_no == 1);
  require(motion.motor[0].header.joint_no == 2 && motion.motor[0].position == 3.5F);
  require(motion.motor[0].velocity == 4.5F && motion.motor[0].torque == 5.5F);
  require(motion.motor[0].loss_rate == 6.5F && motion.motor[1].header.limbs_no == 3);
  require(motion.power.charge_voltage == 0.0F);  // BrainMotionState has no battery.

  // Malformed motion length is discarded without suppressing a valid sensor frame.
  sample.hasSensor = 1; sample.motorNum = -1;
  require(dds_write(writer, &sample) == 0); spin(100);
  require(sensor_count == 2 && motion_count == 1);
  sample.motorNum = 17;
  require(dds_write(writer, &sample) == 0); spin(100);
  require(sensor_count == 3 && motion_count == 1);
  sample.motorNum = 16;
  require(dds_write(writer, &sample) == 0); spin(100);
  require(sensor_count == 4 && motion_count == 2 && motion.motor_num == 16);
  sample.timestamp = std::numeric_limits<uint64_t>::max();
  require(dds_write(writer, &sample) == 0); spin(100);
  require(sensor_count == 4 && motion_count == 2);

  reader.reset();
  sample.timestamp = 123456;
  require(dds_write(writer, &sample) == 0); spin(100);
  require(sensor_count == 4 && motion_count == 2);
  dds_delete(participant); node.reset(); rclcpp::shutdown();
}
