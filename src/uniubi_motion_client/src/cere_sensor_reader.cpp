#include "uniubi_motion_client/cere_sensor_reader.hpp"

#include <algorithm>
#include <chrono>
#include <limits>
#include <stdexcept>
#include <utility>

#include "dds/dds.h"
#include "BrainMotionState.h"

namespace uniubi_motion_client {
namespace {

template<typename Source, typename Target>
void copy_vector(const Source & in, Target & out) {
  out.error = in.error;
  out.x = in.x; out.y = in.y; out.z = in.z;
}

void copy_imu(const uniubi_dds__IMUState & in, uniubi::msg::IMUState & out) {
  out.temp = in.temp;
  copy_vector(in.accel, out.accel);
  copy_vector(in.gyro, out.gyro);
  copy_vector(in.mag, out.mag);
  copy_vector(in.euler, out.euler);
  out.quaternion.error = in.quaternion.error;
  out.quaternion.w = in.quaternion.w;
  out.quaternion.x = in.quaternion.x;
  out.quaternion.y = in.quaternion.y;
  out.quaternion.z = in.quaternion.z;
}

void copy_motor(const uniubi_dds__MotorObserved & in, uniubi::msg::MotorObserved & out) {
  out.enable = in.enable; out.online = in.online; out.error = in.error;
  out.position = in.position; out.velocity = in.velocity; out.torque = in.torque;
  out.temp = in.temp; out.voltage = in.voltage;
  out.loss_rate = in.lossRate; out.max_torque = in.maxTorque;
  out.header.limbs_no = in.header.limbsNo;
  out.header.joint_no = in.header.jointNo;
}

}  // namespace

struct CereSensorReader::Impl {
  dds_entity_t participant = 0;
  dds_entity_t reader = 0;
  rclcpp::TimerBase::SharedPtr timer;
  std::function<void(const uniubi::msg::SensorObserved &)> sensor_callback;
  std::function<void(const uniubi::msg::MotionObserved &)> motion_callback;

  ~Impl() {timer.reset(); if (participant > 0) {dds_delete(participant);}}

  void poll() {
    // One latest sample per tick; do not let observations starve RPC processing.
    void * samples[1] = {nullptr};
    dds_sample_info_t info[1];
    const auto count = dds_take(reader, samples, info, 1, 1);
    if (count <= 0) {return;}
    if (!info[0].valid_data) {dds_return_loan(reader, samples, count); return;}
    const auto & in = *static_cast<const uniubi_dds__BrainMotionState *>(samples[0]);
    const bool timestamp_valid = in.timestamp <= std::numeric_limits<uint64_t>::max() / 1000;
    const bool sensor_valid = timestamp_valid && in.hasSensor && static_cast<bool>(sensor_callback);
    const bool motion_valid = timestamp_valid && in.hasMotion && static_cast<bool>(motion_callback) &&
      in.motorNum >= 0 && in.motorNum <= static_cast<int32_t>(sizeof(in.motor) / sizeof(in.motor[0]));
    uniubi::msg::SensorObserved sensor;
    uniubi::msg::MotionObserved motion;
    if (sensor_valid) {
      // Internal BrainMotionState is milliseconds; public observations use microseconds.
      sensor.timestamp = in.timestamp * 1000;
      sensor.gps.valid = in.gps.valid; sensor.gps.speed = in.gps.speed;
      sensor.gps.level = in.gps.level; sensor.gps.rssi = in.gps.rssi;
      sensor.gps.point.lat = in.gps.point.lat; sensor.gps.point.lng = in.gps.point.lng;
      sensor.uwb.valid = in.uwb.valid; sensor.uwb.pair_state = in.uwb.pairState;
      sensor.uwb.rssi = in.uwb.rssi; sensor.uwb.pitch = in.uwb.pitch;
      sensor.uwb.azimuth = in.uwb.azimuth; sensor.uwb.distance = in.uwb.distance;
      sensor.uwb.beacon_id = in.uwb.beaconId;
      sensor.odom.valid = in.odom.valid; sensor.odom.epoch = in.odom.epoch;
      sensor.odom.yaw = in.odom.yaw; sensor.odom.yaw_speed = in.odom.yawSpeed;
      std::copy_n(in.odom.position, 3, sensor.odom.position.begin());
      std::copy_n(in.odom.velocity, 3, sensor.odom.velocity.begin());
    }
    if (motion_valid) {
      motion.timestamp = in.timestamp * 1000;
      motion.motor_num = in.motorNum;
      copy_imu(in.imu, motion.imu);
      for (int32_t i = 0; i < in.motorNum; ++i) {
        copy_motor(in.motor[i], motion.motor[static_cast<std::size_t>(i)]);
      }
      // BrainMotionState has no PowerObserved. The bridge gets battery from getSystemStatus.
    }
    dds_return_loan(reader, samples, count);
    if (sensor_valid) {sensor_callback(sensor);}
    if (motion_valid) {motion_callback(motion);}
  }
};

CereSensorReader::CereSensorReader(const rclcpp::Node::SharedPtr & node,
  const std::string & topic,
  std::function<void(const uniubi::msg::SensorObserved &)> sensor_callback,
  std::function<void(const uniubi::msg::MotionObserved &)> motion_callback)
: impl_(std::make_unique<Impl>()) {
  if (topic.empty()) {throw std::invalid_argument("cere_motion_topic must not be empty");}
  impl_->sensor_callback = std::move(sensor_callback);
  impl_->motion_callback = std::move(motion_callback);
  impl_->participant = dds_create_participant(node->get_node_base_interface()->get_context()->get_domain_id(), nullptr, nullptr);
  if (impl_->participant < 0) {throw std::runtime_error("Failed to create cere DDS participant");}
  const auto t = dds_create_topic(impl_->participant, &uniubi_dds__BrainMotionState_desc, topic.c_str(), nullptr, nullptr);
  if (t < 0) {throw std::runtime_error("Failed to create cere DDS topic");}
  auto * qos = dds_create_qos();
  dds_qset_reliability(qos, DDS_RELIABILITY_BEST_EFFORT, 0);
  dds_qset_durability(qos, DDS_DURABILITY_VOLATILE);
  dds_qset_history(qos, DDS_HISTORY_KEEP_LAST, 1);
  impl_->reader = dds_create_reader(impl_->participant, t, qos, nullptr);
  dds_delete_qos(qos);
  if (impl_->reader < 0) {throw std::runtime_error("Failed to create cere DDS reader");}
  impl_->timer = node->create_wall_timer(std::chrono::milliseconds(5), [this]() {impl_->poll();});
}
CereSensorReader::~CereSensorReader() = default;
}
