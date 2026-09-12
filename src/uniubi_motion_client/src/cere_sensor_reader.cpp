#include "uniubi_motion_client/cere_sensor_reader.hpp"
#include <algorithm>
#include <limits>
#include <stdexcept>
#include "dds/dds.h"
#include "BrainMotionState.h"

namespace uniubi_motion_client {
struct CereSensorReader::Impl {
  dds_entity_t participant = 0;
  dds_entity_t reader = 0;
  rclcpp::TimerBase::SharedPtr timer;
  std::function<void(const uniubi::msg::SensorObserved &)> callback;
  ~Impl() {timer.reset(); if (participant > 0) {dds_delete(participant);}}
  void poll() {
    // One latest sample per tick; do not let observations starve RPC processing.
    void * samples[1] = {nullptr};
    dds_sample_info_t info[1];
    const auto count = dds_take(reader, samples, info, 1, 1);
    if (count <= 0) {return;}
    if (!info[0].valid_data) {dds_return_loan(reader, samples, count); return;}
    const auto & in = *static_cast<const uniubi_dds__BrainMotionState *>(samples[0]);
    uniubi::msg::SensorObserved out;
    const bool valid = info[0].valid_data && in.hasSensor &&
      in.timestamp <= std::numeric_limits<uint64_t>::max() / 1000;
    if (valid) {
      // Internal BrainMotionState is milliseconds; SensorObserved is microseconds.
      out.timestamp = in.timestamp * 1000;
      out.gps.valid = in.gps.valid; out.gps.speed = in.gps.speed;
      out.gps.level = in.gps.level; out.gps.rssi = in.gps.rssi;
      out.gps.point.lat = in.gps.point.lat; out.gps.point.lng = in.gps.point.lng;
      out.uwb.valid = in.uwb.valid; out.uwb.pair_state = in.uwb.pairState;
      out.uwb.rssi = in.uwb.rssi; out.uwb.pitch = in.uwb.pitch;
      out.uwb.azimuth = in.uwb.azimuth; out.uwb.distance = in.uwb.distance;
      out.uwb.beacon_id = in.uwb.beaconId;
      out.odom.valid = in.odom.valid; out.odom.epoch = in.odom.epoch;
      out.odom.yaw = in.odom.yaw; out.odom.yaw_speed = in.odom.yawSpeed;
      std::copy_n(in.odom.position, 3, out.odom.position.begin());
      std::copy_n(in.odom.velocity, 3, out.odom.velocity.begin());
    }
    dds_return_loan(reader, samples, count);
    if (valid) {callback(out);}
  }
};
CereSensorReader::CereSensorReader(const rclcpp::Node::SharedPtr & node,
  const std::string & topic, std::function<void(const uniubi::msg::SensorObserved &)> callback)
: impl_(std::make_unique<Impl>()) {
  if (topic.empty()) {throw std::invalid_argument("cere_motion_topic must not be empty");}
  impl_->callback = std::move(callback);
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
