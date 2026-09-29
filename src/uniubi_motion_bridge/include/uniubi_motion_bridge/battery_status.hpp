#pragma once

#include <algorithm>
#include <cmath>
#include <limits>

#include "json/json.h"
#include "sensor_msgs/msg/battery_state.hpp"

namespace uniubi_motion_bridge {

// getSystemStatus returns a device-status object containing BMS measurements.
// Do not turn a missing or offline battery into a plausible zero-valued sample.
inline bool battery_from_system_status(
  const Json::Value & status, sensor_msgs::msg::BatteryState & output)
{
  const auto & battery = status["battery"];
  if (!battery.isObject() || (battery.isMember("online") &&
    (!battery["online"].isBool() || !battery["online"].asBool())) ||
    !battery["voltage"].isNumeric() || !battery["power"].isNumeric())
  {
    return false;
  }
  const auto voltage = battery["voltage"].asFloat();
  const auto power = battery["power"].asFloat();
  if (!std::isfinite(voltage) || voltage <= 0.0F ||
    !std::isfinite(power) || power < 0.0F || power > 100.0F)
  {
    return false;
  }
  const auto nan = std::numeric_limits<float>::quiet_NaN();
  const auto optional_number = [&battery, nan](const char * name) {
    if (!battery[name].isNumeric()) {return nan;}
    const float value = battery[name].asFloat();
    return std::isfinite(value) ? value : nan;
  };
  output.voltage = voltage;
  output.percentage = power / 100.0F;
  output.current = optional_number("current");
  output.temperature = optional_number("temperature");
  output.charge = nan;
  output.capacity = nan;
  output.design_capacity = nan;
  output.power_supply_status = sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_UNKNOWN;
  output.power_supply_health = sensor_msgs::msg::BatteryState::POWER_SUPPLY_HEALTH_UNKNOWN;
  output.power_supply_technology = sensor_msgs::msg::BatteryState::POWER_SUPPLY_TECHNOLOGY_UNKNOWN;
  output.present = true;
  return true;
}

}  // namespace uniubi_motion_bridge
