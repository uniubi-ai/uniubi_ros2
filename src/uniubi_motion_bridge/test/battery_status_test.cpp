#include <cmath>
#include <stdexcept>

#include "uniubi_motion_bridge/battery_status.hpp"

static void require(bool ok) {if (!ok) throw std::runtime_error("battery status assertion failed");}

int main() {
  sensor_msgs::msg::BatteryState output;
  Json::Value status(Json::objectValue);
  require(!uniubi_motion_bridge::battery_from_system_status(status, output));
  auto & battery = status["battery"];
  battery["online"] = true;
  battery["voltage"] = 49.5;
  battery["current"] = -2.25;
  battery["temperature"] = 31.5;
  battery["power"] = 75.0;
  require(uniubi_motion_bridge::battery_from_system_status(status, output));
  require(output.present && output.voltage == 49.5F && output.percentage == .75F);
  require(output.current == -2.25F && output.temperature == 31.5F);
  require(std::isnan(output.charge) && std::isnan(output.capacity));
  battery["online"] = false;
  require(!uniubi_motion_bridge::battery_from_system_status(status, output));
  battery["online"] = true;
  battery["power"] = 101.0;
  require(!uniubi_motion_bridge::battery_from_system_status(status, output));
  battery["power"] = 0.0;
  battery["voltage"] = 0.0;
  require(!uniubi_motion_bridge::battery_from_system_status(status, output));
  battery["voltage"] = 50.0;
  battery.removeMember("current");
  require(uniubi_motion_bridge::battery_from_system_status(status, output));
  require(std::isnan(output.current) && output.percentage == 0.0F);
}
