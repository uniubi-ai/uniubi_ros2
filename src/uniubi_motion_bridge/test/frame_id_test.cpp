#include <cassert>
#include "uniubi_motion_bridge/frame_id.hpp"

int main()
{
  using uniubi_motion_bridge::prefixed_frame_id;
  assert(prefixed_frame_id("", "odom") == "odom");
  assert(prefixed_frame_id("dog57", "odom") == "dog57/odom");
  assert(prefixed_frame_id("dog57", "base_link") == "dog57/base_link");
  assert(prefixed_frame_id("dog57/", "imu_link") == "dog57/imu_link");
}
