#pragma once

#include <string>

namespace uniubi_motion_bridge
{

inline std::string prefixed_frame_id(std::string prefix, const std::string & frame)
{
  while (!prefix.empty() && prefix.back() == '/') {
    prefix.pop_back();
  }
  return prefix.empty() ? frame : prefix + "/" + frame;
}

}  // namespace uniubi_motion_bridge
