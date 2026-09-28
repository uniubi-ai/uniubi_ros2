#pragma once

#include <cctype>
#include <set>
#include <stdexcept>
#include <string>
#include <vector>

namespace uniubi_rtsp_driver {

inline bool valid_name(const std::string & value)
{
  if (value.empty() || !std::isalpha(static_cast<unsigned char>(value.front()))) return false;
  for (unsigned char c : value) {
    if (!std::isalnum(c) && c != '_') return false;
  }
  return true;
}

inline std::string image_topic(const std::string & prefix, const std::string & camera)
{
  std::string result = prefix;
  while (!result.empty() && result.back() == '/') result.pop_back();
  if (result.empty()) return camera + "/image_raw";
  return result + "/" + camera + "/image_raw";
}

inline void validate_config(
  const std::vector<std::string> & urls, const std::vector<std::string> & names,
  const std::vector<std::string> & frame_ids, const std::string & topic_prefix,
  int open_timeout_ms, int read_timeout_ms, int reconnect_delay_ms, int jpeg_quality)
{
  if (urls.empty() || urls.size() != names.size() || urls.size() != frame_ids.size()) {
    throw std::invalid_argument("rtsp_urls, camera_names, and frame_ids must have the same nonzero length");
  }
  if (open_timeout_ms < 1 || read_timeout_ms < 1 || reconnect_delay_ms < 1) {
    throw std::invalid_argument("open_timeout_ms, read_timeout_ms, and reconnect_delay_ms must be positive");
  }
  if (jpeg_quality < 1 || jpeg_quality > 100) {
    throw std::invalid_argument("jpeg_quality must be between 1 and 100");
  }
  std::set<std::string> seen;
  for (size_t i = 0; i < urls.size(); ++i) {
    if (urls[i].rfind("rtsp://", 0) != 0 && urls[i].rfind("rtsps://", 0) != 0) {
      throw std::invalid_argument("every rtsp_urls entry must begin with rtsp:// or rtsps://");
    }
    for (unsigned char c : urls[i]) {
      if (std::iscntrl(c)) throw std::invalid_argument("rtsp_urls contains a control character");
    }
    if (!valid_name(names[i]) || !seen.insert(names[i]).second) {
      throw std::invalid_argument("camera_names must be unique ROS names");
    }
    if (frame_ids[i].empty()) throw std::invalid_argument("frame_ids must not contain empty values");
  }
  auto normalized = topic_prefix;
  while (!normalized.empty() && normalized.back() == '/') normalized.pop_back();
  if (!normalized.empty()) {
    if (normalized.front() == '/') normalized.erase(0, 1);
    size_t start = 0;
    do {
      const auto end = normalized.find('/', start);
      const auto part = normalized.substr(start, end - start);
      if (!valid_name(part)) throw std::invalid_argument("topic_prefix contains an invalid ROS name");
      if (end == std::string::npos) break;
      start = end + 1;
    } while (true);
  }
}

}  // namespace uniubi_rtsp_driver
