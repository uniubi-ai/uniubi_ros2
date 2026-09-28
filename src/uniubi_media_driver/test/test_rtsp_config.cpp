#include <gtest/gtest.h>
#include "rtsp_config.hpp"

using uniubi_rtsp_driver::image_topic;
using uniubi_rtsp_driver::validate_config;

TEST(RtspConfig, TopicsRespectNamespace)
{
  EXPECT_EQ(image_topic("", "front_camera_0"), "front_camera_0/image_raw");
  EXPECT_EQ(image_topic("/robot/cameras/", "front_camera_1"),
    "/robot/cameras/front_camera_1/image_raw");
}

TEST(RtspConfig, RejectsInconsistentOrUnsafeParameters)
{
  const std::vector<std::string> urls{"rtsp://127.0.0.1/one", "rtsp://127.0.0.1/two"};
  const std::vector<std::string> names{"front_camera_0", "front_camera_1"};
  const std::vector<std::string> frames{"front_optical", "right_optical"};
  EXPECT_NO_THROW(validate_config(urls, names, frames, "/robot", 5000, 2000, 1000, 80));
  EXPECT_THROW(validate_config(urls, {"front_camera_0"}, frames, "", 5000, 2000, 1000, 80),
    std::invalid_argument);
  EXPECT_THROW(validate_config(urls, {"front_camera_0", "front_camera_0"}, frames,
    "", 5000, 2000, 1000, 80), std::invalid_argument);
  EXPECT_THROW(validate_config(urls, names, frames, "robot//camera", 5000, 2000, 1000, 80),
    std::invalid_argument);
  EXPECT_THROW(validate_config({"http://127.0.0.1/a", urls[1]}, names, frames,
    "", 5000, 2000, 1000, 80), std::invalid_argument);
  EXPECT_THROW(validate_config(urls, names, frames, "", 0, 2000, 1000, 80),
    std::invalid_argument);
  EXPECT_THROW(validate_config(urls, names, frames, "", 5000, 2000, 1000, 101),
    std::invalid_argument);
}
