#include "comm/pub_handler.h"

#include <iostream>
#include <limits>
#include <stdexcept>

namespace {
using namespace livox_ros;
constexpr uint64_t kBase = 1700000000000000000ULL;

void Check(bool condition, const char* message) {
  if (!condition) throw std::runtime_error(message);
}

void AddPacket(LidarPubHandler& handler, uint8_t type, uint64_t stamp,
               uint32_t count = 1) {
  RawPacket packet{};
  packet.lidar_type = kLivoxLidarType;
  packet.data_type = type;
  packet.time_stamp = stamp;
  packet.point_num = count;
  packet.point_interval = 100;
  packet.line_num = 4;
  size_t point_size = sizeof(LivoxLidarCartesianHighRawPoint);
  if (type == kLivoxLidarCartesianCoordinateLowData) {
    point_size = sizeof(LivoxLidarCartesianLowRawPoint);
  } else if (type == kLivoxLidarSphericalCoordinateData) {
    point_size = sizeof(LivoxLidarSpherPoint);
  }
  packet.raw_data.resize(point_size * count, 0);
  handler.PointCloudProcess(packet);
}

void CheckFrames(uint8_t type) {
  LidarPubHandler handler;
  std::vector<PointXyzlt> points;
  uint64_t base = 0;

  AddPacket(handler, type, kBase, 2);
  AddPacket(handler, type, kBase + 480000, 2);
  Check(handler.GetLidarPointClouds(points, base), "normal frame accepted");
  Check(base == kBase && points.size() == 4, "normal frame base/count preserved");
  Check(points[0].offset_time - base == 0 &&
        points[3].offset_time - base == 480100, "normal point offsets preserved");

  // The first-arriving packet is 1 ms newer than a later-arriving packet.
  AddPacket(handler, type, kBase + 100000000, 2);
  AddPacket(handler, type, kBase + 99000000, 2);
  Check(handler.GetLidarBaseTime() == kBase + 99000000,
        "frame base is earliest sample, not first arrival");
  Check(handler.GetRecentTimeStamp() == kBase + 100000100,
        "frame end is latest sample, not last arrival");
  Check(handler.GetLidarPointClouds(points, base), "reordered frame accepted");
  const uint32_t first_offset = static_cast<uint32_t>(points[0].offset_time - base);
  const uint32_t older_offset = static_cast<uint32_t>(points[2].offset_time - base);
  Check(first_offset == 1000000 && older_offset == 0,
        "CustomMsg offsets do not underflow after frame-start reordering");
  Check(handler.GetLidarBaseTime() == 0 && handler.GetRecentTimeStamp() == 0,
        "time range resets when frame is extracted");

  const uint64_t max_offset = std::numeric_limits<uint32_t>::max();
  AddPacket(handler, type, kBase);
  AddPacket(handler, type, kBase + max_offset);
  Check(handler.GetLidarPointClouds(points, base), "uint32 boundary is representable");
  Check(static_cast<uint32_t>(points[1].offset_time - base) == max_offset,
        "boundary offset remains exact");

  AddPacket(handler, type, kBase);
  AddPacket(handler, type, kBase + max_offset + 1);
  Check(!handler.GetLidarPointClouds(points, base) && points.empty() && base == 0,
        "unrepresentable frame is discarded instead of wrapping offsets");
  Check(handler.GetLidarPointCloudsSize() == 0, "invalid frame is cleared");
  AddPacket(handler, type, kBase + 10000000000ULL, 2);
  Check(handler.GetLidarPointClouds(points, base) && points.size() == 2 &&
        points[1].offset_time - base == 100,
        "next valid frame recovers after invalid timestamp span");
}
}  // namespace

int main() {
  try {
    CheckFrames(kLivoxLidarCartesianCoordinateHighData);
    CheckFrames(kLivoxLidarCartesianCoordinateLowData);
    CheckFrames(kLivoxLidarSphericalCoordinateData);
  } catch (const std::exception& error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
  std::cout << "Point frame timing tests passed for all 3 point formats.\n";
}
