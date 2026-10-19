#ifndef LIVOX_DRIVER_DEVICE_TIMESTAMP_H_
#define LIVOX_DRIVER_DEVICE_TIMESTAMP_H_

#include <array>
#include <cstdint>
#include <limits>
#include <map>
#include <mutex>

namespace livox_ros {

// CustomMsg stores each point's offset from the frame base in uint32 ns.
// Arrival order may differ from sampling order, so use the full time range.
class PointTimestampRange {
 public:
  void Add(uint64_t stamp) {
    if (empty_) {
      first_ = last_ = stamp;
      empty_ = false;
    } else {
      if (stamp < first_) first_ = stamp;
      if (stamp > last_) last_ = stamp;
    }
  }
  uint64_t First() const { return first_; }
  uint64_t Last() const { return last_; }
  bool CanEncodeOffsets() const {
    return !empty_ && last_ - first_ <= std::numeric_limits<uint32_t>::max();
  }
  void Reset() { *this = PointTimestampRange{}; }

 private:
  bool empty_ = true;
  uint64_t first_ = 0;
  uint64_t last_ = 0;
};

// Livox time_type: 0 = device uptime, 1 = PTP, 2 = GPS. Both sensor
// streams use the same device clock. All timestamps here are nanoseconds.
class DeviceTimestampMapper {
 public:
  enum class Stream { PointCloud = 0, Imu = 1 };
  struct Result {
    uint64_t stamp;
    uint64_t epoch;
    bool discontinuity;
  };

  Result Map(uint32_t handle, uint8_t source, Stream stream,
             uint64_t device_ns, uint64_t host_ns) {
    std::lock_guard<std::mutex> lock(mutex_);
    auto& clock = clocks_[handle];
    const auto index = static_cast<unsigned>(stream);
    // Compare a stream with itself: an IMU sample may normally arrive after a
    // cloud packet whose device timestamp is newer (and vice versa).
    // Reordering below 1 s stays visible; do not clamp individual samples.
    const bool rollback = clock.seen[index] &&
        device_ns < clock.latest[index] &&
        clock.latest[index] - device_ns > 1000000000ULL;
    const bool discontinuity = clock.initialized &&
        (source != clock.source || rollback);
    if (!clock.initialized || discontinuity) {
      clock.initialized = true;
      clock.source = source;
      clock.device_origin = device_ns;
      clock.host_origin = host_ns;
      clock.seen = {{false, false}};
      clock.latest = {{0, 0}};
      ++clock.epoch;
    }
    clock.seen[index] = true;
    if (device_ns > clock.latest[index]) {
      clock.latest[index] = device_ns;
    }

    if (source == 1 || source == 2) {
      return {device_ns, clock.epoch, discontinuity};
    }
    // One fixed host/device offset per epoch removes packet arrival jitter.
    // It does not correct oscillator drift or the first packet's transport
    // delay; use PTP/GPS when absolute synchronization is required.
    const uint64_t stamp = device_ns >= clock.device_origin
        ? clock.host_origin + (device_ns - clock.device_origin)
        : clock.host_origin - (clock.device_origin - device_ns);
    return {stamp, clock.epoch, discontinuity};
  }

 private:
  struct Clock {
    bool initialized = false;
    uint8_t source = 0;
    uint64_t device_origin = 0;
    uint64_t host_origin = 0;
    uint64_t epoch = 0;
    std::array<bool, 2> seen{{false, false}};
    std::array<uint64_t, 2> latest{{0, 0}};
  };
  std::mutex mutex_;
  std::map<uint32_t, Clock> clocks_;
};

}  // namespace livox_ros

#endif  // LIVOX_DRIVER_DEVICE_TIMESTAMP_H_
