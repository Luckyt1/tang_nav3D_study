#include "comm/device_timestamp.h"

#include <atomic>
#include <iostream>
#include <stdexcept>
#include <thread>
#include <vector>

namespace {
using Mapper = livox_ros::DeviceTimestampMapper;
using Stream = Mapper::Stream;
constexpr uint64_t kHost = 1700000000000000000ULL;
constexpr uint64_t kDevice = 30000000000ULL;

void Check(bool condition, const char* message) {
  if (!condition) {
    throw std::runtime_error(message);
  }
}

void CheckArrivalJitterAndInterleaving() {
  Mapper mapper;
  const auto first = mapper.Map(1, 0, Stream::PointCloud, kDevice, kHost);
  Check(first.stamp == kHost, "first packet anchors to host epoch");
  const auto imu = mapper.Map(1, 0, Stream::Imu, kDevice - 2000000, kHost + 9000000);
  Check(imu.stamp == kHost - 2000000 && imu.epoch == first.epoch,
        "late older IMU retains its sample time and shared epoch");
  const auto cloud = mapper.Map(1, 0, Stream::PointCloud,
                                kDevice + 480000, kHost + 15000000);
  Check(cloud.stamp - first.stamp == 480000, "cloud interval ignores arrival delay");
  const auto imu2 = mapper.Map(1, 0, Stream::Imu,
                               kDevice + 3000000, kHost + 16000000);
  Check(imu2.stamp - imu.stamp == 5000000, "IMU interval preserves device cadence");
  const auto old_cloud = mapper.Map(1, 0, Stream::PointCloud,
                                    kDevice + 240000, kHost + 17000000);
  Check(old_cloud.stamp < cloud.stamp && !old_cloud.discontinuity,
        "small same-stream reordering is preserved, never clamped");
}

void CheckIndependentDevices() {
  Mapper mapper;
  mapper.Map(1, 0, Stream::Imu, kDevice, kHost);
  const auto second = mapper.Map(2, 0, Stream::Imu, 1000, kHost + 1000000);
  Check(second.stamp == kHost + 1000000, "each device has its own epoch");
  const auto first = mapper.Map(1, 0, Stream::PointCloud,
                                kDevice + 5000000, kHost + 9000000);
  Check(first.stamp == kHost + 5000000, "second device does not change first offset");
}

void CheckSynchronizedSourcesAndSwitches() {
  Mapper mapper;
  const auto ptp = mapper.Map(1, 1, Stream::PointCloud, kHost, kHost + 2000000);
  Check(ptp.stamp == kHost, "PTP passes through");
  const auto gps = mapper.Map(1, 2, Stream::Imu, kHost + 5000000, kHost + 9000000);
  Check(gps.stamp == kHost + 5000000, "GPS passes through");
  Check(gps.epoch > ptp.epoch && gps.discontinuity, "source change marks new epoch");
  const auto unsynced = mapper.Map(1, 0, Stream::PointCloud,
                                   kDevice, kHost + 20000000);
  Check(unsynced.stamp == kHost + 20000000 && unsynced.discontinuity,
        "losing synchronization reanchors device uptime");
  const auto imu = mapper.Map(1, 0, Stream::Imu,
                              kDevice - 1000000, kHost + 21000000);
  Check(imu.epoch == unsynced.epoch && imu.stamp == kHost + 19000000,
        "other stream joins new source epoch without resetting it");
}

void CheckRebootAndCrossStreamLag() {
  Mapper mapper;
  const auto first = mapper.Map(1, 0, Stream::PointCloud, kDevice, kHost);
  const auto lagging = mapper.Map(1, 0, Stream::Imu,
                                  kDevice - 2000000000, kHost + 1000000);
  Check(lagging.epoch == first.epoch && !lagging.discontinuity,
        "cross-stream lag is not a clock reboot");
  const auto reboot = mapper.Map(1, 0, Stream::PointCloud,
                                 100000000, kHost + 3000000000);
  Check(reboot.epoch > first.epoch && reboot.discontinuity,
        "large same-stream rollback begins a new epoch");
  Check(reboot.stamp == kHost + 3000000000, "reboot reanchors to current host epoch");
  const auto imu = mapper.Map(1, 0, Stream::Imu,
                              99000000, kHost + 3001000000);
  Check(imu.epoch == reboot.epoch && !imu.discontinuity &&
        imu.stamp == reboot.stamp - 1000000,
        "other stream joins reboot epoch with original relative timing");
}

void CheckConcurrentCallbacksAndHostClockJump() {
  Mapper mapper;
  mapper.Map(1, 0, Stream::PointCloud, kDevice, kHost);
  std::atomic<bool> ok{true};
  std::vector<std::thread> threads;
  for (unsigned thread = 0; thread < 4; ++thread) {
    threads.emplace_back([&, thread] {
      for (unsigned i = 0; i < 1000; ++i) {
        const uint64_t offset = (thread * 1000 + i) * 1000;
        const auto result = mapper.Map(1, 0, thread % 2 ? Stream::Imu : Stream::PointCloud,
                                       kDevice + offset, kHost - 10000000000ULL);
        if (result.stamp != kHost + offset || result.epoch != 1) {
          ok.store(false);
        }
      }
    });
  }
  for (auto& thread : threads) {
    thread.join();
  }
  Check(ok.load(), "concurrent callbacks preserve mapping despite a host clock step");
}
}  // namespace

int main() {
  try {
    CheckArrivalJitterAndInterleaving();
    CheckIndependentDevices();
    CheckSynchronizedSourcesAndSwitches();
    CheckRebootAndCrossStreamLag();
    CheckConcurrentCallbacksAndHostClockJump();
  } catch (const std::exception& error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
  std::cout << "Device timestamp tests passed (5 scenarios).\n";
}
