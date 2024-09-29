/**
 * lidar_driver.cpp — Simulated VLP-16 LiDAR producer
 *
 * Generates ~300,000 points/second spread across 16 laser rings at 10 Hz
 * rotation (30,000 pts per revolution / 16 rings ≈ 1,875 pts per ring per rev).
 *
 * Runs on a SCHED_FIFO real-time thread to guarantee consistent inter-packet
 * timing, matching how a real VLP-16 driver would behave on a production
 * vehicle platform.
 *
 * Points are published into the provided SpscQueue<LidarPoint, N>.
 */

#include "veh_telemetry/lidar_driver.hpp"

#include <cmath>
#include <cstring>
#include <stdexcept>
#include <thread>
#include <pthread.h>
#include <sched.h>
#include <time.h>

namespace veh {

static constexpr int    kRings            = 16;
static constexpr int    kPointsPerRev     = 30000;        // ~300K pts/sec at 10Hz
static constexpr double kRevHz            = 10.0;
static constexpr long   kInterPtNs        = static_cast<long>(
    1e9 / (kPointsPerRev * kRevHz));                      // ns between points

// Elevation angles for VLP-16 (degrees, from spec sheet)
static const float kElevDeg[kRings] = {
    -15.0f, 1.0f, -13.0f, 3.0f, -11.0f, 5.0f, -9.0f,  7.0f,
     -7.0f, 9.0f,  -5.0f, 11.0f, -3.0f, 13.0f,-1.0f, 15.0f
};

static inline uint64_t monotonic_ns() {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return static_cast<uint64_t>(ts.tv_sec) * 1'000'000'000ULL +
           static_cast<uint64_t>(ts.tv_nsec);
}

LidarDriver::LidarDriver(LidarQueue &queue)
    : queue_(queue), running_(false), dropped_(0), published_(0)
{}

LidarDriver::~LidarDriver() { stop(); }

void LidarDriver::start() {
    running_.store(true, std::memory_order_relaxed);
    thread_ = std::thread(&LidarDriver::run, this);

    // Elevate to SCHED_FIFO priority 50 for deterministic timing
    struct sched_param sp{};
    sp.sched_priority = 50;
    if (pthread_setschedparam(thread_.native_handle(), SCHED_FIFO, &sp) != 0) {
        // Non-fatal on desktop; required on real ARM target
        // (needs CAP_SYS_NICE or appropriate cgroup)
    }
}

void LidarDriver::stop() {
    running_.store(false, std::memory_order_relaxed);
    if (thread_.joinable()) thread_.join();
}

void LidarDriver::run() {
    struct timespec deadline{};
    clock_gettime(CLOCK_MONOTONIC, &deadline);

    double azimuth_rad = 0.0;
    const double kAzimuthStep = 2.0 * M_PI / kPointsPerRev;
    uint32_t ring_idx = 0;

    while (running_.load(std::memory_order_relaxed)) {
        // Build a point on the simulated unit sphere, radius [5..30] m
        float elev_rad = kElevDeg[ring_idx] * (static_cast<float>(M_PI) / 180.0f);
        float r = 5.0f + 25.0f * (0.5f + 0.5f * std::sin(static_cast<float>(azimuth_rad) * 3.0f));

        LidarPoint pt;
        pt.x = static_cast<double>(r) * std::cos(static_cast<double>(elev_rad)) * std::cos(azimuth_rad);
        pt.y = static_cast<double>(r) * std::cos(static_cast<double>(elev_rad)) * std::sin(azimuth_rad);
        pt.z = static_cast<double>(r) * std::sin(static_cast<double>(elev_rad));
        pt.intensity = 128.0f + 64.0f * std::sin(static_cast<float>(azimuth_rad) * 7.0f);
        pt.timestamp_ns = monotonic_ns();
        pt.ring_id = static_cast<uint16_t>(ring_idx);

        if (!queue_.push(pt)) {
            dropped_.fetch_add(1, std::memory_order_relaxed);
        } else {
            published_.fetch_add(1, std::memory_order_relaxed);
        }

        ring_idx = (ring_idx + 1) % kRings;
        azimuth_rad += kAzimuthStep;
        if (azimuth_rad >= 2.0 * M_PI) azimuth_rad -= 2.0 * M_PI;

        // Absolute deadline sleep for tight timing
        deadline.tv_nsec += kInterPtNs;
        if (deadline.tv_nsec >= 1'000'000'000L) {
            deadline.tv_sec++;
            deadline.tv_nsec -= 1'000'000'000L;
        }
        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &deadline, nullptr);
    }
}

}  // namespace veh
