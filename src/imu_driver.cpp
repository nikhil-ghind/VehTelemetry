/**
 * imu_driver.cpp — Simulated MPU-9250 IMU producer at 1 kHz
 *
 * In production the driver reads from the MPU-9250 via SPI at 1 kHz using
 * SCHED_FIFO to meet the timing budget.  This simulator generates a realistic
 * noise-plus-gravity signal suitable for testing the complementary filter in
 * fusion_engine.cpp without real hardware.
 */

#include "veh_telemetry/imu_driver.hpp"

#include <cmath>
#include <pthread.h>
#include <sched.h>
#include <time.h>

namespace veh {

static constexpr long kPeriodNs = 1'000'000L;  // 1 ms = 1 kHz

static inline uint64_t monotonic_ns() {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return static_cast<uint64_t>(ts.tv_sec) * 1'000'000'000ULL +
           static_cast<uint64_t>(ts.tv_nsec);
}

// Tiny LCG for zero-dep noise generation
static float lcg_randf(uint32_t &state) {
    state = state * 1664525u + 1013904223u;
    return (static_cast<float>(state >> 16) / 32768.0f) - 1.0f;  // [-1, 1)
}

ImuDriver::ImuDriver(ImuQueue &queue)
    : queue_(queue), running_(false), dropped_(0), published_(0)
{}

ImuDriver::~ImuDriver() { stop(); }

void ImuDriver::start() {
    running_.store(true, std::memory_order_relaxed);
    thread_ = std::thread(&ImuDriver::run, this);

    struct sched_param sp{};
    sp.sched_priority = 60;  // Slightly higher than LiDAR
    pthread_setschedparam(thread_.native_handle(), SCHED_FIFO, &sp);
}

void ImuDriver::stop() {
    running_.store(false, std::memory_order_relaxed);
    if (thread_.joinable()) thread_.join();
}

void ImuDriver::run() {
    struct timespec deadline{};
    clock_gettime(CLOCK_MONOTONIC, &deadline);

    uint32_t noise_state = 0xDEADBEEFu;
    double t = 0.0;
    const double dt = 1.0 / 1000.0;

    while (running_.load(std::memory_order_relaxed)) {
        ImuSample s;

        // Gravity vector (vehicle at rest tilted ~2° pitch)
        const float g = 9.80665f;
        s.accel_x = g * std::sin(0.035f) + 0.005f * lcg_randf(noise_state);
        s.accel_y = 0.0f              + 0.005f * lcg_randf(noise_state);
        s.accel_z = g * std::cos(0.035f) + 0.005f * lcg_randf(noise_state);

        // Slow yaw rotation (simulates turning at ~5 deg/s)
        s.gyro_x = 0.002f * lcg_randf(noise_state);
        s.gyro_y = 0.002f * lcg_randf(noise_state);
        s.gyro_z = static_cast<float>(5.0 * M_PI / 180.0) + 0.001f * lcg_randf(noise_state);

        // Earth's magnetic field (approx. mid-latitude heading)
        s.mag_x = 20.0f + 0.1f * lcg_randf(noise_state);
        s.mag_y = -2.0f + 0.1f * lcg_randf(noise_state);
        s.mag_z = 45.0f + 0.1f * lcg_randf(noise_state);

        s.temperature_c = 25.0f + 0.01f * static_cast<float>(t);
        s.timestamp_ns  = monotonic_ns();

        if (!queue_.push(s))
            dropped_.fetch_add(1, std::memory_order_relaxed);
        else
            published_.fetch_add(1, std::memory_order_relaxed);

        t += dt;
        deadline.tv_nsec += kPeriodNs;
        if (deadline.tv_nsec >= 1'000'000'000L) {
            deadline.tv_sec++;
            deadline.tv_nsec -= 1'000'000'000L;
        }
        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &deadline, nullptr);
    }
}

}  // namespace veh
