/**
 * fusion_engine.cpp — Complementary filter + dead-reckoning fusion at 100 Hz
 *
 * Algorithm:
 *   1. Consume all available IMU samples since the last fusion tick.
 *      Integrate gyroscope to propagate orientation quaternion (gyro update).
 *   2. Every fusion tick, blend the accelerometer-derived pitch/roll into the
 *      quaternion via a low-pass complementary filter (alpha=0.98 gyro, 0.02 accel).
 *   3. Integrate the gravity-corrected acceleration to update dead-reckoning position.
 *   4. Merge the latest OBD speed into the position model for forward velocity.
 *   5. Sample one LiDAR batch to compute a crude forward-sector point density.
 *   6. Publish a FusedSample.
 */

#include "veh_telemetry/fusion_engine.hpp"

#include <cmath>
#include <cstring>
#include <time.h>

namespace veh {

static constexpr double kFusionHz   = 100.0;
static constexpr long   kFusionNs   = static_cast<long>(1e9 / kFusionHz);  // 10 ms
static constexpr float  kAlpha      = 0.98f;  // gyro weight in complementary filter

static inline uint64_t monotonic_ns() {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return static_cast<uint64_t>(ts.tv_sec) * 1'000'000'000ULL +
           static_cast<uint64_t>(ts.tv_nsec);
}

static inline float qnorm(float w, float x, float y, float z) {
    return std::sqrt(w*w + x*x + y*y + z*z);
}

// Normalise quaternion in-place
static inline void qnormalise(float &w, float &x, float &y, float &z) {
    float n = qnorm(w, x, y, z);
    if (n < 1e-6f) { w = 1; x = y = z = 0; return; }
    float inv = 1.0f / n;
    w *= inv; x *= inv; y *= inv; z *= inv;
}

FusionEngine::FusionEngine(LidarQueue &lidar, ImuQueue &imu, ObdQueue &obd,
                           FusedQueue &out, Metrics &metrics)
    : lidar_(lidar), imu_(imu), obd_(obd), out_(out), metrics_(metrics),
      running_(false),
      qw_(1.0f), qx_(0.0f), qy_(0.0f), qz_(0.0f),
      pos_x_(0.0), pos_y_(0.0), pos_z_(0.0),
      vel_x_(0.0), vel_y_(0.0), vel_z_(0.0),
      last_speed_mps_(0.0f), last_rpm_(0.0f), last_throttle_(0.0f),
      published_(0)
{}

FusionEngine::~FusionEngine() { stop(); }

void FusionEngine::start() {
    running_.store(true, std::memory_order_relaxed);
    thread_ = std::thread(&FusionEngine::run, this);
}

void FusionEngine::stop() {
    running_.store(false, std::memory_order_relaxed);
    if (thread_.joinable()) thread_.join();
}

void FusionEngine::run() {
    struct timespec deadline{};
    clock_gettime(CLOCK_MONOTONIC, &deadline);

    while (running_.load(std::memory_order_relaxed)) {
        uint64_t t0 = monotonic_ns();

        // ── 1. Drain IMU queue ────────────────────────────────────────────
        ImuSample imu_s;
        float accel_x = 0, accel_y = 0, accel_z = 9.80665f;
        float gyro_x  = 0, gyro_y  = 0, gyro_z  = 0;
        int   imu_count = 0;
        const float dt_imu = 1.0f / 1000.0f;  // IMU runs at 1 kHz

        while (imu_.pop(imu_s)) {
            // Gyroscope integration: quaternion differential
            float half_dt = 0.5f * dt_imu;
            float dqw = -half_dt * (qx_*imu_s.gyro_x + qy_*imu_s.gyro_y + qz_*imu_s.gyro_z);
            float dqx =  half_dt * (qw_*imu_s.gyro_x + qy_*imu_s.gyro_z - qz_*imu_s.gyro_y);
            float dqy =  half_dt * (qw_*imu_s.gyro_y - qx_*imu_s.gyro_z + qz_*imu_s.gyro_x);
            float dqz =  half_dt * (qw_*imu_s.gyro_z + qx_*imu_s.gyro_y - qy_*imu_s.gyro_x);
            qw_ += dqw; qx_ += dqx; qy_ += dqy; qz_ += dqz;
            qnormalise(qw_, qx_, qy_, qz_);

            accel_x += imu_s.accel_x;
            accel_y += imu_s.accel_y;
            accel_z += imu_s.accel_z;
            gyro_x  += imu_s.gyro_x;
            gyro_y  += imu_s.gyro_y;
            gyro_z  += imu_s.gyro_z;
            imu_count++;
        }

        if (imu_count > 0) {
            float inv = 1.0f / static_cast<float>(imu_count);
            accel_x *= inv; accel_y *= inv; accel_z *= inv;
            gyro_x  *= inv; gyro_y  *= inv; gyro_z  *= inv;

            // Complementary filter: blend accel-derived pitch/roll
            float accel_norm = std::sqrt(accel_x*accel_x + accel_y*accel_y + accel_z*accel_z);
            if (accel_norm > 0.5f) {
                float ax = accel_x / accel_norm;
                float ay = accel_y / accel_norm;
                // Target quaternion from gravity direction (simplified)
                float pitch = std::asin(-ax);
                float roll  = std::atan2(ay, std::sqrt(1.0f - ax*ax));
                float cos_p2 = std::cos(pitch*0.5f), sin_p2 = std::sin(pitch*0.5f);
                float cos_r2 = std::cos(roll*0.5f),  sin_r2 = std::sin(roll*0.5f);
                float aw = cos_p2 * cos_r2;
                float ax2 = cos_p2 * sin_r2;
                float ay2 = sin_p2 * cos_r2;
                float az2 = -sin_p2 * sin_r2;
                // Blend
                qw_ = kAlpha*qw_ + (1.0f-kAlpha)*aw;
                qx_ = kAlpha*qx_ + (1.0f-kAlpha)*ax2;
                qy_ = kAlpha*qy_ + (1.0f-kAlpha)*ay2;
                qz_ = kAlpha*qz_ + (1.0f-kAlpha)*az2;
                qnormalise(qw_, qx_, qy_, qz_);
            }
        }

        // ── 2. Drain OBD queue ────────────────────────────────────────────
        ObdData obd_s;
        while (obd_.pop(obd_s)) {
            if (obd_s.speed_mps > 0)    last_speed_mps_   = obd_s.speed_mps;
            if (obd_s.rpm > 0)          last_rpm_         = obd_s.rpm;
            if (obd_s.throttle_pct > 0) last_throttle_    = obd_s.throttle_pct;
        }

        // ── 3. Dead-reckoning position update ─────────────────────────────
        const double dt_fusion = 1.0 / kFusionHz;
        // Simple: assume vehicle moves in the direction of its yaw quaternion
        double yaw = std::atan2(2.0*(qw_*qz_ + qx_*qy_), 1.0 - 2.0*(qy_*qy_ + qz_*qz_));
        vel_x_ = static_cast<double>(last_speed_mps_) * std::cos(yaw);
        vel_y_ = static_cast<double>(last_speed_mps_) * std::sin(yaw);
        vel_z_ = 0.0;
        pos_x_ += vel_x_ * dt_fusion;
        pos_y_ += vel_y_ * dt_fusion;

        // ── 4. LiDAR forward-sector density ───────────────────────────────
        float lidar_density = 0.0f;
        {
            LidarPoint lp;
            int fwd_pts = 0, total_pts = 0;
            while (total_pts < 500 && lidar_.pop(lp)) {
                total_pts++;
                // Forward sector: x > 0, |y| < 2m, z in [-1, 2]m
                if (lp.x > 0 && std::abs(lp.y) < 2.0 && lp.z > -1.0 && lp.z < 2.0)
                    fwd_pts++;
            }
            if (total_pts > 0)
                lidar_density = static_cast<float>(fwd_pts) / (4.0f * 3.0f * 1.0f);  // per m³ approx
        }

        // ── 5. Build and publish FusedSample ──────────────────────────────
        FusedSample fs;
        fs.timestamp_ns    = monotonic_ns();
        fs.qw = qw_; fs.qx = qx_; fs.qy = qy_; fs.qz = qz_;
        fs.omega_x = gyro_x;  fs.omega_y = gyro_y;  fs.omega_z = gyro_z;
        fs.lin_accel_x = accel_x; fs.lin_accel_y = accel_y;
        fs.lin_accel_z = accel_z - 9.80665f;  // gravity-subtracted
        fs.pos_x = pos_x_; fs.pos_y = pos_y_; fs.pos_z = pos_z_;
        fs.speed_mps     = last_speed_mps_;
        fs.rpm           = last_rpm_;
        fs.throttle_pct  = last_throttle_;
        fs.lidar_density_fwd = lidar_density;
        memset(fs._pad, 0, sizeof(fs._pad));

        uint64_t t1 = monotonic_ns();
        metrics_.record_fusion_latency_ns(t1 - t0);

        if (!out_.push(fs)) {
            metrics_.record_drop();
        } else {
            published_.fetch_add(1, std::memory_order_relaxed);
        }

        // ── 6. Sleep until next 10 ms tick ────────────────────────────────
        deadline.tv_nsec += kFusionNs;
        if (deadline.tv_nsec >= 1'000'000'000L) {
            deadline.tv_sec++;
            deadline.tv_nsec -= 1'000'000'000L;
        }
        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &deadline, nullptr);
    }
}

}  // namespace veh
