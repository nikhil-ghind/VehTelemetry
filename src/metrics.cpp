/**
 * metrics.cpp — Lightweight in-process metrics for the telemetry pipeline
 *
 * Records per-second throughput, fusion latency histogram, and queue depths.
 * A background thread logs a summary to stderr every 5 seconds so operators
 * can spot queue back-pressure or latency spikes without external tooling.
 */

#include "veh_telemetry/metrics.hpp"

#include <cstdio>
#include <chrono>
#include <algorithm>
#include <numeric>

namespace veh {

Metrics::Metrics()
    : lidar_samples_(0), imu_samples_(0), can_frames_(0), fused_samples_(0),
      drops_(0), fusion_latency_sum_ns_(0), fusion_latency_count_(0),
      fusion_latency_max_ns_(0), running_(false)
{}

Metrics::~Metrics() { stop(); }

void Metrics::start() {
    running_.store(true, std::memory_order_relaxed);
    epoch_ = std::chrono::steady_clock::now();
    thread_ = std::thread(&Metrics::run, this);
}

void Metrics::stop() {
    running_.store(false, std::memory_order_relaxed);
    if (thread_.joinable()) thread_.join();
}

void Metrics::record_lidar(uint64_t n)  { lidar_samples_.fetch_add(n, std::memory_order_relaxed); }
void Metrics::record_imu(uint64_t n)    { imu_samples_.fetch_add(n, std::memory_order_relaxed); }
void Metrics::record_can(uint64_t n)    { can_frames_.fetch_add(n, std::memory_order_relaxed); }
void Metrics::record_fused(uint64_t n)  { fused_samples_.fetch_add(n, std::memory_order_relaxed); }
void Metrics::record_drop()             { drops_.fetch_add(1, std::memory_order_relaxed); }

void Metrics::record_fusion_latency_ns(uint64_t ns) {
    fusion_latency_sum_ns_.fetch_add(ns, std::memory_order_relaxed);
    fusion_latency_count_.fetch_add(1, std::memory_order_relaxed);

    // Update max with a CAS loop
    uint64_t prev = fusion_latency_max_ns_.load(std::memory_order_relaxed);
    while (ns > prev &&
           !fusion_latency_max_ns_.compare_exchange_weak(
               prev, ns, std::memory_order_relaxed)) {}
}

void Metrics::record_queue_depth(const char *name, std::size_t depth) {
    std::lock_guard<std::mutex> lk(queue_depths_mu_);
    queue_depths_[name] = depth;
}

void Metrics::run() {
    using namespace std::chrono_literals;
    while (running_.load(std::memory_order_relaxed)) {
        std::this_thread::sleep_for(5s);
        print_report();
    }
}

void Metrics::print_report() {
    auto now     = std::chrono::steady_clock::now();
    double secs  = std::chrono::duration<double>(now - epoch_).count();

    uint64_t lidar  = lidar_samples_.load(std::memory_order_relaxed);
    uint64_t imu    = imu_samples_.load(std::memory_order_relaxed);
    uint64_t can    = can_frames_.load(std::memory_order_relaxed);
    uint64_t fused  = fused_samples_.load(std::memory_order_relaxed);
    uint64_t drops  = drops_.load(std::memory_order_relaxed);
    uint64_t lat_sum = fusion_latency_sum_ns_.load(std::memory_order_relaxed);
    uint64_t lat_cnt = fusion_latency_count_.load(std::memory_order_relaxed);
    uint64_t lat_max = fusion_latency_max_ns_.load(std::memory_order_relaxed);

    double avg_lat_us = (lat_cnt > 0)
        ? (static_cast<double>(lat_sum) / lat_cnt / 1000.0) : 0.0;

    fprintf(stderr,
        "[metrics] uptime=%.1fs  "
        "lidar=%.0f/s  imu=%.0f/s  can=%.0f/s  fused=%.0f/s  "
        "drops=%llu  fusion_lat avg=%.1fus max=%.1fus\n",
        secs,
        lidar  / secs, imu  / secs, can  / secs, fused / secs,
        (unsigned long long)drops,
        avg_lat_us,
        static_cast<double>(lat_max) / 1000.0);

    // Queue depths
    {
        std::lock_guard<std::mutex> lk(queue_depths_mu_);
        for (const auto &kv : queue_depths_)
            fprintf(stderr, "[metrics]   queue[%s] depth=%zu\n",
                    kv.first.c_str(), kv.second);
    }
}

}  // namespace veh
