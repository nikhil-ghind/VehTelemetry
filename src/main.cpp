/**
 * main.cpp — VehTelemetry pipeline entry point
 *
 * Instantiates all components, wires queues, starts threads, and handles
 * graceful shutdown on SIGINT/SIGTERM.
 *
 * Thread layout:
 *   lidar_thread   (SCHED_FIFO, pri=50) — 300K pts/s
 *   imu_thread     (SCHED_FIFO, pri=60) — 1 kHz
 *   can_thread     (normal)             — 50 Hz sim / SocketCAN
 *   fusion_thread  (normal)             — 100 Hz
 *   export_thread  (normal)             — batch+rotate Parquet files
 *   metrics_thread (normal)             — 5s log interval
 */

#include "veh_telemetry/lidar_driver.hpp"
#include "veh_telemetry/imu_driver.hpp"
#include "veh_telemetry/can_driver.hpp"
#include "veh_telemetry/fusion_engine.hpp"
#include "veh_telemetry/parquet_exporter.hpp"
#include "veh_telemetry/metrics.hpp"

#include <cstdio>
#include <csignal>
#include <atomic>
#include <chrono>
#include <thread>

// ── Config (can be replaced by YAML parsing from config/pipeline.yaml) ───

static constexpr int  kBatchSize      = 1000;
static constexpr int  kRotationSec    = 60;
static const char    *kOutputDir      = "./telemetry_output";
static const char    *kCanIface       = "vcan0";

// ── Signal handling ───────────────────────────────────────────────────────

static std::atomic<bool> g_running{true};

static void on_signal(int sig) {
    fprintf(stderr, "\nveh_telemetry: caught signal %d — shutting down…\n", sig);
    g_running.store(false, std::memory_order_relaxed);
}

// ── Queue singletons ──────────────────────────────────────────────────────
//
// Queues live in main so their lifetime spans all component threads.

static veh::LidarQueue  g_lidar_q;
static veh::ImuQueue    g_imu_q;
static veh::CanQueue    g_can_raw_q;
static veh::ObdQueue    g_obd_q;
static veh::FusedQueue  g_fused_q;

// ── Entry point ───────────────────────────────────────────────────────────

int main(int argc, char *argv[]) {
    (void)argc; (void)argv;

    signal(SIGINT,  on_signal);
    signal(SIGTERM, on_signal);

    fprintf(stderr, "veh_telemetry: starting pipeline\n");
    fprintf(stderr, "  LiDAR queue capacity : %zu\n", g_lidar_q.capacity());
    fprintf(stderr, "  IMU   queue capacity : %zu\n", g_imu_q.capacity());
    fprintf(stderr, "  Fused queue capacity : %zu\n", g_fused_q.capacity());
    fprintf(stderr, "  Output directory     : %s\n",  kOutputDir);
    fprintf(stderr, "  Parquet rotation     : %ds / %d rows\n",
            kRotationSec, kBatchSize);

    // ── Instantiate components ────────────────────────────────────────────
    veh::Metrics          metrics;
    veh::LidarDriver      lidar(g_lidar_q);
    veh::ImuDriver        imu(g_imu_q);
    veh::CanDriver        can(g_can_raw_q, g_obd_q, kCanIface);
    veh::FusionEngine     fusion(g_lidar_q, g_imu_q, g_obd_q, g_fused_q, metrics);
    veh::ParquetExporter  exporter(g_fused_q, kOutputDir, kBatchSize, kRotationSec);

    // ── Start all threads ─────────────────────────────────────────────────
    metrics.start();
    lidar.start();
    imu.start();
    can.start();
    fusion.start();
    exporter.start();

    if (can.is_simulated())
        fprintf(stderr, "veh_telemetry: CAN using simulator (no %s found)\n", kCanIface);

    fprintf(stderr, "veh_telemetry: all threads started — press Ctrl+C to stop\n");

    // ── Main loop: periodically update queue depth metrics ────────────────
    while (g_running.load(std::memory_order_relaxed)) {
        using namespace std::chrono_literals;
        std::this_thread::sleep_for(1s);

        metrics.record_queue_depth("lidar",  g_lidar_q.size_approx());
        metrics.record_queue_depth("imu",    g_imu_q.size_approx());
        metrics.record_queue_depth("can_raw",g_can_raw_q.size_approx());
        metrics.record_queue_depth("obd",    g_obd_q.size_approx());
        metrics.record_queue_depth("fused",  g_fused_q.size_approx());
    }

    // ── Graceful shutdown ─────────────────────────────────────────────────
    fprintf(stderr, "veh_telemetry: stopping components…\n");
    exporter.stop();  // flush remaining batches first
    fusion.stop();
    can.stop();
    imu.stop();
    lidar.stop();
    metrics.stop();

    fprintf(stderr,
        "veh_telemetry: shutdown complete\n"
        "  LiDAR  published=%llu dropped=%llu\n"
        "  IMU    published=%llu dropped=%llu\n"
        "  Fusion published=%llu\n"
        "  Parquet files=%llu  rows=%llu\n",
        (unsigned long long)lidar.published(),
        (unsigned long long)lidar.dropped(),
        (unsigned long long)imu.published(),
        (unsigned long long)imu.dropped(),
        (unsigned long long)fusion.published(),
        (unsigned long long)exporter.files_written(),
        (unsigned long long)exporter.rows_written());

    return 0;
}
