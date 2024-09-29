/**
 * can_driver.cpp — SocketCAN receiver + OBD-II decoder
 *
 * Opens a raw CAN socket on the interface named in config (default: vcan0).
 * Decodes OBD-II response frames (CAN ID 0x7E8) for:
 *   PID 0x0D — Vehicle speed  (km/h → m/s)
 *   PID 0x0C — Engine RPM     (raw/4)
 *   PID 0x11 — Throttle pos   (% = raw*100/255)
 *
 * Falls back to a simulator thread when the socket bind fails (no vcan0
 * loaded) so the rest of the pipeline works on a development machine.
 */

#include "veh_telemetry/can_driver.hpp"

#include <cstring>
#include <cstdio>
#include <cerrno>
#include <cmath>

// Linux-specific CAN headers (compile guard for macOS dev builds)
#ifdef __linux__
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <unistd.h>
#include <fcntl.h>
#include <time.h>
#endif

namespace veh {

static constexpr uint32_t kOBD2_RESPONSE_ID = 0x7E8;
static constexpr uint8_t  kPID_SPEED        = 0x0D;
static constexpr uint8_t  kPID_RPM          = 0x0C;
static constexpr uint8_t  kPID_THROTTLE     = 0x11;

static inline uint64_t monotonic_ns() {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return static_cast<uint64_t>(ts.tv_sec) * 1'000'000'000ULL +
           static_cast<uint64_t>(ts.tv_nsec);
}

// ── Simulator ────────────────────────────────────────────────────────────

static uint32_t sim_rng = 0xCAFEBABEu;
static float sim_randf() {
    sim_rng = sim_rng * 1664525u + 1013904223u;
    return static_cast<float>(sim_rng >> 16) / 65536.0f;
}

CanDriver::CanDriver(CanQueue &raw_queue, ObdQueue &obd_queue,
                     const std::string &iface)
    : raw_queue_(raw_queue), obd_queue_(obd_queue), iface_(iface),
      running_(false), sockfd_(-1), use_simulator_(false),
      dropped_(0), published_(0)
{}

CanDriver::~CanDriver() { stop(); }

void CanDriver::start() {
    running_.store(true, std::memory_order_relaxed);

#ifdef __linux__
    // Try to open a real SocketCAN socket
    sockfd_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (sockfd_ < 0) {
        fprintf(stderr, "can_driver: socket() failed (%s) — using simulator\n",
                strerror(errno));
        use_simulator_ = true;
    } else {
        struct ifreq ifr;
        strncpy(ifr.ifr_name, iface_.c_str(), IFNAMSIZ - 1);
        if (ioctl(sockfd_, SIOCGIFINDEX, &ifr) < 0) {
            fprintf(stderr, "can_driver: interface %s not found — using simulator\n",
                    iface_.c_str());
            close(sockfd_);
            sockfd_ = -1;
            use_simulator_ = true;
        } else {
            struct sockaddr_can addr{};
            addr.can_family  = AF_CAN;
            addr.can_ifindex = ifr.ifr_ifindex;
            if (bind(sockfd_, reinterpret_cast<struct sockaddr *>(&addr),
                     sizeof(addr)) < 0) {
                fprintf(stderr, "can_driver: bind failed — using simulator\n");
                close(sockfd_);
                sockfd_ = -1;
                use_simulator_ = true;
            }
        }
    }
#else
    use_simulator_ = true;
#endif

    thread_ = std::thread(use_simulator_
                          ? &CanDriver::run_simulator
                          : &CanDriver::run_socketcan, this);
}

void CanDriver::stop() {
    running_.store(false, std::memory_order_relaxed);
#ifdef __linux__
    if (sockfd_ >= 0) { close(sockfd_); sockfd_ = -1; }
#endif
    if (thread_.joinable()) thread_.join();
}

// ── Decode OBD-II from a raw CAN frame ───────────────────────────────────

bool CanDriver::decode_obd(const CanFrame &frame, ObdData &obd) {
    if (frame.can_id != kOBD2_RESPONSE_ID || frame.dlc < 3)
        return false;

    // Byte 0: length, Byte 1: mode (0x41 = positive response to mode 0x01),
    // Byte 2: PID, Byte 3+: data
    if (frame.data[1] != 0x41) return false;

    uint8_t pid = frame.data[2];
    obd.timestamp_ns = frame.timestamp_ns;

    if (pid == kPID_SPEED) {
        obd.speed_mps = frame.data[3] / 3.6f;  // km/h → m/s
        return true;
    } else if (pid == kPID_RPM) {
        obd.rpm = static_cast<float>((frame.data[3] * 256 + frame.data[4])) / 4.0f;
        return true;
    } else if (pid == kPID_THROTTLE) {
        obd.throttle_pct = frame.data[3] * 100.0f / 255.0f;
        return true;
    }
    return false;
}

// ── SocketCAN receive loop ────────────────────────────────────────────────

void CanDriver::run_socketcan() {
#ifdef __linux__
    struct can_frame kframe;
    while (running_.load(std::memory_order_relaxed)) {
        ssize_t nbytes = recv(sockfd_, &kframe, sizeof(kframe), 0);
        if (nbytes <= 0) break;

        CanFrame frame;
        frame.can_id       = kframe.can_id & CAN_EFF_MASK;
        frame.dlc          = kframe.can_dlc;
        memcpy(frame.data, kframe.data, 8);
        frame.timestamp_ns = monotonic_ns();

        raw_queue_.push(frame);

        ObdData obd{};
        if (decode_obd(frame, obd)) {
            if (!obd_queue_.push(obd))
                dropped_.fetch_add(1, std::memory_order_relaxed);
            else
                published_.fetch_add(1, std::memory_order_relaxed);
        }
    }
#endif
}

// ── Simulator: synthesises realistic OBD-II traffic at ~50 Hz ────────────

void CanDriver::run_simulator() {
    struct timespec deadline{};
    clock_gettime(CLOCK_MONOTONIC, &deadline);
    const long kPeriodNs = 20'000'000L;  // 50 Hz

    float speed_kph = 60.0f;
    float rpm       = 2200.0f;
    float throttle  = 30.0f;

    while (running_.load(std::memory_order_relaxed)) {
        uint64_t ts = monotonic_ns();

        // Slowly vary values
        speed_kph  = 60.0f + 10.0f * std::sin(static_cast<float>(ts) * 1e-10f);
        rpm        = 2200.0f + 300.0f * sim_randf();
        throttle   = 30.0f + 5.0f * sim_randf();

        // Speed frame
        CanFrame spd_frame{};
        spd_frame.can_id = kOBD2_RESPONSE_ID;
        spd_frame.dlc    = 4;
        spd_frame.data[0] = 3; spd_frame.data[1] = 0x41;
        spd_frame.data[2] = kPID_SPEED;
        spd_frame.data[3] = static_cast<uint8_t>(speed_kph);
        spd_frame.timestamp_ns = ts;
        raw_queue_.push(spd_frame);

        ObdData obd_spd{};
        obd_spd.speed_mps   = speed_kph / 3.6f;
        obd_spd.rpm         = 0; obd_spd.throttle_pct = 0;
        obd_spd.timestamp_ns = ts;
        if (!obd_queue_.push(obd_spd))
            dropped_.fetch_add(1, std::memory_order_relaxed);
        else
            published_.fetch_add(1, std::memory_order_relaxed);

        // RPM + throttle merged
        ObdData obd_rt{};
        obd_rt.speed_mps    = 0;
        obd_rt.rpm          = rpm;
        obd_rt.throttle_pct = throttle;
        obd_rt.timestamp_ns = ts;
        obd_queue_.push(obd_rt);

        deadline.tv_nsec += kPeriodNs;
        if (deadline.tv_nsec >= 1'000'000'000L) {
            deadline.tv_sec++;
            deadline.tv_nsec -= 1'000'000'000L;
        }
        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &deadline, nullptr);
    }
}

}  // namespace veh
