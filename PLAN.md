# Veh Telemetry

## Project Overview
Real-time C++ pipeline on ARM hardware for processing LiDAR point clouds, IMU data, and CAN bus sensor data. Uses multithreaded lock-free queues for inter-stage data transfer with sub-10ms per-frame latency. Integrates with Databricks for large-scale offline diagnostics and analytics.

## Tech Stack
- **Language:** C++17
- **Platform:** ARM (cross-compiled), Linux
- **Build:** CMake 3.20+, ARM cross-compilation toolchain
- **Sensor I/O:** SocketCAN, serial (UART for IMU)
- **Data Structures:** Lock-free SPSC ring buffers
- **Testing:** Google Test
- **Analytics:** Databricks (PySpark), Parquet file export
- **Profiling:** perf, Valgrind

## Architecture Overview
```
┌───────────┐   ┌───────────┐   ┌───────────┐
│  LiDAR    │   │   IMU     │   │  CAN Bus  │
│  Driver   │   │  Driver   │   │  Driver   │
└─────┬─────┘   └─────┬─────┘   └─────┬─────┘
      │ SPSC          │ SPSC          │ SPSC
      ▼               ▼               ▼
┌─────────────────────────────────────────────┐
│           Fusion Pipeline                    │
│  (timestamp align → fuse → transform)       │
└────────────────────┬────────────────────────┘
                     │
              ┌──────┴──────┐
              │  Output     │  → Parquet files → Databricks
              │  Logger     │  → Real-time display
              └─────────────┘
```

## Phase 1: Lock-Free Queues & Sensor Data Types
**Goal:** Implement SPSC ring buffers and define sensor data structures.

### Tasks
1. Project structure:
   ```
   embeddedVehicleTelemetryPipeline/
   ├── CMakeLists.txt
   ├── include/
   │   ├── spsc_queue.h         # Lock-free SPSC ring buffer
   │   ├── sensor_types.h       # LiDAR, IMU, CAN data structs
   │   ├── lidar_driver.h
   │   ├── imu_driver.h
   │   ├── can_driver.h
   │   ├── fusion_pipeline.h
   │   └── data_logger.h
   ├── src/
   │   ├── lidar_driver.cpp
   │   ├── imu_driver.cpp
   │   ├── can_driver.cpp
   │   ├── fusion_pipeline.cpp
   │   ├── data_logger.cpp
   │   └── main.cpp
   ├── tests/
   ├── scripts/
   │   └── databricks_analysis.py
   └── toolchain/
       └── arm-toolchain.cmake
   ```
2. `include/spsc_queue.h`:
   - Template `SPSCQueue<T, N>`: fixed-size, cache-line-padded head/tail atomics, power-of-2 capacity
   - `bool try_push(const T& item)`, `bool try_pop(T& item)`, `size_t size() const`
   - Memory ordering: `memory_order_release` on push, `memory_order_acquire` on pop
3. `include/sensor_types.h`:
   - `struct LiDARFrame { uint64_t timestamp_ns; uint32_t num_points; struct Point { float x, y, z, intensity; } points[MAX_POINTS]; }`
   - `struct IMUReading { uint64_t timestamp_ns; float accel_x, accel_y, accel_z; float gyro_x, gyro_y, gyro_z; float mag_x, mag_y, mag_z; }`
   - `struct CANMessage { uint64_t timestamp_ns; uint32_t can_id; uint8_t dlc; uint8_t data[8]; }`
   - `struct FusedFrame { uint64_t timestamp_ns; LiDARFrame lidar; IMUReading imu; std::vector<CANMessage> can_messages; }`
4. Tests: SPSC queue correctness with producer/consumer threads, verify no data loss.

## Phase 2: Sensor Drivers
**Goal:** Implement drivers for LiDAR, IMU, and CAN bus data acquisition.

### Tasks
1. `include/lidar_driver.h` / `src/lidar_driver.cpp`:
   - `class LiDARDriver`:
     - `LiDARDriver(const std::string& device, SPSCQueue<LiDARFrame>& output_queue)`
     - `void start()` — spawns reader thread, reads UDP packets (Velodyne-style format), parses point clouds, pushes to queue
     - `void stop()`
     - Simulated mode: generates synthetic point clouds for testing without hardware
2. `include/imu_driver.h` / `src/imu_driver.cpp`:
   - `class IMUDriver`:
     - Reads UART serial port at configured baud rate
     - Parses binary protocol (e.g., BNO055 format)
     - Pushes `IMUReading` to SPSC queue at 100Hz
     - Simulated mode: generates random walk IMU data
3. `include/can_driver.h` / `src/can_driver.cpp`:
   - `class CANDriver`:
     - Uses SocketCAN (`socket(PF_CAN, SOCK_RAW, CAN_RAW)`)
     - `bind` to `can0` interface
     - `read()` loop parsing `can_frame` structs
     - Filters: configurable CAN ID whitelist
     - Simulated mode: generates vehicle speed, RPM, temperature messages
4. Each driver: thread with `pthread_setaffinity_np` for CPU pinning, configurable via constructor.

## Phase 3: Fusion Pipeline
**Goal:** Align timestamps and fuse multi-sensor data into unified frames.

### Tasks
1. `include/fusion_pipeline.h` / `src/fusion_pipeline.cpp`:
   - `class FusionPipeline`:
     - `FusionPipeline(SPSCQueue<LiDARFrame>& lidar_q, SPSCQueue<IMUReading>& imu_q, SPSCQueue<CANMessage>& can_q, SPSCQueue<FusedFrame>& output_q)`
     - `void run()` — main loop:
       1. Pop latest LiDAR frame (this is the "anchor" at ~10Hz)
       2. Find nearest IMU reading by timestamp (interpolate if needed)
       3. Collect CAN messages within the frame's time window
       4. Assemble `FusedFrame`, push to output queue
     - Timestamp alignment: nearest-neighbor within 5ms tolerance
     - Handles sensor dropout: if IMU missing, use last valid reading with stale flag
   - Pipeline metrics: frames processed, dropped frames, max latency
2. `src/main.cpp`:
   - Parse CLI args: device paths, simulation mode, output directory
   - Create SPSC queues, drivers, fusion pipeline, data logger
   - Start all threads, run until SIGINT
   - Print pipeline statistics on shutdown
3. Tests: feed known timestamped data through pipeline, verify correct alignment and fusion.

## Phase 4: Data Logger & Parquet Export
**Goal:** Log fused frames to disk in Parquet format for offline analysis.

### Tasks
1. `include/data_logger.h` / `src/data_logger.cpp`:
   - `class DataLogger`:
     - `DataLogger(SPSCQueue<FusedFrame>& input_q, const std::string& output_dir)`
     - `void run()` — consumes frames, writes to Parquet files
     - Uses Apache Arrow C++ library for Parquet serialization
     - File rotation: new file every N frames or M minutes
     - Schema: flatten FusedFrame into columnar format (timestamp, lidar_x[], lidar_y[], imu_accel_x, can_speed, etc.)
2. Add Arrow/Parquet to CMakeLists.txt as dependency.
3. `scripts/databricks_analysis.py`:
   - PySpark notebook for Databricks:
     - Load Parquet files from cloud storage
     - Compute per-frame statistics: point cloud density, IMU drift, CAN message rate
     - Detect anomalies: sudden acceleration, sensor dropout intervals
     - Visualization: 3D scatter plots of point clouds, time series of IMU data
4. Integration test: run full pipeline for 60s in simulation mode, verify Parquet output readable by PySpark.

## Phase 5: Performance Optimization & ARM Cross-Compilation
**Goal:** Optimize for sub-10ms latency and cross-compile for ARM target.

### Tasks
1. `toolchain/arm-toolchain.cmake`:
   ```cmake
   set(CMAKE_SYSTEM_NAME Linux)
   set(CMAKE_SYSTEM_PROCESSOR aarch64)
   set(CMAKE_C_COMPILER aarch64-linux-gnu-gcc)
   set(CMAKE_CXX_COMPILER aarch64-linux-gnu-g++)
   ```
2. Build: `cmake -DCMAKE_TOOLCHAIN_FILE=toolchain/arm-toolchain.cmake ..`
3. Performance optimizations:
   - Thread affinity: pin each driver to dedicated core
   - SPSC queue sizing: profile to find optimal capacity (avoid producer blocking)
   - Memory pre-allocation: avoid runtime `malloc` in hot path
   - Compile with `-O3 -march=armv8-a` for ARM NEON auto-vectorization
4. Latency measurement:
   - Instrument pipeline: `clock_gettime(CLOCK_MONOTONIC)` at each stage
   - Report per-frame latency histogram: min, p50, p95, p99, max
   - Target: < 10ms from sensor read to output queue
5. Stress test: simulate 10Hz LiDAR (100K points/frame), 100Hz IMU, 1000Hz CAN bus.
