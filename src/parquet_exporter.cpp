/**
 * parquet_exporter.cpp — Arrow/Parquet batch writer for FusedSample stream
 *
 * Behaviour:
 *   - Accumulates FusedSamples from the fusion queue.
 *   - Every `batch_size` samples (default 1000) writes one Parquet row group.
 *   - Every `rotation_interval_s` seconds (default 60) closes the current file
 *     and opens a new one with a timestamp-embedded filename.
 *   - File naming: telemetry_<YYYYMMDD_HHMMSS>_<seq>.parquet
 *
 * Schema (one column per FusedSample field):
 *   timestamp_ns (int64), qw/qx/qy/qz (float), omega_x/y/z (float),
 *   lin_accel_x/y/z (float), pos_x/y/z (double),
 *   speed_mps/rpm/throttle_pct/lidar_density_fwd (float)
 */

#include "veh_telemetry/parquet_exporter.hpp"

#include <arrow/api.h>
#include <arrow/io/api.h>
#include <parquet/arrow/writer.h>
#include <parquet/properties.h>

#include <ctime>
#include <cstring>
#include <sstream>
#include <iomanip>
#include <stdexcept>
#include <filesystem>

namespace veh {

namespace fs = std::filesystem;

// ── Schema ────────────────────────────────────────────────────────────────

static std::shared_ptr<arrow::Schema> make_schema() {
    return arrow::schema({
        arrow::field("timestamp_ns",        arrow::int64()),
        arrow::field("qw",                  arrow::float32()),
        arrow::field("qx",                  arrow::float32()),
        arrow::field("qy",                  arrow::float32()),
        arrow::field("qz",                  arrow::float32()),
        arrow::field("omega_x",             arrow::float32()),
        arrow::field("omega_y",             arrow::float32()),
        arrow::field("omega_z",             arrow::float32()),
        arrow::field("lin_accel_x",         arrow::float32()),
        arrow::field("lin_accel_y",         arrow::float32()),
        arrow::field("lin_accel_z",         arrow::float32()),
        arrow::field("pos_x",               arrow::float64()),
        arrow::field("pos_y",               arrow::float64()),
        arrow::field("pos_z",               arrow::float64()),
        arrow::field("speed_mps",           arrow::float32()),
        arrow::field("rpm",                 arrow::float32()),
        arrow::field("throttle_pct",        arrow::float32()),
        arrow::field("lidar_density_fwd",   arrow::float32()),
    });
}

// ── Helpers ───────────────────────────────────────────────────────────────

static std::string timestamp_str() {
    std::time_t t = std::time(nullptr);
    std::tm tm_buf;
    localtime_r(&t, &tm_buf);
    std::ostringstream oss;
    oss << std::put_time(&tm_buf, "%Y%m%d_%H%M%S");
    return oss.str();
}

// ── ParquetExporter implementation ───────────────────────────────────────

ParquetExporter::ParquetExporter(FusedQueue &queue, const std::string &out_dir,
                                 int batch_size, int rotation_s)
    : queue_(queue), out_dir_(out_dir), batch_size_(batch_size),
      rotation_s_(rotation_s), running_(false), file_seq_(0),
      last_rotation_(std::chrono::steady_clock::now()),
      files_written_(0), rows_written_(0)
{
    fs::create_directories(out_dir_);
    schema_ = make_schema();
}

ParquetExporter::~ParquetExporter() { stop(); }

void ParquetExporter::start() {
    running_.store(true, std::memory_order_relaxed);
    thread_ = std::thread(&ParquetExporter::run, this);
}

void ParquetExporter::stop() {
    running_.store(false, std::memory_order_relaxed);
    if (thread_.joinable()) thread_.join();
    if (!batch_.empty()) flush_batch();
    close_file();
}

std::string ParquetExporter::next_filename() {
    std::ostringstream oss;
    oss << out_dir_ << "/telemetry_"
        << timestamp_str() << "_"
        << std::setw(4) << std::setfill('0') << file_seq_++
        << ".parquet";
    return oss.str();
}

void ParquetExporter::open_file() {
    current_path_ = next_filename();
    auto result = arrow::io::FileOutputStream::Open(current_path_);
    if (!result.ok())
        throw std::runtime_error("ParquetExporter: cannot open " + current_path_);
    out_stream_ = result.ValueOrDie();

    auto props = parquet::WriterProperties::Builder()
        .compression(parquet::Compression::SNAPPY)
        ->build();
    auto arrow_props = parquet::ArrowWriterProperties::Builder()
        .store_schema()
        ->build();

    auto status = parquet::arrow::FileWriter::Open(
        *schema_, arrow::default_memory_pool(),
        out_stream_, props, arrow_props, &writer_);
    if (!status.ok())
        throw std::runtime_error("ParquetExporter: FileWriter::Open failed: " +
                                 status.ToString());

    last_rotation_ = std::chrono::steady_clock::now();
}

void ParquetExporter::close_file() {
    if (writer_) { writer_->Close(); writer_.reset(); }
    if (out_stream_) { out_stream_->Close(); out_stream_.reset(); }
    if (!current_path_.empty())
        fprintf(stderr, "parquet_exporter: closed %s\n", current_path_.c_str());
}

void ParquetExporter::flush_batch() {
    if (batch_.empty()) return;

    if (!writer_) open_file();

    // Build Arrow arrays from batch vector
    arrow::Int64Builder    ts_b;
    arrow::FloatBuilder    qw_b, qx_b, qy_b, qz_b;
    arrow::FloatBuilder    ox_b, oy_b, oz_b;
    arrow::FloatBuilder    ax_b, ay_b, az_b;
    arrow::DoubleBuilder   px_b, py_b, pz_b;
    arrow::FloatBuilder    spd_b, rpm_b, thr_b, ldr_b;

    for (const auto &s : batch_) {
        ts_b.Append(static_cast<int64_t>(s.timestamp_ns));
        qw_b.Append(s.qw); qx_b.Append(s.qx);
        qy_b.Append(s.qy); qz_b.Append(s.qz);
        ox_b.Append(s.omega_x); oy_b.Append(s.omega_y); oz_b.Append(s.omega_z);
        ax_b.Append(s.lin_accel_x); ay_b.Append(s.lin_accel_y); az_b.Append(s.lin_accel_z);
        px_b.Append(s.pos_x); py_b.Append(s.pos_y); pz_b.Append(s.pos_z);
        spd_b.Append(s.speed_mps); rpm_b.Append(s.rpm);
        thr_b.Append(s.throttle_pct); ldr_b.Append(s.lidar_density_fwd);
    }

#define FINISH(b) ([&]{ std::shared_ptr<arrow::Array> a; (b).Finish(&a); return a; }())

    auto table = arrow::Table::Make(schema_, {
        FINISH(ts_b),
        FINISH(qw_b), FINISH(qx_b), FINISH(qy_b), FINISH(qz_b),
        FINISH(ox_b), FINISH(oy_b), FINISH(oz_b),
        FINISH(ax_b), FINISH(ay_b), FINISH(az_b),
        FINISH(px_b), FINISH(py_b), FINISH(pz_b),
        FINISH(spd_b), FINISH(rpm_b), FINISH(thr_b), FINISH(ldr_b),
    });
#undef FINISH

    auto status = writer_->WriteTable(*table, batch_.size());
    if (!status.ok())
        fprintf(stderr, "parquet_exporter: WriteTable error: %s\n",
                status.ToString().c_str());

    rows_written_ += static_cast<uint64_t>(batch_.size());
    batch_.clear();
}

void ParquetExporter::run() {
    while (running_.load(std::memory_order_relaxed)) {
        FusedSample fs;
        if (queue_.pop(fs)) {
            batch_.push_back(fs);
            if (static_cast<int>(batch_.size()) >= batch_size_)
                flush_batch();
        } else {
            // Nothing in queue — sleep briefly then check rotation timer
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }

        // Time-based file rotation
        auto now = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(
            now - last_rotation_).count();
        if (elapsed >= rotation_s_) {
            if (!batch_.empty()) flush_batch();
            close_file();
            files_written_++;
        }
    }
}

}  // namespace veh
