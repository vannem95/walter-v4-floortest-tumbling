#pragma once

#include <chrono>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>
#include <Eigen/Dense>

class MatLogger {
public:
    void init(const std::string& log_dir = "/tmp/walter_logs", const std::string& prefix = "osc_log") {
        std::filesystem::create_directories(log_dir);
        auto now = std::chrono::system_clock::now();
        std::time_t now_c = std::chrono::system_clock::to_time_t(now);
        std::tm tm_buf;
        localtime_r(&now_c, &tm_buf);

        std::ostringstream oss;
        oss << log_dir << "/" << prefix << "_"
            << std::put_time(&tm_buf, "%Y_%m_%d-%H_%M_%S");
        base_path_ = oss.str();

        bin_file_.open(base_path_ + ".bin", std::ios::out | std::ios::binary);
        // 64 KB buffer so disk I/O never blocks the 200 Hz loop
        bin_buffer_.resize(65536);
        bin_file_.rdbuf()->pubsetbuf(bin_buffer_.data(), bin_buffer_.size());

        start_time_ = std::chrono::steady_clock::now();
        header_locked_ = false;
        initialized_ = true;
        std::cout << "[MatLogger] Logging to: " << base_path_ << ".bin\n";
    }

    ~MatLogger() {
        if (bin_file_.is_open()) bin_file_.close();
    }

    // Call at the start of every 200 Hz control tick
    void begin_tick(std::vector<double>* ros_vec = nullptr) {
        row_values_.clear();
        ros_mirror_ = ros_vec;
        if (ros_mirror_) ros_mirror_->clear();

        if (! initialized_) init();

        // Automatically record elapsed time (seconds) as the very first column in the .bin file
        double t_sec = std::chrono::duration<double>(
            std::chrono::steady_clock::now() - start_time_).count();
        if (!header_locked_) col_names_.push_back("t");
        row_values_.push_back(t_sec); // Note: not pushed to ros_mirror_ so ROS indices stay 1..126!
    }

    // Scalar variable
    void add(const std::string& name, double val, bool mirror_to_ros = true) {
        if (!header_locked_) col_names_.push_back(name);
        row_values_.push_back(val);
        if (ros_mirror_ && mirror_to_ros) ros_mirror_->push_back(val);
    }

    // 4-leg helper: automatically creates <prefix>_tl, <prefix>_tr, <prefix>_hl, <prefix>_hr
    void add_legs(const std::string& prefix, double tl, double tr, double hl, double hr, bool mirror_to_ros = true) {
        add(prefix + "_tl", tl, mirror_to_ros);
        add(prefix + "_tr", tr, mirror_to_ros);
        add(prefix + "_hl", hl, mirror_to_ros);
        add(prefix + "_hr", hr, mirror_to_ros);
    }

    // 8-joint helper: [rlh, rlk, rrh, rrk, flh, flk, frh, frk]
    template <typename Derived>
    void add_joints8(const std::string& prefix, const Eigen::MatrixBase<Derived>& v, bool mirror_to_ros = true) {
        static const char* jnames[8] = {"rlh", "rlk", "rrh", "rrk", "flh", "flk", "frh", "frk"};
        for (int i = 0; i < 8; ++i) {
            add(prefix + "_" + jnames[i], static_cast<double>(v(i)), mirror_to_ros);
        }
    }

    // 8-wheel helper: [tlf, tlr, trf, trr, hlf, hlr, hrf, hrr]
    template <typename Derived>
    void add_wheels8(const std::string& prefix, const Eigen::MatrixBase<Derived>& v, bool mirror_to_ros = true) {
        static const char* wnames[8] = {"tlf", "tlr", "trf", "trr", "hlf", "hlr", "hrf", "hrr"};
        for (int i = 0; i < 8; ++i) {
            add(prefix + "_" + wnames[i], static_cast<double>(v(i)), mirror_to_ros);
        }
    }

    // Call at the end of every 200 Hz control tick
    void end_tick() {
        if (!header_locked_) {
            std::ofstream hdr(base_path_ + ".hdr");
            for (size_t i = 0; i < col_names_.size(); ++i) {
                hdr << col_names_[i] << "\n";
            }
            hdr.close();
            header_locked_ = true;
        }
        bin_file_.write(reinterpret_cast<const char*>(row_values_.data()),
                        row_values_.size() * sizeof(double));
    }

private:
    bool initialized_ = false;
    bool header_locked_ = false;
    std::string base_path_;
    std::ofstream bin_file_;
    std::vector<char> bin_buffer_;
    std::vector<std::string> col_names_;
    std::vector<double> row_values_;
    std::vector<double>* ros_mirror_ = nullptr;
    std::chrono::steady_clock::time_point start_time_;
};