#pragma once

#include <fstream>
#include <string>
#include <chrono>
#include <iomanip>
#include <sstream>
#include "adas/panda/can_frame.h"

struct CanFrameWithTimestamp {
  uint64_t timestamp;
  can_frame frame;
};

class CanLogger {
private:
  std::ofstream log_file_;
  std::string filename_;

  uint64_t getCurrentTimestamp()
  {
    auto now = std::chrono::system_clock::now();
    return std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()).count();
  }

  std::string formatHexData(const std::string& data)
  {
    std::stringstream ss;
    ss << std::hex << std::uppercase << std::setfill('0');
    for (size_t i = 0; i < data.size(); ++i) {
      if (i > 0)
        ss << " ";
      ss << std::setw(2) << static_cast<int>(static_cast<uint8_t>(data[i]));
    }
    return ss.str();
  }

public:
  CanLogger(const std::string& filename) : filename_(filename)
  {
    std::ifstream check_file(filename_);
    if (!check_file.good()) {
      std::ofstream create_file(filename_);
      create_file.close();
    }
    check_file.close();

    log_file_.open(filename_, std::ios::out | std::ios::app);
  }

  CanLogger()
  {
    auto now = std::chrono::system_clock::now();
    auto time_t = std::chrono::system_clock::to_time_t(now);
    std::stringstream ss;
    ss << "can_log_" << std::put_time(std::localtime(&time_t), "%Y%m%d_%H%M%S") << ".csv";
    filename_ = ss.str();

    std::ifstream check_file(filename_);
    if (!check_file.good()) {
      std::ofstream create_file(filename_);
      create_file.close();
    }
    check_file.close();

    log_file_.open(filename_, std::ios::out | std::ios::app);
  }

  ~CanLogger()
  {
    if (log_file_.is_open()) {
      log_file_.close();
    }
  }

  void logCanFrame(const can_frame& frame)
  {
    if (!log_file_.is_open())
      return;

    log_file_ << getCurrentTimestamp() << ","
              << "0x" << std::hex << std::uppercase << std::setfill('0') << std::setw(3) << frame.address << ","
              << std::dec << frame.busTime << "," << static_cast<int>(frame.src) << ","
              << static_cast<int>(frame.dat.size()) << "," << formatHexData(frame.dat) << std::endl;

    log_file_.flush();
  }

  const std::string& getFilename() const { return filename_; }

  bool isOpen() const { return log_file_.is_open(); }

  static std::optional<CanFrameWithTimestamp> parseCanFrameFromLine(const std::string& log_line)
  {
    if (log_line.empty())
      return std::nullopt;

    std::istringstream line_stream(log_line);
    std::string timestamp, address, bus_time, src, length, data;

    if (!std::getline(line_stream, timestamp, ','))
      return std::nullopt;
    if (!std::getline(line_stream, address, ','))
      return std::nullopt;
    if (!std::getline(line_stream, bus_time, ','))
      return std::nullopt;
    if (!std::getline(line_stream, src, ','))
      return std::nullopt;
    if (!std::getline(line_stream, length, ','))
      return std::nullopt;
    if (!std::getline(line_stream, data))
      return std::nullopt;

    try {
      CanFrameWithTimestamp result;

      result.timestamp = std::stoull(timestamp);

      if (address.length() < 3 || address.substr(0, 2) != "0x") {
        return std::nullopt;
      }
      result.frame.address = std::stoul(address.substr(2), nullptr, 16);
      result.frame.busTime = std::stol(bus_time);
      result.frame.src = std::stol(src);
      result.frame.dat = parseHexData(data);

      return result;
    } catch (const std::exception&) {
      return std::nullopt;
    }
  }

private:
  static std::string parseHexData(const std::string& hex_data)
  {
    std::string result;
    std::istringstream hex_stream(hex_data);
    std::string hex_byte;

    while (std::getline(hex_stream, hex_byte, ' ')) {
      if (!hex_byte.empty()) {
        try {
          uint8_t byte_value = static_cast<uint8_t>(std::stoul(hex_byte, nullptr, 16));
          result += static_cast<char>(byte_value);
        } catch (const std::exception&) {
        }
      }
    }

    return result;
  }
};
