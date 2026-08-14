
#pragma once
#include <fstream>
#include <string>
#include <map>
#include <sstream>
#include <optional>
#include <vector>
#include "adas/panda/can_frame.h"

struct Signal {
  std::string name;
  int start_bit;
  int length;
  bool is_signed;
  double factor;
  double offset;
  double min_val;
  double max_val;
  std::string unit;
};

struct Message {
  uint32_t id;
  std::string name;
  uint8_t length;
  std::map<std::string, Signal> signals;
};

/// Decodes CAN frames into named signals using a DBC description.
class DBSParser {
private:
  std::map<uint32_t, Message> messages;
  uint32_t current_message_id = 0;

public:
  DBSParser(const std::string& filename)
  {
    if (!loadDBC(filename)) {
      throw std::runtime_error("Failed to load DBC file: " + filename);
    }
  }

  std::optional<Message> getMessage(uint32_t message_id)
  {
    auto it = messages.find(message_id);
    if (it != messages.end()) {
      return it->second;
    }
    return std::nullopt;
  }

  const std::map<uint32_t, Message>& getAllMessages() const { return messages; }

  std::optional<double> extractSignal(const can_frame& frame, const std::string& signal_name);

private:
  bool loadDBC(const std::string& filename)
  {
    std::ifstream file(filename);
    if (!file.is_open()) {
      return false;
    }

    std::string line;
    while (std::getline(file, line)) {
      parseLine(line);
    }

    return true;
  }

  void parseLine(const std::string& line)
  {
    if (line.find("BO_") == 0) {
      parseMessage(line);
    } else if (line.find(" SG_") == 0) {
      parseSignal(line);
    }
  }

  void parseMessage(const std::string& line)
  {
    std::istringstream iss(line);
    std::string token;

    iss >> token;
    iss >> token;
    uint32_t id = std::stoul(token);

    iss >> token;
    size_t colon_pos = token.find(':');
    std::string name = token.substr(0, colon_pos);

    iss >> token;
    uint8_t length = static_cast<uint8_t>(std::stoul(token));

    messages[id] = {id, name, length, {}};
    current_message_id = id;
  }

  void parseSignal(const std::string& line)
  {
    std::istringstream iss(line);
    std::string token;

    iss >> token;
    iss >> token;

    std::string signal_name = token;

    size_t pos_start = line.find(" : ");
    if (pos_start == std::string::npos)
      return;

    std::string pos_str = line.substr(pos_start + 3);
    size_t pipe_pos = pos_str.find('|');
    if (pipe_pos == std::string::npos)
      return;

    int start_bit = std::stoi(pos_str.substr(0, pipe_pos));

    std::string length_str = pos_str.substr(pipe_pos + 1);
    size_t at_pos = length_str.find('@');
    if (at_pos == std::string::npos)
      return;

    int length = std::stoi(length_str.substr(0, at_pos));

    size_t paren_start = line.find('(');
    size_t paren_end = line.find(')');
    if (paren_start != std::string::npos && paren_end != std::string::npos) {
      std::string factor_offset = line.substr(paren_start + 1, paren_end - paren_start - 1);
      size_t comma_pos = factor_offset.find(',');

      double factor = 0.01;
      double offset = 0.0;

      if (comma_pos != std::string::npos) {
        factor = std::stod(factor_offset.substr(0, comma_pos));
        offset = std::stod(factor_offset.substr(comma_pos + 1));
      }

      auto it = messages.find(current_message_id);
      if (it != messages.end()) {
        it->second.signals[signal_name] = {signal_name, start_bit, length, false, factor, offset, 0.0, 655.35, "km/h"};
      }
    }
  }
};
