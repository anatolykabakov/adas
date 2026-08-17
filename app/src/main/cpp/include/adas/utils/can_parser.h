
#pragma once
#include <cstdio>
#include <stdexcept>
#include <fstream>
#include <string>
#include <map>
#include <sstream>
#include <optional>
#include <vector>
#include "adas/panda/can_frame.h"

struct Signal {
  std::string name;
  /// True for `@0` (Motorola, most significant bit first). VW writes `@1`, Toyota writes `@0`, and a
  /// parser that assumes one silently returns nonsense for the other rather than failing.
  bool is_big_endian = false;
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
    int line_no = 0;
    int skipped = 0;
    while (std::getline(file, line)) {
      ++line_no;
      try {
        parseLine(line);
      } catch (const std::exception& e) {
        // One line we cannot read must not void the whole database. A DBC that fails to load leaves
        // the car undecoded, which looks exactly like a car that is not talking — and that is the
        // wrong thing to conclude from a spacing difference on an unrelated message.
        ++skipped;
        if (skipped <= 5)
          fprintf(stderr, "DBC %s:%d skipped (%s): %s\n", filename.c_str(), line_no, e.what(), line.c_str());
      }
    }
    if (skipped > 0)
      fprintf(stderr, "DBC %s: %d line(s) skipped, %zu messages loaded\n", filename.c_str(), skipped, messages.size());

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

    // The colon may be attached to the name or stand on its own: both `BO_ 170 WHEEL_SPEEDS: 8 XXX`
    // and `BO_ 869 DSU_CRUISE : 7 DSU` are written by real tools, and the second shape used to throw
    // on `stoul(":")` and take the entire file down with it.
    iss >> token;
    const size_t colon_pos = token.find(':');
    std::string name = token.substr(0, colon_pos);
    if (colon_pos == std::string::npos) {
      iss >> token;
      if (token != ":")
        throw std::runtime_error("expected ':' after the message name");
    }

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

    // "@0+" / "@1-" — byte order then sign, exactly two characters after the '@'.
    const std::string order_sign = length_str.substr(at_pos + 1, 2);
    const bool is_big_endian = !order_sign.empty() && order_sign[0] == '0';
    const bool is_signed = order_sign.size() > 1 && order_sign[1] == '-';

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
        it->second.signals[signal_name] = {signal_name, is_big_endian, start_bit, length, is_signed,
                                           factor,      offset,        0.0,       655.35, "km/h"};
      }
    }
  }
};
