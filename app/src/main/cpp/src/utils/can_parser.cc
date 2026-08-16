#include "adas/utils/can_parser.h"
#include "adas/panda/can_frame.h"

std::optional<double> DBSParser::extractSignal(const can_frame& frame, const std::string& signal_name)
{
  auto msg_it = messages.find(frame.address);
  if (msg_it == messages.end()) {
    return std::nullopt;
  }

  auto sig_it = msg_it->second.signals.find(signal_name);
  if (sig_it == msg_it->second.signals.end()) {
    return std::nullopt;
  }

  const Signal& signal = sig_it->second;

  const auto* bytes = reinterpret_cast<const uint8_t*>(frame.dat.data());
  const int frame_len = static_cast<int>(frame.dat.length());

  uint64_t raw_value = 0;
  if (signal.is_big_endian) {
    // Motorola. `start_bit` names the most significant bit in the DBC's sawtooth numbering; turn it
    // into a plain MSB-first index across the frame and walk down from there.
    const int msb_index = (signal.start_bit / 8) * 8 + (7 - (signal.start_bit % 8));
    if ((msb_index + signal.length + 7) / 8 > frame_len) {
      return std::nullopt;
    }
    for (int i = 0; i < signal.length; i++) {
      const int index = msb_index + i;
      const int byte_pos = index / 8;
      const int bit_in_byte = 7 - (index % 8);
      if (byte_pos >= frame_len) {
        return std::nullopt;
      }
      if (bytes[byte_pos] & (1u << bit_in_byte)) {
        raw_value |= 1ULL << (signal.length - 1 - i);
      }
    }
  } else {
    // Intel. `start_bit` names the least significant bit and the value ascends from it.
    if ((signal.start_bit + signal.length + 7) / 8 > frame_len) {
      return std::nullopt;
    }
    for (int i = 0; i < signal.length; i++) {
      const int bit_pos = signal.start_bit + i;
      const int byte_pos = bit_pos / 8;
      const int bit_in_byte = bit_pos % 8;
      if (byte_pos >= frame_len) {
        return std::nullopt;
      }
      if (bytes[byte_pos] & (1u << bit_in_byte)) {
        raw_value |= 1ULL << i;
      }
    }
  }

  // Sign extension before scaling: a negative steering angle is a left turn, and reading it as a very
  // large positive one would turn the wheel the other way.
  if (signal.is_signed && signal.length < 64 && (raw_value & (1ULL << (signal.length - 1))) != 0) {
    raw_value |= ~((1ULL << signal.length) - 1);
    const double value_signed = static_cast<double>(static_cast<int64_t>(raw_value)) * signal.factor + signal.offset;
    return value_signed;
  }

  double value = raw_value * signal.factor + signal.offset;
  return value;
}
