#include "utils/can_parser.h"
#include "panda/can_frame.h"

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

  int bytes_needed = (signal.start_bit + signal.length + 7) / 8;
  if (frame.dat.length() < bytes_needed) {
    return std::nullopt;
  }

  uint64_t raw_value = 0;
  for (int i = 0; i < signal.length; i++) {
    int bit_pos = signal.start_bit + i;
    int byte_pos = bit_pos / 8;
    int bit_in_byte = bit_pos % 8;

    if (byte_pos < frame.dat.length() && (frame.dat[byte_pos] & (1 << bit_in_byte))) {
      raw_value |= (1ULL << i);
    }
  }

  double value = raw_value * signal.factor + signal.offset;
  return value;
}
