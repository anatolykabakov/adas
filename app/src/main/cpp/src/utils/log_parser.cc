#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <cstdint>
#include <iomanip>
#include "utils/can_parser.h"
#include "utils/can_logger.h"

void print_frame(int frame_number, const can_frame& frame)
{
  std::cout << "Frame " << frame_number << ": "
            << "Addr=0x" << std::hex << std::uppercase << frame.address << std::dec << ", Len=" << frame.dat.size()
            << ", Data=";

  for (size_t i = 0; i < frame.dat.size(); ++i) {
    if (i > 0)
      std::cout << " ";
    std::cout << std::hex << std::uppercase << std::setfill('0') << std::setw(2)
              << static_cast<int>(static_cast<uint8_t>(frame.dat[i])) << std::dec;
  }
  std::cout << std::endl;
}

void parseWheelSpeedMessage(DBSParser& dbc_parser, uint64_t timestamp, const can_frame& can_frame)
{
  auto esc51_msg = dbc_parser.getMessage(can_frame.address);
  if (esc51_msg.has_value()) {
    auto vr_speed = dbc_parser.extractSignal(can_frame, "VR_Radgeschw");
    auto vl_speed = dbc_parser.extractSignal(can_frame, "VL_Radgeschw");
    auto hr_speed = dbc_parser.extractSignal(can_frame, "HR_Radgeschw");
    auto hl_speed = dbc_parser.extractSignal(can_frame, "HL_Radgeschw");

    if (vr_speed.has_value() && vl_speed.has_value() && hr_speed.has_value() && hl_speed.has_value()) {
      std::cout << "WHEEL SPEED: " << timestamp << "," << vr_speed.value() << "," << vl_speed.value() << ","
                << hr_speed.value() << "," << hl_speed.value() << std::endl;
    } else {
      std::cout << "  No wheel speed signals found or extraction failed" << std::endl;
    }
  } else {
    std::cout << "  ESC_51 message not found in DBC" << std::endl;
  }
}

void parseSteeringMessage(DBSParser& dbc_parser, uint64_t timestamp, const can_frame& can_frame)
{
  auto lwi01_msg = dbc_parser.getMessage(can_frame.address);
  if (lwi01_msg.has_value()) {
    auto steering_angle_abs = dbc_parser.extractSignal(can_frame, "LWI_Lenkradwinkel");
    auto steering_angle_sign = dbc_parser.extractSignal(can_frame, "LWI_VZ_Lenkradwinkel");

    if (steering_angle_abs.has_value() && steering_angle_sign.has_value()) {
      std::cout << "STEERING: " << timestamp << "," << steering_angle_abs.value() << "," << steering_angle_sign.value()
                << std::endl;
    } else {
      std::cout << "  No steering angle signal found or extraction failed" << std::endl;
    }
  } else {
    std::cout << "  LWI_01 message not found in DBC" << std::endl;
  }
}

void parseVehicleSpeedMessage(DBSParser& dbc_parser, uint64_t timestamp, const can_frame& can_frame)
{
  auto esp21_msg = dbc_parser.getMessage(can_frame.address);
  if (esp21_msg.has_value()) {
    auto speed_signal = dbc_parser.extractSignal(can_frame, "ESP_v_Signal");

    if (speed_signal.has_value()) {
      std::cout << "VEHICLE SPEED: " << timestamp << "," << speed_signal.value() << std::endl;
    } else {
      std::cout << "  No vehicle speed signal found or extraction failed" << std::endl;
    }
  } else {
    std::cout << "  ESP_21 message not found in DBC" << std::endl;
  }
}

void parseGearMessage(DBSParser& dbc_parser, uint64_t timestamp, const can_frame& can_frame)
{
  auto gateway73_msg = dbc_parser.getMessage(can_frame.address);
  if (gateway73_msg.has_value()) {
    auto gear_signal = dbc_parser.extractSignal(can_frame, "GE_Fahrstufe");

    if (gear_signal.has_value()) {
      std::string gear_name;
      int gear_value = static_cast<int>(gear_signal.value());

      switch (gear_value) {
        case 0:
          gear_name = "ZWISCHENSTELLUNG";
          break;
        case 1:
          gear_name = "INIT";
          break;
        case 5:
          gear_name = "PARK";
          break;
        case 6:
          gear_name = "REVERSE";
          break;
        case 7:
          gear_name = "NEUTRAL";
          break;
        case 8:
          gear_name = "DRIVE";
          break;
        case 9:
          gear_name = "DRIVE2";
          break;
        case 10:
          gear_name = "ECO";
          break;
        case 13:
          gear_name = "SPORT";
          break;
        case 14:
          gear_name = "SPORT2";
          break;
        case 15:
          gear_name = "ERROR";
          break;
        default:
          gear_name = "UNKNOWN";
          break;
      }

      std::cout << "GEAR: " << timestamp << "," << gear_name << "," << gear_value << std::endl;
    } else {
      std::cout << "  No gear signal found or extraction failed" << std::endl;
    }
  } else {
    std::cout << "  Gateway_73 message not found in DBC" << std::endl;
  }
}

void parseQFK01Message(DBSParser& dbc_parser, uint64_t timestamp, const can_frame& can_frame)
{
  auto qfk01_msg = dbc_parser.getMessage(can_frame.address);
  if (qfk01_msg.has_value()) {
    auto curvature = dbc_parser.extractSignal(can_frame, "Curvature");
    auto steering_angle = dbc_parser.extractSignal(can_frame, "Steering_Angle");
    auto latcon_hca_accept = dbc_parser.extractSignal(can_frame, "LatCon_HCA_Accept");
    auto latcon_hca_status = dbc_parser.extractSignal(can_frame, "LatCon_HCA_Status");
    auto hca_no_override = dbc_parser.extractSignal(can_frame, "HCA_No_Override");
    auto curvature_vz = dbc_parser.extractSignal(can_frame, "Curvature_VZ");
    auto steering_angle_vz = dbc_parser.extractSignal(can_frame, "Steering_Angle_VZ");

    std::cout << "QFK_01: " << timestamp;

    if (curvature.has_value()) {
      std::cout << ", Curvature=" << std::fixed << std::setprecision(6) << curvature.value() << " rad/m";
    }

    if (steering_angle.has_value()) {
      std::cout << ", Steering_Angle=" << steering_angle.value() << " (raw)";

      double real_angle = steering_angle.value() * 0.00906;

      std::cout << " (" << std::fixed << std::setprecision(2) << real_angle << "°)";
    }

    if (latcon_hca_accept.has_value()) {
      std::cout << ", HCA_Accept=" << static_cast<int>(latcon_hca_accept.value());
    }

    if (latcon_hca_status.has_value()) {
      std::cout << ", HCA_Status=" << static_cast<int>(latcon_hca_status.value());
    }

    if (hca_no_override.has_value()) {
      std::cout << ", HCA_No_Override=" << static_cast<int>(hca_no_override.value());
    }

    if (curvature_vz.has_value()) {
      std::cout << ", Curvature_VZ=" << static_cast<int>(curvature_vz.value());
    }

    if (steering_angle_vz.has_value()) {
      std::cout << ", Steering_Angle_VZ=" << static_cast<int>(steering_angle_vz.value());
    }

    std::cout << std::endl;

    if (latcon_hca_accept.has_value() && latcon_hca_status.has_value()) {
      int accept = static_cast<int>(latcon_hca_accept.value());
      int status = static_cast<int>(latcon_hca_status.value());

      std::cout << "  QFK_01 Control State: ";
      if (accept == 2 && status == 3) {
        std::cout << "ACTIVE_CONTROL";
      } else if (accept == 1 && status == 2) {
        std::cout << "PASSIVE_MODE";
      } else {
        std::cout << "UNKNOWN (" << accept << "," << status << ")";
      }
      std::cout << std::endl;
    }

  } else {
    std::cout << "  QFK_01 message not found in DBC" << std::endl;
  }
}

int main(int argc, char** argv)
{
  if (argc < 2) {
    std::cout << "Usage: " << argv[0] << " <log_file> [dbc_file]" << std::endl;
    std::cout << "  log_file: Path to CAN log file" << std::endl;
    std::cout << "  dbc_file: Path to DBC file (optional, defaults to vw_mqb_2010.dbc)" << std::endl;
    return 1;
  }

  std::string log_file = argv[1];
  std::string dbc_file = (argc > 2) ? argv[2] : "vw_mqb_2010.dbc";

  DBSParser dbc_parser(dbc_file);

  std::ifstream log_stream(log_file);
  if (!log_stream.is_open()) {
    std::cerr << "Error: Cannot open log file: " << log_file << std::endl;
    return 1;
  }

  std::string line;

  while (std::getline(log_stream, line)) {
    auto can_frame_with_timestamp_opt = CanLogger::parseCanFrameFromLine(line);
    if (!can_frame_with_timestamp_opt.has_value()) {
      continue;
    }

    const auto& [timestamp, can_frame] = can_frame_with_timestamp_opt.value();

    switch (can_frame.address) {
      case 0xFC:
        parseWheelSpeedMessage(dbc_parser, timestamp, can_frame);
        break;

      case 0x86:
        parseSteeringMessage(dbc_parser, timestamp, can_frame);
        break;

      case 0xFD:
        parseVehicleSpeedMessage(dbc_parser, timestamp, can_frame);
        break;

      case 0x3DC:
        parseGearMessage(dbc_parser, timestamp, can_frame);
        break;

      default:

        break;
    }
  }
  return 0;
}
