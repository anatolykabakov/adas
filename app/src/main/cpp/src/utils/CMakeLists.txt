# ============================================================================
# Log Parser Utility (Linux only)
# ============================================================================
# Parses CAN frames from log files and extracts odometry data
# Usage: ./log_parser <can_log_file> [dbc_file]
# Output: WHEEL SPEED, STEERING, GEAR, VEHICLE SPEED messages

if(NOT BUILD_FOR_ANDROID)
  add_executable(
    log_parser
    log_parser.cc can_parser.cc
  )

  target_include_directories(
    log_parser
    PRIVATE ${PROJECT_SOURCE_DIR}/include
  )

  # Log parser only needs standard libraries, no ZMQ, protobuf or panda needed
  target_link_libraries(log_parser pthread)

  message(
    STATUS
      "log_parser executable will be built (standalone, no panda dependency)"
  )
endif()
