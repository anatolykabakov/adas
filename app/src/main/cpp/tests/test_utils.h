#pragma once

#include <string>
#include <vector>
#include <memory>
#include <chrono>
#include <thread>
#include <cmath>
#include <gtest/gtest.h>
#include "messages.pb.h"

struct TestConfig {
  static constexpr int DEFAULT_TIMEOUT_MS = 5000;
  static constexpr int MESSAGE_DELAY_MS = 100;
  static constexpr int THREAD_STARTUP_DELAY_MS = 200;
  static constexpr int ZMQ_BIND_DELAY_MS = 100;
};

struct TestIMUData {
  float accel_x = 1.0f;
  float accel_y = 2.0f;
  float accel_z = 3.0f;
  float gyro_x = 4.0f;
  float gyro_y = 5.0f;
  float gyro_z = 6.0f;
  float mag_x = 7.0f;
  float mag_y = 8.0f;
  float mag_z = 9.0f;
  long timestamp = 1234567890;
};

class TestUtils {
public:
  template <typename Predicate>
  static bool waitForCondition(Predicate pred, int timeoutMs = TestConfig::DEFAULT_TIMEOUT_MS)
  {
    auto start = std::chrono::steady_clock::now();
    while (std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - start).count() <
           timeoutMs) {
      if (pred()) {
        return true;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    return false;
  }

  static void waitFor(int milliseconds) { std::this_thread::sleep_for(std::chrono::milliseconds(milliseconds)); }

  static TestIMUData createTestIMUData() { return TestIMUData{}; }

  static TestIMUData createTestIMUData(float accel_x, float accel_y, float accel_z, float gyro_x, float gyro_y,
                                       float gyro_z, float mag_x, float mag_y, float mag_z, long timestamp)
  {
    TestIMUData data;
    data.accel_x = accel_x;
    data.accel_y = accel_y;
    data.accel_z = accel_z;
    data.gyro_x = gyro_x;
    data.gyro_y = gyro_y;
    data.gyro_z = gyro_z;
    data.mag_x = mag_x;
    data.mag_y = mag_y;
    data.mag_z = mag_z;
    data.timestamp = timestamp;
    return data;
  }

  static ai::flow::adas::ZMQMessage createTestIMUMessage(const TestIMUData& data)
  {
    ai::flow::adas::ZMQMessage zmq_msg;
    zmq_msg.set_topic("imuData");
    zmq_msg.set_timestamp(data.timestamp);

    auto* imu_data = zmq_msg.mutable_imu_data();
    imu_data->set_accel_x(data.accel_x);
    imu_data->set_accel_y(data.accel_y);
    imu_data->set_accel_z(data.accel_z);
    imu_data->set_gyro_x(data.gyro_x);
    imu_data->set_gyro_y(data.gyro_y);
    imu_data->set_gyro_z(data.gyro_z);
    imu_data->set_mag_x(data.mag_x);
    imu_data->set_mag_y(data.mag_y);
    imu_data->set_mag_z(data.mag_z);
    imu_data->set_timestamp(data.timestamp);

    return zmq_msg;
  }

  static std::vector<uint8_t> serializeMessage(const ai::flow::adas::ZMQMessage& msg)
  {
    std::string serialized;
    msg.SerializeToString(&serialized);
    return std::vector<uint8_t>(serialized.begin(), serialized.end());
  }

  static bool verifyIMUData(const ai::flow::adas::ZMQMessage& message, const TestIMUData& expectedData)
  {
    if (!message.has_imu_data()) {
      return false;
    }

    const auto& imuData = message.imu_data();

    return (std::abs(imuData.accel_x() - expectedData.accel_x) < 0.001f &&
            std::abs(imuData.accel_y() - expectedData.accel_y) < 0.001f &&
            std::abs(imuData.accel_z() - expectedData.accel_z) < 0.001f &&
            std::abs(imuData.gyro_x() - expectedData.gyro_x) < 0.001f &&
            std::abs(imuData.gyro_y() - expectedData.gyro_y) < 0.001f &&
            std::abs(imuData.gyro_z() - expectedData.gyro_z) < 0.001f &&
            std::abs(imuData.mag_x() - expectedData.mag_x) < 0.001f &&
            std::abs(imuData.mag_y() - expectedData.mag_y) < 0.001f &&
            std::abs(imuData.mag_z() - expectedData.mag_z) < 0.001f && imuData.timestamp() == expectedData.timestamp);
  }

  static bool parseMessage(const std::vector<uint8_t>& data, ai::flow::adas::ZMQMessage& msg)
  {
    std::string serialized(data.begin(), data.end());
    return msg.ParseFromString(serialized);
  }
};
