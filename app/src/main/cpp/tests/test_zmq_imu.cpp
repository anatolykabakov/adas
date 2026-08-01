#include <gtest/gtest.h>
#include <memory>
#include <thread>
#include <chrono>
#include <atomic>
#include <vector>
#include <string>
#include <mutex>
#include <zmq.hpp>
#include "utils/logger.h"
#include "adas_app.h"

#include "test_utils.h"

TEST(ZMQIMUTest, StartAdasAppAndConnect)
{
  LOGI("Starting test: StartAdasAppAndConnect");

  GTEST_SKIP() << "Test requires USB Panda device, skipping on Linux";

  AdasApp app(-1);
  bool started = app.start();
  EXPECT_TRUE(started) << "AdasApp should start successfully";
  LOGI("AdasApp started successfully");

  std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  LOGI("AdasApp initialization complete");

  zmq::context_t context(1);

  zmq::socket_t imu_publisher(context, ZMQ_PUB);
  imu_publisher.connect("tcp://127.0.0.1:5555");
  LOGI("Test IMU publisher connected to tcp://127.0.0.1:5555");
  std::this_thread::sleep_for(std::chrono::milliseconds(200));

  zmq::socket_t subscriber(context, ZMQ_SUB);
  subscriber.connect("tcp://127.0.0.1:5556");
  subscriber.set(zmq::sockopt::subscribe, "");
  subscriber.set(zmq::sockopt::rcvtimeo, 1000);
  LOGI("Test subscriber connected to tcp://127.0.0.1:5556");

  std::this_thread::sleep_for(std::chrono::milliseconds(200));
  LOGI("Connections established, preparing to send message");

  ai::flow::adas::ZMQMessage zmq_msg;
  zmq_msg.set_topic("imuData");
  zmq_msg.set_timestamp(
      std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch())
          .count());

  auto* imu_data = zmq_msg.mutable_imu_data();
  imu_data->set_accel_x(1.1);
  imu_data->set_accel_y(2.2);
  imu_data->set_accel_z(3.3);
  imu_data->set_gyro_x(4.4);
  imu_data->set_gyro_y(5.5);
  imu_data->set_gyro_z(6.6);
  imu_data->set_mag_x(0.1);
  imu_data->set_mag_y(0.2);
  imu_data->set_mag_z(0.3);
  imu_data->set_timestamp(zmq_msg.timestamp());

  std::string serialized_data;
  zmq_msg.SerializeToString(&serialized_data);

  const std::string topic = "sensors/imu";
  zmq::message_t topic_frame(topic.data(), topic.size());
  zmq::message_t payload_frame(serialized_data.data(), serialized_data.size());
  imu_publisher.send(topic_frame, zmq::send_flags::sndmore);
  imu_publisher.send(payload_frame, zmq::send_flags::none);
  LOGI("Sent protobuf IMU message to AdasApp: accel(%.1f, %.1f, %.1f), gyro(%.1f, %.1f, %.1f)", imu_data->accel_x(),
       imu_data->accel_y(), imu_data->accel_z(), imu_data->gyro_x(), imu_data->gyro_y(), imu_data->gyro_z());

  LOGI("Waiting for AdasApp to process message...");
  std::this_thread::sleep_for(std::chrono::milliseconds(500));

  LOGI("Attempting to receive message from AdasApp...");
  zmq::message_t msg;
  LOGI("Calling subscriber.recv()...");
  auto result = subscriber.recv(msg);
  LOGI("subscriber.recv() returned: %s", result ? "true" : "false");

  if (result) {
    LOGI("Message received successfully! Size: %ld", msg.size());
  } else {
    LOGE("Failed to receive message from AdasApp");
  }

  EXPECT_TRUE(result) << "Message should be received";
  std::string message_data(static_cast<char*>(msg.data()), msg.size());
  EXPECT_FALSE(message_data.empty()) << "Message should not be empty";

  ai::flow::adas::ZMQMessage received_msg;
  EXPECT_TRUE(received_msg.ParseFromString(message_data)) << "Should be able to parse received protobuf message";

  EXPECT_TRUE(received_msg.has_imu_data()) << "Received message should contain IMU data";

  const auto& received_imu = received_msg.imu_data();
  LOGI("Received IMU data: accel(%.1f, %.1f, %.1f), gyro(%.1f, %.1f, %.1f), mag(%.1f, %.1f, %.1f)",
       received_imu.accel_x(), received_imu.accel_y(), received_imu.accel_z(), received_imu.gyro_x(),
       received_imu.gyro_y(), received_imu.gyro_z(), received_imu.mag_x(), received_imu.mag_y(), received_imu.mag_z());

  EXPECT_NEAR(1.1, received_imu.accel_x(), 0.01) << "Accel X should match";
  EXPECT_NEAR(2.2, received_imu.accel_y(), 0.01) << "Accel Y should match";
  EXPECT_NEAR(3.3, received_imu.accel_z(), 0.01) << "Accel Z should match";
  EXPECT_NEAR(4.4, received_imu.gyro_x(), 0.01) << "Gyro X should match";
  EXPECT_NEAR(5.5, received_imu.gyro_y(), 0.01) << "Gyro Y should match";
  EXPECT_NEAR(6.6, received_imu.gyro_z(), 0.01) << "Gyro Z should match";
  EXPECT_NEAR(0.1, received_imu.mag_x(), 0.01) << "Mag X should match";
  EXPECT_NEAR(0.2, received_imu.mag_y(), 0.01) << "Mag Y should match";
  EXPECT_NEAR(0.3, received_imu.mag_z(), 0.01) << "Mag Z should match";

  LOGI("Test completed successfully, stopping AdasApp");
  app.stop();
  LOGI("Test finished");
}
