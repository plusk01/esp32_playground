/**
 * @file serial_driver.cpp
 * @brief Serial communication driver for esp32-imu
 * @author Parker Lusk <plusk@mit.edu>
 * @date 21 Nov 2020
 */

#include <iostream>
#include <stdexcept>

#include <async_comm/serial.h>

#include "esp32imu/serial_driver.h"

namespace acl {
namespace esp32imu {

SerialDriver::SerialDriver(const std::string& port, uint32_t baud)
{
  using namespace std::placeholders;

  serial_.reset(new async_comm::Serial(port, baud));
  serial_->register_receive_callback(
            std::bind(&SerialDriver::callback, this, _1, _2));

  if (!serial_->init()) {
    throw std::runtime_error("Could not open serial port '" + port + "'");
  }
}

// ----------------------------------------------------------------------------

SerialDriver::~SerialDriver()
{
  serial_->close();
}

// ----------------------------------------------------------------------------
// Send message methods
// ----------------------------------------------------------------------------

void SerialDriver::sendRate(uint16_t frequency)
{
  // put into standard message
  esp_serial_rate_msg_t msg;
  msg.frequency = frequency;

  uint8_t buf[ESP_SERIAL_MAX_MESSAGE_LEN];
  const size_t len = esp_serial_rate_msg_send_to_buffer(buf, &msg);
  serial_->send_bytes(buf, len);
}

// ----------------------------------------------------------------------------
// Callback stuff
// ----------------------------------------------------------------------------

void SerialDriver::registerCallbackStatus(CallbackStatus cb)
{
  std::lock_guard<std::mutex> lock(mtx_);
  cb_status_ = cb;
}

// ----------------------------------------------------------------------------

void SerialDriver::registerCallbackIMU(CallbackIMU cb)
{
  std::lock_guard<std::mutex> lock(mtx_);
  cb_imu_ = cb;
}

// ----------------------------------------------------------------------------

void SerialDriver::registerCallbackRate(CallbackRate cb)
{
  std::lock_guard<std::mutex> lock(mtx_);
  cb_rate_ = cb;
}

// ----------------------------------------------------------------------------

void SerialDriver::unregisterCallbacks()
{
  std::lock_guard<std::mutex> lock(mtx_);
  cb_imu_ = nullptr;
  cb_status_ = nullptr;
  cb_rate_ = nullptr;
}


// ----------------------------------------------------------------------------
// Private Methods
// ----------------------------------------------------------------------------

void SerialDriver::callback(const uint8_t * data, size_t len)
{
  esp_serial_message_t msg;
  for (size_t i=0; i<len; ++i) {
    if (esp_serial_parse_byte(data[i], &msg)) {

      switch (msg.type) {
        case ESP_SERIAL_MSG_IMU:
          handleIMUMsg(msg);
          break;
        case ESP_SERIAL_MSG_STATUS:
          handleStatusMsg(msg);
          break;
        case ESP_SERIAL_MSG_RATE:
          handleRateMsg(msg);
          break;
      }

    }
  }
}

// ----------------------------------------------------------------------------

void SerialDriver::handleIMUMsg(const esp_serial_message_t& msg)
{
  esp_serial_imu_msg_t imu;
  esp_serial_imu_msg_unpack(&imu, &msg);

  std::lock_guard<std::mutex> lock(mtx_);
  if (cb_imu_) cb_imu_(imu);
}

// ----------------------------------------------------------------------------

void SerialDriver::handleStatusMsg(const esp_serial_message_t& msg)
{
  esp_serial_status_msg_t status;
  esp_serial_status_msg_unpack(&status, &msg);

  std::lock_guard<std::mutex> lock(mtx_);
  if (cb_status_) cb_status_(status);
}

// ----------------------------------------------------------------------------

void SerialDriver::handleRateMsg(const esp_serial_message_t& msg)
{
  esp_serial_rate_msg_t rate;
  esp_serial_rate_msg_unpack(&rate, &msg);

  std::lock_guard<std::mutex> lock(mtx_);
  if (cb_rate_) cb_rate_(rate);
}

} // ns esp32imu
} // ns acl
