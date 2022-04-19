/**
 * @file serial_driver.h
 * @brief Serial communication driver for esp32-imu
 * @author Parker Lusk <plusk@mit.edu>
 * @date 21 Nov 2020
 */

#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <string>

#include "protocol/esp32imu_serial.h"

namespace async_comm { class Serial; }

namespace acl {
namespace esp32imu {

  using CallbackIMU = std::function<void(const esp_serial_imu_msg_t&)>;
  using CallbackStatus = std::function<void(const esp_serial_status_msg_t&)>;
  using CallbackRate = std::function<void(const esp_serial_rate_msg_t&)>;

  class SerialDriver
  {
  public:
    SerialDriver(const std::string& port = "/dev/ttyACM0", uint32_t baud = 115200);
    ~SerialDriver();

    void sendRate(uint16_t frequency);

    void registerCallbackIMU(CallbackIMU cb);
    void registerCallbackStatus(CallbackStatus cb);
    void registerCallbackRate(CallbackRate cb);
    void unregisterCallbacks();
    
  private:
    std::unique_ptr<async_comm::Serial> serial_;
    CallbackIMU cb_imu_;
    CallbackStatus cb_status_;
    CallbackRate cb_rate_;
    std::mutex mtx_; ///< synchronize callback resource reg/unreg

    void callback(const uint8_t * data, size_t len);

    void handleIMUMsg(const esp_serial_message_t& msg);
    void handleStatusMsg(const esp_serial_message_t& msg);
    void handleRateMsg(const esp_serial_message_t& msg);
  };

} // ns esp32imu
} // ns acl
