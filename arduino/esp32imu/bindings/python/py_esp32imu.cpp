/**
 * @file py_teensyimu.cpp
 * @brief Python bindings for teensyimu
 * @author Parker Lusk <plusk@mit.edu>
 * @date 22 Nov 2020
 */

#include <cstdint>

#include <pybind11/pybind11.h>
#include <pybind11/functional.h>

#include <protocol/esp32imu_serial.h>
#include <esp32imu/serial_driver.h>

namespace py = pybind11;

PYBIND11_MODULE(esp32imu, m)
{
  m.doc() = "Serial driver and tools for ESP32 IMU board";
  m.attr("__version__") = PROJECT_VERSION;

  py::class_<esp_serial_imu_msg_t>(m, "SerialIMUMsg")
    .def_readwrite("t_us", &esp_serial_imu_msg_t::t_us)
    .def_readwrite("accel_x", &esp_serial_imu_msg_t::accel_x)
    .def_readwrite("accel_y", &esp_serial_imu_msg_t::accel_y)
    .def_readwrite("accel_z", &esp_serial_imu_msg_t::accel_z)
    .def_readwrite("gyro_x", &esp_serial_imu_msg_t::gyro_x)
    .def_readwrite("gyro_y", &esp_serial_imu_msg_t::gyro_y)
    .def_readwrite("gyro_z", &esp_serial_imu_msg_t::gyro_z)
    .def_readwrite("mag_x", &esp_serial_imu_msg_t::mag_x)
    .def_readwrite("mag_y", &esp_serial_imu_msg_t::mag_y)
    .def_readwrite("mag_z", &esp_serial_imu_msg_t::mag_z);

  // py::class_<ti_serial_rate_msg_t>(m, "SerialRateMsg")
  //   .def(py::init<uint16_t>(),
  //         py::arg("frequency")=500)
  //   .def_readwrite("frequency", &ti_serial_rate_msg_t::frequency);

  // py::class_<ti_serial_motorcmd_msg_t>(m, "SerialMotorCmdMsg")
  //   .def(py::init<uint16_t>(),
  //         py::arg("percentage")=0)
  //   .def_readwrite("percentage", &ti_serial_motorcmd_msg_t::percentage);

  py::class_<acl::esp32imu::SerialDriver>(m, "SerialDriver")
    .def(py::init<const std::string&, uint32_t>(),
          py::arg("port")="/dev/ttyUSB0", py::arg("baud")=2000000)
    // .def("sendRate", &acl::esp32imu::SerialDriver::sendRate)
    // .def("sendMotorCmd", &acl::esp32imu::SerialDriver::sendMotorCmd)
    .def("registerCallbackIMU", &acl::esp32imu::SerialDriver::registerCallbackIMU)
    // .def("registerCallbackRate", &acl::esp32imu::SerialDriver::registerCallbackRate)
    .def("unregisterCallbacks", &acl::esp32imu::SerialDriver::unregisterCallbacks, py::call_guard<py::gil_scoped_release>());
}