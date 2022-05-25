/**
 * LSM6D3 connected to SPI2 (HSPI) [default pins]
 */
 
#include <SPI.h>
#include <Adafruit_LSM6DS3.h>

#include "esp32imu_serial.h"

SPIClass SPI2(HSPI);
//SPIClass SPI3(VSPI);

static constexpr int LSM_CS = 15;
static constexpr int LSM_INT1 = 22;

Adafruit_LSM6DS3 imu;
volatile bool flag = false;

// serial stuff
uint8_t out_buf[ESP_SERIAL_MAX_MESSAGE_LEN];
esp_serial_message_t msg_buf;

// timing
uint32_t start_time_us = 0;

//=============================================================================
// ISR
//=============================================================================

void myInterrupt() {
  flag = true;
}

//=============================================================================
// initialize
//=============================================================================

void init_imu() {
  SPI2.begin();

  if (!imu.begin_SPI(LSM_CS, &SPI2)) {
    Serial.println("Failed to find LSM6DS33 chip");
    while (1) {
      delay(10);
    }
  }
  
  Serial.println("LSM6DS33 Found!");

  // imu.setAccelRange(LSM6DS_ACCEL_RANGE_2_G);
  Serial.print("Accelerometer range set to: ");
  switch (imu.getAccelRange()) {
  case LSM6DS_ACCEL_RANGE_2_G:
    Serial.println("+-2G");
    break;
  case LSM6DS_ACCEL_RANGE_4_G:
    Serial.println("+-4G");
    break;
  case LSM6DS_ACCEL_RANGE_8_G:
    Serial.println("+-8G");
    break;
  case LSM6DS_ACCEL_RANGE_16_G:
    Serial.println("+-16G");
    break;
  }

  // imu.setGyroRange(LSM6DS_GYRO_RANGE_250_DPS);
  Serial.print("Gyro range set to: ");
  switch (imu.getGyroRange()) {
  case LSM6DS_GYRO_RANGE_125_DPS:
    Serial.println("125 degrees/s");
    break;
  case LSM6DS_GYRO_RANGE_250_DPS:
    Serial.println("250 degrees/s");
    break;
  case LSM6DS_GYRO_RANGE_500_DPS:
    Serial.println("500 degrees/s");
    break;
  case LSM6DS_GYRO_RANGE_1000_DPS:
    Serial.println("1000 degrees/s");
    break;
  case LSM6DS_GYRO_RANGE_2000_DPS:
    Serial.println("2000 degrees/s");
    break;
  case ISM330DHCX_GYRO_RANGE_4000_DPS:
    break; // unsupported range for the DS33
  }

   imu.setAccelDataRate(LSM6DS_RATE_1_66K_HZ);
  Serial.print("Accelerometer data rate set to: ");
  switch (imu.getAccelDataRate()) {
  case LSM6DS_RATE_SHUTDOWN:
    Serial.println("0 Hz");
    break;
  case LSM6DS_RATE_12_5_HZ:
    Serial.println("12.5 Hz");
    break;
  case LSM6DS_RATE_26_HZ:
    Serial.println("26 Hz");
    break;
  case LSM6DS_RATE_52_HZ:
    Serial.println("52 Hz");
    break;
  case LSM6DS_RATE_104_HZ:
    Serial.println("104 Hz");
    break;
  case LSM6DS_RATE_208_HZ:
    Serial.println("208 Hz");
    break;
  case LSM6DS_RATE_416_HZ:
    Serial.println("416 Hz");
    break;
  case LSM6DS_RATE_833_HZ:
    Serial.println("833 Hz");
    break;
  case LSM6DS_RATE_1_66K_HZ:
    Serial.println("1.66 KHz");
    break;
  case LSM6DS_RATE_3_33K_HZ:
    Serial.println("3.33 KHz");
    break;
  case LSM6DS_RATE_6_66K_HZ:
    Serial.println("6.66 KHz");
    break;
  }

   imu.setGyroDataRate(LSM6DS_RATE_1_66K_HZ);
  Serial.print("Gyro data rate set to: ");
  switch (imu.getGyroDataRate()) {
  case LSM6DS_RATE_SHUTDOWN:
    Serial.println("0 Hz");
    break;
  case LSM6DS_RATE_12_5_HZ:
    Serial.println("12.5 Hz");
    break;
  case LSM6DS_RATE_26_HZ:
    Serial.println("26 Hz");
    break;
  case LSM6DS_RATE_52_HZ:
    Serial.println("52 Hz");
    break;
  case LSM6DS_RATE_104_HZ:
    Serial.println("104 Hz");
    break;
  case LSM6DS_RATE_208_HZ:
    Serial.println("208 Hz");
    break;
  case LSM6DS_RATE_416_HZ:
    Serial.println("416 Hz");
    break;
  case LSM6DS_RATE_833_HZ:
    Serial.println("833 Hz");
    break;
  case LSM6DS_RATE_1_66K_HZ:
    Serial.println("1.66 KHz");
    break;
  case LSM6DS_RATE_3_33K_HZ:
    Serial.println("3.33 KHz");
    break;
  case LSM6DS_RATE_6_66K_HZ:
    Serial.println("6.66 KHz");
    break;
  }

  imu.configInt1(false, false, true); // accelerometer DRDY on INT1
//  imu.configInt2(false, true, false); // gyro DRDY on INT2


  pinMode(LSM_INT1, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(LSM_INT1), myInterrupt, RISING);


  sensors_event_t accel;
  sensors_event_t gyro;
  sensors_event_t temp;
  imu.getEvent(&accel, &gyro, &temp);
}

// -------------------------------------------------------------------------

void setup() {
//  Serial.setTxBufferSize(512); // uncomment when CONFIG_DISABLE_HAL_LOCKS not defined
  Serial.begin(115200);
  
  init_imu();

//  xTaskCreatePinnedToCore(vTaskGetData, "vTaskGetData", 1024, NULL, 2, &xTaskToNotify, 1);
  xTaskCreatePinnedToCore(vLoop, "vLoop", 4096, NULL, 1, NULL, 1);
}

// -------------------------------------------------------------------------

void loop() {
  vTaskDelete(NULL);
}

// -------------------------------------------------------------------------

void vLoop(void * pvParameters) {
  (void) pvParameters;
  TickType_t xLastWakeTime = xTaskGetTickCount();
  uint32_t lastwritemicros = 0;

  for (;;) {

    uint32_t current_time_us = micros() - start_time_us;
 
    if (flag) {
  
      sensors_event_t accel;
      sensors_event_t gyro;
      sensors_event_t temp;
      imu.getEvent(&accel, &gyro, &temp);
  
      // pack and ship IMU data
      esp_serial_imu_msg_t imu_msg;
      imu_msg.t_us = current_time_us;
      imu_msg.accel_x = accel.acceleration.x;
      imu_msg.accel_y = accel.acceleration.y;
      imu_msg.accel_z = accel.acceleration.z;
      imu_msg.gyro_x = gyro.gyro.x;
      imu_msg.gyro_y = gyro.gyro.y;
      imu_msg.gyro_z = gyro.gyro.z;
      imu_msg.mag_x = 0;
      imu_msg.mag_y = 0;
      imu_msg.mag_z = 0;
     
      const size_t len = esp_serial_imu_msg_send_to_buffer(out_buf, &imu_msg);
      Serial.write(out_buf, len);
  
//      sensor_poll_previous_us = current_time_us;
      flag = false;
    }

//    xTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(2));
  }
}
