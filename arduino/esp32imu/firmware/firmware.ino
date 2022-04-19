/**
 * ICM-20948 connected to SPI3 (VSPI) [default pins]
 * SD card connected to SPI2 (HSPI) [default pins]
 * 
 * Change sample rate via SAMPLE_PER_SEC
 * Change sample duration via NUM_SAMPLES
 * 
 * Change storage medium via USE_* #define, e.g., USE_SD
 * 
 * Program first collects samples, then reads file and prints via Serial.
 */
 
#include <SPI.h>
#include <ICM_20948.h>

#include "esp32imu_serial.h"

SPIClass SPI2(HSPI);
SPIClass SPI3(VSPI);

// IMU ring buffer
static constexpr int IMUBUF_SIZE = 200;
esp_serial_imu_msg_t imubuf_[IMUBUF_SIZE];
volatile uint16_t imubuf_head_ = 0;
volatile uint16_t imubuf_tail_ = 0;
volatile uint32_t seq = 0;

volatile uint32_t t0_ = 0; ///< starting time
static constexpr int PRINT_MAX = 2000;

ICM_20948_SPI imu;
float Gscale = (M_PI / 180.0) / 32.8;
float Ascale = 1.0 / 16384.0;
float Mscale = 1.0;

// sample rate, controlled by hardware timer
hw_timer_t * timer = nullptr;
static constexpr int TICKS_PER_SEC = 1000000;
static constexpr int SAMPLE_PER_SEC = 4000;
static constexpr int PERIOD_TICKS = TICKS_PER_SEC / SAMPLE_PER_SEC;

TaskHandle_t xTaskToNotify = NULL;

// serial stuff
uint8_t serbuf[IMUBUF_SIZE * ESP_SERIAL_MAX_MESSAGE_LEN];


//=============================================================================
// ISR
//=============================================================================

bool IRAM_ATTR timer_isr() {
  BaseType_t xHigherPriorityTaskHasWoken = pdFALSE;
  vTaskNotifyGiveFromISR(xTaskToNotify, &xHigherPriorityTaskHasWoken);

//    uint32_t tnow = micros() - t0_;
//    imubuf_[imubuf_head_].t_us = tnow;
//    imubuf_[imubuf_head_].seq = seq++;
//    imubuf_head_ = (imubuf_head_ + 1) % IMUBUF_SIZE;

  return xHigherPriorityTaskHasWoken;
}

//=============================================================================
// RTOS Tasks
//=============================================================================

void vTaskGetData(void * pvParameters) {
  (void) pvParameters;

  // begin the hardware timer / sampling
  timer = timerBegin(0, 80, true);
  timerAttachInterrupt(timer, timer_isr, true);
  timerAlarmWrite(timer, PERIOD_TICKS, true);
  timerAlarmEnable(timer);
  t0_ = micros();
  
  for (;;) {
    if (ulTaskNotifyTake(pdTRUE, portMAX_DELAY) == pdTRUE) {

        uint32_t tnow = micros() - t0_;

       // complete a SPI transaction with IMU to get data
       ICM_20948_AGMT_t agmt = imu.getAGMT(); // takes ~208 usec @ 4MHz, ~169usec @ 7MHz
//        delayMicroseconds(100);

       imubuf_[imubuf_head_].t_us = tnow;
       imubuf_[imubuf_head_].seq = seq++;
//       imubuf_[imubuf_head_].accel_x = micros() - t0_ - tnow; //Ascale * agmt.acc.axes.x;
//       imubuf_[imubuf_head_].accel_y = imubuf_head_; //Ascale * agmt.acc.axes.y;
       imubuf_[imubuf_head_].accel_x = Ascale * agmt.acc.axes.x;
       imubuf_[imubuf_head_].accel_y = Ascale * agmt.acc.axes.y;
       imubuf_[imubuf_head_].accel_z = Ascale * agmt.acc.axes.z;
       imubuf_[imubuf_head_].gyro_x = Gscale * agmt.gyr.axes.x;
       imubuf_[imubuf_head_].gyro_y = Gscale * agmt.gyr.axes.y;
       imubuf_[imubuf_head_].gyro_z = Gscale * agmt.gyr.axes.z;
       imubuf_[imubuf_head_].mag_x = Mscale * agmt.mag.axes.x;
       imubuf_[imubuf_head_].mag_y = Mscale * agmt.mag.axes.y;
       imubuf_[imubuf_head_].mag_z = Mscale * agmt.mag.axes.z;

        // move the head of the buffer, wrapping around if neccessary
        imubuf_head_ = (imubuf_head_ + 1) % IMUBUF_SIZE;      
    }
  }
}

//=============================================================================
// initialize
//=============================================================================

void init_imu() {
  SPI3.begin();

  bool initialized = false;
  while (!initialized) {
    imu.begin(SPI3.pinSS(), SPI3, 7000000);
    if (imu.status != ICM_20948_Stat_Ok) {
      delay(500);
    } else {
      initialized = true;
    }
  }

  // reset sensor so it is in known state
  imu.swReset();
  delay(500);

  // wake it up
  imu.sleep(false);
  imu.lowPower(false);

  // configure sample mode
  imu.setSampleMode((ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), ICM_20948_Sample_Mode_Continuous);

  // set full scale ranges
  ICM_20948_fss_t fss;
  fss.a = gpm2; // gpm2, gpm4, gpm8, gpm16
  fss.g = dps1000; // dps250, dps500, dps1000, dps2000
  imu.setFullScale((ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), fss);

  // set digital low-pass filter config
  ICM_20948_dlpcfg_t dlpcfg;
  dlpcfg.a = acc_d473bw_n499bw;
  dlpcfg.g = gyr_d361bw4_n376bw5;
  imu.setDLPFcfg((ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), dlpcfg);
  imu.enableDLPF((ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), false);

  // set sample rates to maximum
  ICM_20948_smplrt_t smplrt;
  smplrt.a = 1;
  smplrt.g = 1;
  imu.setSampleRate((ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), smplrt);

  // startup mag
  imu.startupMagnetometer();
  delay(1000);

  // get the ball rolling
  imu.getAGMT();
}

// -------------------------------------------------------------------------

void init_uart() {
  
}

// -------------------------------------------------------------------------

void setup() {
  Serial.setTxBufferSize(512); // uncomment when CONFIG_DISABLE_HAL_LOCKS not defined
  Serial.begin(2000000);
  
  init_imu();
  init_uart();

  xTaskCreatePinnedToCore(vTaskGetData, "vTaskGetData", 1024, NULL, 2, &xTaskToNotify, 1);
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

  static constexpr bool SEND_IMU = true;
  static constexpr bool SEND_STATUS = false;

  for (;;) {

    uint32_t tnow = micros();
 
    // timer may put additional data in buffer while this loop executes,
    // but we will only read up to what we can see right now
    const uint16_t head = imubuf_head_;
  
    // if there is new data to write
    if (head != imubuf_tail_) {
  
      size_t blen = 0;
  
      // if head ptr has looped back around, first get the data from here to the end
      if (head < imubuf_tail_) {
          const uint16_t len = min(PRINT_MAX, IMUBUF_SIZE - imubuf_tail_);
          for (size_t i=0; i<len; ++i) {
            if (SEND_IMU) {
              blen += esp_serial_imu_msg_send_to_buffer(&serbuf[blen], &imubuf_[imubuf_tail_+i]);
            }
          }
          imubuf_tail_ = (imubuf_tail_ + len) % IMUBUF_SIZE; // = 0;
      }
      
      // getting data is easy, from tail to head
      if (head > imubuf_tail_) {
        const uint16_t len = min(PRINT_MAX, head - imubuf_tail_);
        for (size_t i=0; i<len; ++i) {
          if (SEND_IMU) {
            blen += esp_serial_imu_msg_send_to_buffer(&serbuf[blen], &imubuf_[imubuf_tail_+i]);
          }
        }
        imubuf_tail_ = (imubuf_tail_ + len) % IMUBUF_SIZE; // = head;
      }

      Serial.write(serbuf, blen);
    }

    lastwritemicros = micros() - tnow;

    xTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(2));
  }
}
