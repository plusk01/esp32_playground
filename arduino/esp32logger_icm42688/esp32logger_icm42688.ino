/**
 * ICM-42688 connected to SPI2 (HSPI) [default pins]
 * SD card connected to SPI3 (VSPI) [default pins]
 * 
 * Change sample rate via SAMPLE_PER_SEC
 * Change sample duration via NUM_SAMPLES
 * 
 * Change storage medium via USE_* #define, e.g., USE_SD
 * 
 * Program first collects samples, then reads file and prints via Serial.
 */

#include <memory>
 
#include <FS.h>
#include <SPI.h>
#include <FFat.h>
#include <LittleFS.h>
#include <SD.h>
#include <ICM42688.h>

union imudata_t {
 uint8_t bytes[32];
 struct {
   float t_sec;
   uint32_t idx;
   float a[3];
   float g[3];
 } data;
};

SPIClass SPI2(HSPI);
SPIClass SPI3(VSPI);

// IMU ring buffer
static constexpr int IMUBUF_SIZE = 1000;
imudata_t imubuf_[IMUBUF_SIZE];
uint16_t imubuf_head_ = 0;
uint16_t imubuf_tail_ = 0;
uint32_t seq = 0;

volatile uint32_t t0_ = 0; ///< starting time

// logger state machine
enum State { IDLE, LOG, READ, DONE };
State state;

std::unique_ptr<ICM42688> imu;

// sample rate, controlled by hardware timer
hw_timer_t * timer = nullptr;
static constexpr int TICKS_PER_SEC = 1000000;
static constexpr int SAMPLE_PER_SEC = 4000;
static constexpr int PERIOD_TICKS = TICKS_PER_SEC / SAMPLE_PER_SEC;

// how many samples to collect?
static constexpr int NUM_SAMPLES = 2 * SAMPLE_PER_SEC;

TaskHandle_t xTaskToNotify = NULL;

// #define USE_LittleFS
// #define USE_FFat
#define USE_SD

#if defined(USE_LittleFS)
FS& fs_ = LittleFS;
#elif defined(USE_FFat)
FS& fs_ = FFat;
#elif defined(USE_SD)
FS& fs_ = SD;
#endif

char DATABIN[] = "/data.bin";
File file_;

///////////////////////////////////////////////////////////////////////

bool init_fs() {
  static constexpr bool format_if_failed = true;
  
#if defined(USE_LittleFS)
  if (!LittleFS.begin(format_if_failed)) {
#elif defined(USE_FFat)
 FFat.format();
 if (!FFat.begin()) {
#elif defined(USE_SD)
 SPI3.begin();
 if (!SD.begin(SPI3.pinSS(), SPI3, 25000000)) {
#endif
    return false;
  }

  return true;
}

// --------------------------------------------------------------------

void init_imu() {
  SPI2.begin();

  bool initialized = false;
  while (!initialized) {
    imu.reset(new ICM42688(SPI2, SPI2.pinSS(), 12000000));
    int status = imu->begin();
    if (status < 0) {
      Serial.println("IMU initialization unsuccessful");
      Serial.println("Check IMU wiring or try cycling power");
      Serial.print("Status: ");
      Serial.println(status);
      delay(500);
    } else {
      initialized = true;
    }
  }

  imu->setAccelODR(ICM42688::odr8k);
  imu->setGyroODR(ICM42688::odr8k);

  Serial.println("IMU config'd");
}

// -------------------------------------------------------------------------

void IRAM_ATTR timer_isr() {
  vTaskNotifyGiveFromISR(xTaskToNotify, NULL);
  // TODO: if higher priority woken, do context switch?
}

// -------------------------------------------------------------------------

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

      // const float t0 = micros(); // use this to measure SPI time
      // (i.e., change t0_ to t0)

      // complete a SPI transaction with IMU to get data
      // takes ~55usec @ 4MHz, ~41usec @ 7MHz, ~32usec @ 12MHz,
      //       ~29usec @ 16MHz, ~26usec @ 24MHz
      imu->getAGT();

      imubuf_[imubuf_head_].data.t_sec = (micros() - t0_) * 1e-6;
      imubuf_[imubuf_head_].data.idx = seq++;
      imubuf_[imubuf_head_].data.a[0] = imu->accX();
      imubuf_[imubuf_head_].data.a[1] = imu->accY();
      imubuf_[imubuf_head_].data.a[2] = imu->accZ();
      imubuf_[imubuf_head_].data.g[0] = imu->gyrX();
      imubuf_[imubuf_head_].data.g[1] = imu->gyrY();
      imubuf_[imubuf_head_].data.g[2] = imu->gyrZ();

      // move the head of the buffer, wrapping around if neccessary
      imubuf_head_ = (imubuf_head_ + 1) % IMUBUF_SIZE;      
    }
  }
}

// -------------------------------------------------------------------------

void setup() {
  Serial.begin(2000000);

  delay(2000);

  if (!init_fs()) {
    Serial.println("filesystem mount failed");
    return;
  }
  
  init_imu();

  delay(1000);

  file_ = fs_.open(DATABIN, FILE_WRITE);
  Serial.print(DATABIN); Serial.print(": "); Serial.print(file_.size()); Serial.println(" bytes");

  xTaskCreatePinnedToCore(vTaskGetData, "vTaskGetData", 1024, NULL, 2, &xTaskToNotify, 1);
  xTaskCreatePinnedToCore(vLoop, "vLoop", 4096, NULL, 1, NULL, 1);
}

// -------------------------------------------------------------------------

void loop() {
  vTaskDelete(NULL);
}

// -------------------------------------------------------------------------

void vLoop(void * pvParameters) {
  static float t0 = 0;
  static float tf = 0;
  static uint32_t seqnow = 0;
  TickType_t xLastWakeTime = xTaskGetTickCount();
  for (;;) {

    if (state == State::IDLE) {
      t0 = millis() * 1e-3;
      state = State::LOG;
    } else if (state == State::LOG) {
    
      // timer may put additional data in buffer while this loop executes,
      // but we will only read up to what we can see right now
      const uint16_t head = imubuf_head_;
      seqnow = seq;
  
      // if there is new data to write
      if (head != imubuf_tail_) {
  
  
        // if head ptr has looped back around, first get the data from here to the end
        if (head < imubuf_tail_) {
            const uint16_t len = IMUBUF_SIZE - imubuf_tail_;
            file_.write(reinterpret_cast<uint8_t*>(&imubuf_[imubuf_tail_]), len * sizeof(imudata_t));
            imubuf_tail_ = 0;
        }
  
        
        // getting data is easy, from tail to head
        if (head > imubuf_tail_) {
          const uint16_t len = head - imubuf_tail_;
          file_.write(reinterpret_cast<uint8_t*>(&imubuf_[imubuf_tail_]), len * sizeof(imudata_t));
          imubuf_tail_ = head;
        }
        
      }
  
      if (seqnow >= NUM_SAMPLES) {
        tf = (millis() * 1e-3) - t0;
        Serial.print(seqnow); Serial.print(" samples in "); Serial.print(tf); Serial.println(" seconds");
        state = State::READ;
      }
    } else if (state == State::READ) {
  
      file_.close();
  
      Serial.println("");
      Serial.println("Reading...");
      Serial.println("");
  
      file_ = fs_.open(DATABIN);
      Serial.print(DATABIN); Serial.print(": "); Serial.print(file_.size()); Serial.println(" bytes");
  
      if (file_) {
        imudata_t imu;
        while (file_.position() < file_.size()) {
          file_.read(imu.bytes, sizeof(imudata_t));
  
          static constexpr int txtlen = 200;
          char txt[txtlen];
          snprintf(txt, txtlen, "%d, %3.6f, %3.6f, %7.4f, %7.4f, %9.5f, %9.5f, %9.5f\n",
              imu.data.idx, imu.data.t_sec,
              imu.data.a[0], imu.data.a[1], imu.data.a[2],
              imu.data.g[0], imu.data.g[1], imu.data.g[2]);
          Serial.print(txt);
        }
        
      } else {
        Serial.println("Failed to open data file for reading");
      }
  
      state = State::DONE;
    } else if (state == State::DONE) {
  
    }
    
    xTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(1));
  }
}
