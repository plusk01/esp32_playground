/**
 * LSM6DSM connected to SPI2 (HSPI) [default pins]
 * SD card connected to SPI3 (VSPI) [default pins]
 * 
 * Change sample rate via LSM6DS_RATE_3_33K_HZ
 * Change sample duration via SECONDS_TO_SAMPLE
 * 
 * Change storage medium via USE_* #define, e.g., USE_SD
 * 
 * Program first collects samples, then reads file and prints via Serial.
 */
 
#include <FS.h>
#include <SPI.h>
#include <FFat.h>
#include <LittleFS.h>
#include <SD.h>
#include <Adafruit_LSM6DSL.h> // note -- impl is same for LSM6DSM

union imudata_t {
 uint8_t bytes[44];
 struct {
   float t_sec;
   uint32_t idx;
   float a[3];
   float g[3];
   float m[3];
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

Adafruit_LSM6DSL imu;
static constexpr int LSM_INT1 = 22;

// how long to sample for?
static constexpr int SECONDS_TO_SAMPLE = 2;

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

void IRAM_ATTR dataready_isr() {
  vTaskNotifyGiveFromISR(xTaskToNotify, NULL);
  // TODO: if higher priority woken, do context switch?
}

// -------------------------------------------------------------------------

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

  if (!imu.begin_SPI(SPI2.pinSS(), &SPI2, 0, 10000000)) {
    Serial.println("Failed to find LSM6DS3 chip");
    while (1) {
      delay(10);
    }
  }

  Serial.println("LSM6DS33 Found!");

  imu.setAccelRange(LSM6DS_ACCEL_RANGE_16_G);
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

  imu.setGyroRange(LSM6DS_GYRO_RANGE_2000_DPS);
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

  // set start time
  t0_ = micros();

  imu.configInt1(false, true, false, false, false, true); // accelerometer DRDY on INT1
//  imu.configInt2(false, true, false); // gyro DRDY on INT2

  pinMode(LSM_INT1, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(LSM_INT1), dataready_isr, RISING);

//  sensors_event_t accel;
//  sensors_event_t gyro;
//  sensors_event_t temp;
//  imu.getEvent(&accel, &gyro, &temp);

  Serial.println("IMU config'd");
}

// -------------------------------------------------------------------------

void vTaskGetData(void * pvParameters) {
  (void) pvParameters;
  
  for (;;) {
    if (ulTaskNotifyTake(pdTRUE, portMAX_DELAY) == pdTRUE) {
       // complete a SPI transaction with IMU to get data
       sensors_event_t accel;
       sensors_event_t gyro;
       sensors_event_t temp;
//       imu.getEvent(&accel, &gyro, &temp);/

       imubuf_[imubuf_head_].data.t_sec = (micros() - t0_) * 1e-6;
       imubuf_[imubuf_head_].data.idx = seq++;
       imubuf_[imubuf_head_].data.a[0] = accel.acceleration.x;
       imubuf_[imubuf_head_].data.a[1] = accel.acceleration.y;
       imubuf_[imubuf_head_].data.a[2] = accel.acceleration.z;
       imubuf_[imubuf_head_].data.g[0] = gyro.gyro.x;
       imubuf_[imubuf_head_].data.g[1] = gyro.gyro.y;
       imubuf_[imubuf_head_].data.g[2] = gyro.gyro.z;
       imubuf_[imubuf_head_].data.m[0] = 0;
       imubuf_[imubuf_head_].data.m[1] = 0;
       imubuf_[imubuf_head_].data.m[2] = 0;

        // move the head of the buffer, wrapping around if neccessary
        imubuf_head_ = (imubuf_head_ + 1) % IMUBUF_SIZE;      
    }
  }
}

// -------------------------------------------------------------------------

void setup() {
  Serial.begin(2000000);

  if (!init_fs()) {
    Serial.println("filesystem mount failed");
    return;
  }

  file_ = fs_.open(DATABIN, FILE_WRITE);
  Serial.print(DATABIN); Serial.print(": "); Serial.print(file_.size()); Serial.println(" bytes");

  xTaskCreatePinnedToCore(vTaskGetData, "vTaskGetData", 2048, NULL, 2, &xTaskToNotify, 1);
  
  init_imu();

  delay(1000);

  xTaskCreatePinnedToCore(vLoop, "vLoop", 4096, NULL, 1, NULL, 1);

  sensors_event_t accel;
  sensors_event_t gyro;
  sensors_event_t temp;
  imu.getEvent(&accel, &gyro, &temp);
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
      Serial.print("Starting logger - sampling for ");
      Serial.print(SECONDS_TO_SAMPLE);
      Serial.println(" seconds");
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

      const float tnow = millis() * 1e-3;
  
      if (tnow - t0 >= SECONDS_TO_SAMPLE) {
        tf = (millis() * 1e-3) - t0;
        Serial.print(seqnow); Serial.print(" samples in "); Serial.print(tf); Serial.println(" seconds");
        Serial.print("t0 = "); Serial.print(t0); Serial.print("\ttnow = "); Serial.println(tnow);
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
          snprintf(txt, txtlen, "%d, %3.6f, %3.6f, %7.4f, %7.4f, %9.5f, %9.5f, %9.5f, %4.0f, %4.0f, %4.0f\n",
              imu.data.idx, imu.data.t_sec,
              imu.data.a[0], imu.data.a[1], imu.data.a[2],
              imu.data.g[0], imu.data.g[1], imu.data.g[2],
              imu.data.m[0], imu.data.m[1], imu.data.m[2]);
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
