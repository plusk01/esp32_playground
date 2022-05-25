// Basic demo for accelerometer/gyro readings from Adafruit LSM6DSM

#include <Adafruit_LSM6DSM.h>
Adafruit_LSM6DSM lsm6dsm;

SPIClass SPI2(HSPI);
//SPIClass SPI3(VSPI);/

#define LSM_CS 15

const int INT1_PIN_DRDY = 22; // INT1 pin to D3 - will be attached to gyro
volatile bool flag = false;
volatile long int last_time = micros();

void IRAM_ATTR myInterrupt() {
    flag = true;
}

void setup(void) {
  delay(1000);
  
  Serial.begin(115200);
  while (!Serial)
    delay(10); // will pause Zero, Leonardo, etc until serial console opens

  Serial.println("Adafruit LSM6DSM test!");

  if (!lsm6dsm.begin_SPI(LSM_CS, &SPI2)) {
    Serial.println("Failed to find LSM6DSM chip");
    while (1) {
      delay(10);
    }
  }

  Serial.println("LSM6DSM Found!");

  lsm6dsm.setAccelRange(LSM6DS_ACCEL_RANGE_16_G);
  Serial.print("Accelerometer range set to: ");
  switch (lsm6dsm.getAccelRange()) {
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

  lsm6dsm.setGyroRange(LSM6DS_GYRO_RANGE_2000_DPS);
  Serial.print("Gyro range set to: ");
  switch (lsm6dsm.getGyroRange()) {
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

  lsm6dsm.setAccelDataRate(LSM6DS_RATE_3_33K_HZ);
  Serial.print("Accelerometer data rate set to: ");
  switch (lsm6dsm.getAccelDataRate()) {
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

  lsm6dsm.setGyroDataRate(LSM6DS_RATE_3_33K_HZ);
  Serial.print("Gyro data rate set to: ");
  switch (lsm6dsm.getGyroDataRate()) {
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

  lsm6dsm.configInt1(false, false, true); // accelerometer DRDY on INT1
//  lsm6dsm.configInt2(false, true, false); // gyro DRDY on INT2

  pinMode(INT1_PIN_DRDY, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(INT1_PIN_DRDY), myInterrupt, RISING);

  sensors_event_t accel;
  sensors_event_t gyro;
  sensors_event_t temp;
  lsm6dsm.getEvent(&accel, &gyro, &temp);
}

long int jstart = micros();

void loop() {
    sensors_event_t accel;
    sensors_event_t gyro;
    sensors_event_t temp;
  if (flag) {
    //  /* Get a new normalized sensor event */
    lsm6dsm.getEvent(&accel, &gyro, &temp);

    flag = false;
    
    Serial.print((micros() - jstart)/1e6,6);
    Serial.print(", ");
    Serial.print(accel.acceleration.x,4);
    Serial.print(", "); Serial.print(accel.acceleration.y,4);
    Serial.print(", "); Serial.print(accel.acceleration.z,4);
    Serial.print(", ");
    
    Serial.print(gyro.gyro.x,4);
    Serial.print(", "); Serial.print(gyro.gyro.y,4);
    Serial.print(", "); Serial.print(gyro.gyro.z,4);
    Serial.println();
  }

}
