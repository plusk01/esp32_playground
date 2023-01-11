#include <memory>
#include <ICM42688.h>

SPIClass SPI2(HSPI); // IMU on SPI2 (HSPI)

// an ICM42688 object with the ICM42688 sensor on SPI bus 0 and chip select pin 10
std::unique_ptr<ICM42688> imu;

// to use interrupts, uncomment this line
// #define USE_INTERRUPTS

volatile bool dataReady = false;

void setup() {
  // serial to display data
  Serial.begin(115200);
  while(!Serial) {}

  SPI2.begin();

  imu.reset(new ICM42688(SPI2, SPI2.pinSS()));

  // start communication with IMU
  int status = imu->begin();
  if (status < 0) {
    Serial.println("IMU initialization unsuccessful");
    Serial.println("Check IMU wiring or try cycling power");
    Serial.print("Status: ");
    Serial.println(status);
    while(1) {}
  }

  // imu->setAccelFS(ICM42688::gpm8);
  // imu->setGyroFS(ICM42688::dps500);

#ifdef USE_INTERRUPTS
  pinMode(22, INPUT);
  attachInterrupt(22, setImuFlag, RISING);

  imu->setAccelODR(ICM42688::odr12_5);
  imu->setGyroODR(ICM42688::odr12_5);

  imu->enableDataReadyInterrupt();
#endif

  Serial.println("ax,ay,az,gx,gy,gz,temp_C");
}

#ifdef USE_INTERRUPTS
void loop() {
  if (!dataReady) return;

  dataReady = false;

  // read the sensor
  imu->getAGT();
  
  // display the data
  Serial.print(imu->accX(),6);
  Serial.print("\t");
  Serial.print(imu->accY(),6);
  Serial.print("\t");
  Serial.print(imu->accZ(),6);
  Serial.print("\t");
  Serial.print(imu->gyrX(),6);
  Serial.print("\t");
  Serial.print(imu->gyrY(),6);
  Serial.print("\t");
  Serial.print(imu->gyrZ(),6);
  Serial.print("\t");
  Serial.println(imu->temp(),6);
}

void ARDUINO_ISR_ATTR setImuFlag() {
  dataReady = true;
}
#else
void loop() {
  // read the sensor
  imu->getAGT();
  // display the data
  Serial.print(imu->accX(),6);
  Serial.print("\t");
  Serial.print(imu->accY(),6);
  Serial.print("\t");
  Serial.print(imu->accZ(),6);
  Serial.print("\t");
  Serial.print(imu->gyrX(),6);
  Serial.print("\t");
  Serial.print(imu->gyrY(),6);
  Serial.print("\t");
  Serial.print(imu->gyrZ(),6);
  Serial.print("\t");
  Serial.println(imu->temp(),6);
  delay(100);
}
#endif