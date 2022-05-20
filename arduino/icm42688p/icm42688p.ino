#include <SPI.h>

SPIClass SPI2(HSPI); // IMU on SPI2 (HSPI)

static constexpr int freq = 24 * 1000 * 1000;
static constexpr int ICM42688P_WHOAMI = 0x47;

static constexpr uint8_t BANK0_DEVICE_CONFIG = 0x11;
static constexpr uint8_t BANK0_WHO_AM_I = 0x75;
static constexpr uint8_t BANK0_REG_BANK_SEL = 0x76;

static constexpr uint8_t DEVICE_CONFIG_SOFT_RESET_BIT = 0x01;

uint8_t whoami = 0x00;
SPISettings settings;

void setup() {
  Serial.begin(115200);
  SPI2.begin();

  pinMode(0, OUTPUT);
  pinMode(2, OUTPUT);
  pinMode(4, OUTPUT);
  pinMode(SPI2.pinSS(), OUTPUT);

  digitalWrite(0, LOW); // R
  digitalWrite(2, LOW); // G
  digitalWrite(4, LOW); // B

  settings = SPISettings(freq, MSBFIRST, SPI_MODE0);

  //
  // Soft reset
  //

  // kickstart SPI hardware...
  SPI2.beginTransaction(settings);
  SPI2.transfer(0x00);
  SPI2.endTransaction();

  SPI2.beginTransaction(settings);
  digitalWrite(SPI2.pinSS(), LOW);
  SPI2.transfer(BANK0_DEVICE_CONFIG | 0x00);
  SPI2.transfer(DEVICE_CONFIG_SOFT_RESET_BIT);
  digitalWrite(SPI2.pinSS(), HIGH);
  SPI2.endTransaction();

  delay(2000); // 1ms delay for soft reset to be effective

  //
  // Set BANK 0
  //

  // kickstart SPI hardware...
  SPI2.beginTransaction(settings);
  SPI2.transfer(0x00);
  SPI2.endTransaction();

  SPI2.beginTransaction(settings);
  digitalWrite(SPI2.pinSS(), LOW);
  SPI2.transfer(BANK0_REG_BANK_SEL | 0x00);
  SPI2.transfer(0x00); // BANK 0
  digitalWrite(SPI2.pinSS(), HIGH);
  SPI2.endTransaction();

  //
  // Get WHO_AM_I
  //

  // kickstart SPI hardware...
  SPI2.beginTransaction(settings);
  SPI2.transfer(0x00);
  SPI2.endTransaction();

  SPI2.beginTransaction(settings);
  digitalWrite(SPI2.pinSS(), LOW);
  SPI2.transfer(BANK0_WHO_AM_I | 0x80);
  whoami = SPI2.transfer(0x00); // get data
  digitalWrite(SPI2.pinSS(), HIGH);
  SPI2.endTransaction();
}

void loop() {
  //
  // Get WHO_AM_I
  //

  // kickstart SPI hardware...
  SPI2.beginTransaction(settings);
  SPI2.transfer(0x00);
  SPI2.endTransaction();

  SPI2.beginTransaction(settings);
  digitalWrite(SPI2.pinSS(), LOW);
  SPI2.transfer(BANK0_WHO_AM_I | 0x80);
  whoami = SPI2.transfer(0x00); // get data
  digitalWrite(SPI2.pinSS(), HIGH);
  SPI2.endTransaction();
  
  Serial.print("WHOAMI: "); Serial.println(whoami);

  delay(1000);
}
