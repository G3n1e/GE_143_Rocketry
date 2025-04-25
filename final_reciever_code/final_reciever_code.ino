#include <Wire.h>
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <LiquidCrystal_I2C.h>

// Initialize LCD (address 0x27 is common; if your screen doesn't work, try 0x3F)
LiquidCrystal_I2C lcd(0x27, 16, 2);

// RF24
RF24 radio(9, 10);  // CE, CSN
const byte address[6] = "00001";

struct SensorData {
  float temperature, pressure, altitude;
  int16_t ax, ay, az;
  int16_t gx, gy, gz;
  unsigned long time;
};

void setup() {
  Serial.begin(9600);

  // LCD Init
  lcd.begin();
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print("Waiting for data");

  // RF24 Init
  radio.begin();
  radio.openReadingPipe(0, address);
  radio.setPALevel(RF24_PA_LOW);
  radio.startListening();
}

void loop() {
  if (radio.available()) {
    SensorData data;
    radio.read(&data, sizeof(SensorData));

    // Print to Serial Monitor
    Serial.print("Altitude: ");
    Serial.print(data.altitude);
    Serial.println(" m");

    // Display on LCD
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Altitude:");
    lcd.setCursor(0, 1);
    lcd.print(data.altitude, 2); // 2 decimal places
    lcd.print(" m");
  }
}
