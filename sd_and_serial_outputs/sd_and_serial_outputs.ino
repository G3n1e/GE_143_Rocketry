#include <LiquidCrystal_I2C.h>
#include <LiquidCrystal.h>
#include <Adafruit_BMP085.h>
#include <SPI.h>
#include <SD.h>

#include <Adafruit_BMP085.h>
#include <SD.h>
#include <SPI.h>

// BMP180 Sensor
Adafruit_BMP085 bmp;

// SD Card
#define SD_CS 4

void setup() {
  Serial.begin(9600);

  // Initialize BMP180
  if (!bmp.begin()) {
    Serial.println("Could not find BMP180 sensor.");
    while (1);
  }
  Serial.println("BMP180 initialized.");

  // Initialize SD card
  if (!SD.begin(SD_CS)) {
    Serial.println("SD card initialization failed!");
    while (1);
  }
  Serial.println("SD card initialized.");
}

void loop() {
  float temp = bmp.readTemperature();
  float pressure = bmp.readPressure() / 100.0; // Convert Pa to hPa

  // Output to Serial
  Serial.print("Temperature: ");
  Serial.print(temp);
  Serial.print(" °C, Pressure: ");
  Serial.print(pressure);
  Serial.println(" hPa");

  // Log to SD card
  File logFile = SD.open("data.txt", FILE_WRITE);
  if (logFile) {
    logFile.print("Temperature: ");
    logFile.print(temp);
    logFile.print(" °C, Pressure: ");
    logFile.print(pressure);
    logFile.println(" hPa");
    logFile.close();
  } else {
    Serial.println("Failed to write to SD card.");
  }

  delay(2000); // Delay between readings
}
