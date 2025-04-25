#include <SD.h>
#include <MPU6050.h>
#include <Adafruit_BMP085.h>

// BMP180 Sensor
Adafruit_BMP085 bmp;

// MPU6050 Sensor
MPU6050 mpu;

// SD Card
#define SD_CS 4

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  Wire.begin();

  // Initialize BMP180
  if (!bmp.begin()) 
  {
    Serial.println("BMP180 dead or not working");
    while (1);
  }
  
  // Initialize MPU6050
  // mpu.initialize();
  // if (!mpu.testConnection()) 
  // {
  //   Serial.println("MPU6050 connection failed!");
  //   while (1);
  // }
  Serial.println("Sensors initialized");
  
  // // Initialize SD card
  // if (!SD.begin(SD_CS)) {
  //   Serial.println("SD card initialization failed!");
  //   while (1);
  // }
  // Serial.println("SD card initialized.");

}

void loop() {
  // put your main code here, to run repeatedly:
  float temp = bmp.readTemperature();
  float pressure = bmp.readPressure() * 9.869233e-6 ; // Convert Pa to atm

  // Output to Serial
  Serial.print("Temperature: ");
  Serial.print(temp);
  Serial.print(" °C, Pressure: ");
  Serial.print(pressure);
  Serial.println(" atm");

  // // Log to SD card
  // File logFile = SD.open("data.txt", FILE_WRITE);
  // if (logFile) {
  //   logFile.print("Temperature: ");
  //   logFile.print(temp);
  //   logFile.print(" °C, Pressure: ");
  //   logFile.print(pressure);
  //   logFile.println(" hPa");
  //   logFile.close();
  // } else {
  //   Serial.println("Failed to write to SD card.");
  // }

  delay(500);
}
