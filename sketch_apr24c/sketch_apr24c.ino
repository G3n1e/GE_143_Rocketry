#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <RF24.h>
#include <Adafruit_BMP085.h>
#include <MPU6050.h>

// Initialize modules
Adafruit_BMP085 bmp;
MPU6050 mpu;
RF24 radio(9, 10); // CE, CSN
#define SD_CS 4
const byte address[6] = "00001";


void setup() 
{
  Serial.begin(9600);
  delay(1000);
  Serial.println("===== BEGIN SYSTEM CHECK =====");

  testAltimeter();
  testIMU();
  testTransmitter();
  testStorage();

  Serial.println("===== SYSTEM CHECK COMPLETE =====");
}

void loop() {
  // Nothing to do in loop
}

// ----------- MODULE TEST FUNCTIONS -----------

void testAltimeter() 
{
  Serial.print("BMP180: ");
  if (bmp.begin()) {
    Serial.println("OK");
    Serial.print("  Temp: "); Serial.print(bmp.readTemperature()); Serial.println(" C");
    Serial.print("  Pressure: "); Serial.print(bmp.readPressure()); Serial.println(" Pa");
  } else {
    Serial.println("FAIL");
  }
}

void testIMU() 
{
  Serial.print("MPU6050: ");
  mpu.initialize();
  if (mpu.testConnection()) {
    Serial.println("OK");

    int16_t ax, ay, az, gx, gy, gz;
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    Serial.println("  Accel (x, y, z):");
    Serial.print("   "); Serial.print(ax); Serial.print(", ");
    Serial.print(ay); Serial.print(", ");
    Serial.println(az);

    Serial.println("  Gyro (x, y, z):");
    Serial.print("   "); Serial.print(gx); Serial.print(", ");
    Serial.print(gy); Serial.print(", ");
    Serial.println(gz);
  } else {
    Serial.println("FAIL");
  }
}

void testTransmitter() 
{
  Serial.print("RF24: ");
  if (radio.begin()) 
  {
    radio.openWritingPipe(address);
    radio.setPALevel(RF24_PA_LOW);
    radio.stopListening();

    const char testMsg[] = "RF24 Test OK";
    bool sent = radio.write(&testMsg, sizeof(testMsg));
    if (sent) {
      Serial.println("OK (message sent)");
    } 
    else 
    {
      Serial.println("FAIL (write failed)");
    }
  } else 
  {
    Serial.println("FAIL (init failed)");
  }
}

void testStorage() 
{
  Serial.print("SD Card: ");
  if (SD.begin(SD_CS)) {
    Serial.println("OK");
    File testFile = SD.open("test.txt", FILE_WRITE);
    if (testFile) {
      testFile.println("SD test successful.");
      testFile.close();
      Serial.println("  File write OK");
    } else {
      Serial.println("  File write FAIL");
    }
  } else {
    Serial.println("FAIL");
  }
}
