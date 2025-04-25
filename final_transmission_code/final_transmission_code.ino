#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <RF24.h>
#include <Adafruit_BMP085.h>
#include <MPU6050.h>

Adafruit_BMP085 bmp;
MPU6050 mpu;
RF24 radio(9, 10); // CE, CSN

#define SD_CS 4
File logFile;

const byte address[6] = "00001";
unsigned long calibrationCompleteTime = 0;
const unsigned long delayBeforeCalibration = 30000;  // 30 sec after power on
const unsigned long delayAfterCalibration = 20000;   // 20 sec after calibration
const unsigned long loggingDuration = 5UL * 60UL * 1000UL; // 5 minutes

struct SensorData 
{
  float temperature, pressure, altitude;
  int16_t ax, ay, az;
  int16_t gx, gy, gz;
  unsigned long time;
};

// ---------- Function Prototypes ----------
void initSDCard();
void calibrateAndCheckModules();
SensorData readSensors();
void sendData(SensorData &d);
void saveToSD(SensorData &d);

void setup() 
{
  Serial.begin(9600);
  Wire.begin();

  radio.begin();
  radio.openWritingPipe(address);
  radio.setPALevel(RF24_PA_LOW);
  radio.stopListening();

  initSDCard();

  Serial.println("Waiting 30 seconds before calibration...");
  delay(delayBeforeCalibration);

  calibrateAndCheckModules();

  Serial.println("Waiting 20 seconds before logging...");
  delay(delayAfterCalibration);
  calibrationCompleteTime = millis();
}



void loop() 
{
  if (millis() - calibrationCompleteTime > loggingDuration) 
  {
    // Stop logging and transmitting after 5 minutes
    return;
  }

  SensorData data = readSensors();
  data.time = millis() - calibrationCompleteTime;

  sendData(data);
  saveToSD(data);
  delay(200); // Adjust sample rate
}

// ------------------ SUPPORT FUNCTIONS ------------------

void initSDCard() 
{
  if (!SD.begin(SD_CS)) 
  {
    Serial.println("SD init failed");
    while (1);
  }

  logFile = SD.open("flight.csv", FILE_WRITE);
  if (logFile) 
  {
    logFile.println("Time(ms),Temp(C),Pressure(Pa),Altitude(m),AX,AY,AZ,GX,GY,GZ");
    logFile.close();
  }
}



void calibrateAndCheckModules() 
{
  bool ok = true;

  // BMP180
  if (!bmp.begin()) 
  {
    radio.write("BMP180 fail", 12);
    ok = false;
  } 
  else 
  {
    radio.write("BMP180 OK", 10);
  }

  // MPU6050
  mpu.initialize();
  if (!mpu.testConnection()) 
  {
    radio.write("MPU6050 fail", 14);
    ok = false;
  } 
  else 
  {
    radio.write("MPU6050 OK", 12);
  }

  if (!ok) 
  {
    radio.write("Module Fail", 12);
    while (1);
  }

  // Calibration
  int32_t ax_off = 0, ay_off = 0, az_off = 0;
  int32_t gx_off = 0, gy_off = 0, gz_off = 0;
  int16_t ax, ay, az, gx, gy, gz;
  const int samples = 100;

  for (int i = 0; i < samples; i++) 
  {
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    ax_off += ax; ay_off += ay; az_off += az;
    gx_off += gx; gy_off += gy; gz_off += gz;
    delay(10);
  }

  ax_off /= samples; ay_off /= samples; az_off = (az_off / samples) - 16384;
  gx_off /= samples; gy_off /= samples; gz_off /= samples;

  mpu.setXAccelOffset(ax_off);
  mpu.setYAccelOffset(ay_off);
  mpu.setZAccelOffset(az_off);
  mpu.setXGyroOffset(gx_off);
  mpu.setYGyroOffset(gy_off);
  mpu.setZGyroOffset(gz_off);

  // Save offsets to SD
  File offsetFile = SD.open("offsets.txt", FILE_WRITE);
  if (offsetFile) 
  {
    offsetFile.println("MPU6050 Calibration Offsets:");
    offsetFile.print("AX: "); offsetFile.println(ax_off);
    offsetFile.print("AY: "); offsetFile.println(ay_off);
    offsetFile.print("AZ: "); offsetFile.println(az_off);
    offsetFile.print("GX: "); offsetFile.println(gx_off);
    offsetFile.print("GY: "); offsetFile.println(gy_off);
    offsetFile.print("GZ: "); offsetFile.println(gz_off);
    offsetFile.close();
  }

  // Initial sensor reading
  SensorData initData = readSensors();
  radio.write("CAL DONE", 9);
  delay(50);
  radio.write(&initData, sizeof(SensorData));
}



SensorData readSensors() 
{
  SensorData d;
  d.temperature = bmp.readTemperature();
  d.pressure = bmp.readPressure();
  d.altitude = bmp.readAltitude();
  mpu.getMotion6(&d.ax, &d.ay, &d.az, &d.gx, &d.gy, &d.gz);
  return d;
}



void sendData(SensorData &d) 
{
  radio.write(&d, sizeof(SensorData));
}



void saveToSD(SensorData &d) 
{
  logFile = SD.open("flight.csv", FILE_WRITE);
  if (logFile) 
  {
    logFile.print(d.time); logFile.print(",");
    logFile.print(d.temperature); logFile.print(",");
    logFile.print(d.pressure); logFile.print(",");
    logFile.print(d.altitude); logFile.print(",");
    logFile.print(d.ax); logFile.print(",");
    logFile.print(d.ay); logFile.print(",");
    logFile.print(d.az); logFile.print(",");
    logFile.print(d.gx); logFile.print(",");
    logFile.print(d.gy); logFile.print(",");
    logFile.println(d.gz);
    logFile.close();
  }
}
