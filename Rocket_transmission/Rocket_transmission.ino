#include <SPI.h>

#include <SD.h>

#include <Wire.h>

#include <Adafruit_BMP085.h>

#include <MPU6050.h>

#include <RF24.h>

// BMP180 Sensor
Adafruit_BMP085 bmp;

// MPU6050 Sensor
MPU6050 mpu;

// nRF24L01 setup
RF24 radio(9, 10); // CE, CSN
const byte address[6] = "00001";

// Sensor data structure for transmission
struct SensorData 
{
  float temperature;
  float pressure;
  float altitude;
  int16_t ax, ay, az;
  int16_t gx, gy, gz;
};

// Sensor data structure for calibration
struct CalibrationData {
  float launchPressure;
  int16_t avg_ax, avg_ay, avg_az;
  int16_t avg_gx, avg_gy, avg_gz;
};

// Global variables
float launchHeight;

// Function declarations
float getTemp();
float getPressure();
float getAltitude(float baseHeight);
void calibrate();

void setup() {
  Serial.begin(9600);
  Wire.begin();

  // Initialize BMP180
  if (!bmp.begin()) 
  {
    Serial.println("BMP180 dead or not working");
    while (1);
  }

  // Initialize MPU6050
  mpu.initialize();
  if (!mpu.testConnection()) 
  {
    Serial.println("MPU6050 connection failed!");
    while (1);
  }
  Serial.println("Sensors initialized");

  // Initialize nRF24L01
  radio.begin();
  radio.openWritingPipe(address);
  radio.setPALevel(RF24_PA_LOW);
  radio.stopListening();

  // Calibrate sensors
  calibrate();
}

void loop() 
{
  SensorData data;

  // BMP180 Data
  data.temperature = getTemp();
  data.pressure = getPressure();
  data.altitude = getAltitude(launchHeight);

  // MPU6050 Data
  mpu.getMotion6(&data.ax, &data.ay, &data.az, &data.gx, &data.gy, &data.gz);

  // Send via nRF24L01
  radio.write(&data, sizeof(SensorData));

  delay(100); // Adjust for your data rate
}

void calibrate() 
{
  delay(2500);

  CalibrationData calib;
  // Calibrate BMP180 launch pressure
  calib.launchPressure = getPressure();
  launchHeight = calib.launchPressure;

  // MPU6050 calibration (simple average offset method)
  int32_t ax = 0, ay = 0, az = 0, gx = 0, gy = 0, gz = 0;
  const int samples = 100;

  for (int i = 0; i < samples; i++) 
  {
    int16_t axt, ayt, azt, gxt, gyt, gzt;
    mpu.getMotion6(&axt, &ayt, &azt, &gxt, &gyt, &gzt);
    ax += axt; 
    ay += ayt; 
    az += azt;
    gx += gxt; 
    gy += gyt; 
    gz += gzt;
    delay(10);
  }

  calib.avg_ax = ax / samples;
  calib.avg_ay = ay / samples;
  calib.avg_az = az / samples;
  calib.avg_gx = gx / samples;
  calib.avg_gy = gy / samples;
  calib.avg_gz = gz / samples;

  Serial.println("Calibration complete:");
  // Pressure
  Serial.print("Initial Pressure: "); 
  Serial.print(calib.launchPressure); 
  Serial.println(" atm");
  // Acceleration data
  Serial.print("MPU6050 avg acc: "); 
  Serial.print(calib.avg_ax); 
  Serial.print(", "); 
  Serial.print(calib.avg_ay); 
  Serial.print(", "); 
  Serial.println(calib.avg_az);
  // Gyro data
  Serial.print("MPU6050 avg gyro: "); 
  Serial.print(calib.avg_gx); 
  Serial.print(", "); 
  Serial.print(calib.avg_gy); 
  Serial.print(", "); 
  Serial.println(calib.avg_gz);

  // Transmit calibration data once
  radio.write(&calib, sizeof(CalibrationData));
}

float getTemp()
{
  return bmp.readTemperature();
}

float getPressure()
{
  return bmp.readPressure() * 9.869233e-6; // Convert Pa to atm
}

float getAltitude(float baseHeight)
{
  return bmp.readAltitude();
}
