#include <Wire.h>
#include <MPU6050.h>
#include <Adafruit_BMP085.h>

Adafruit_BMP085 bmp;
MPU6050 mpu;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  Wire.begin();

  if (!bmp.begin()) 
  {
    Serial.println("BMP180 dead or not working");
    while(1);
  }

  mpu.initialize();
  if (!mpu.testConnection()) 
  {
    Serial.println("MPU6050 connection failed!");
    while(1);
  }
  Serial.println("Sensors initalized");
}

void loop() {
  // put your main code here, to run repeatedly:
  float temp = bmp.readTemperature();
  float pressure = bmp.readPressure();

  int16_t ax, ay, az;
  int16_t gx, gy, gz;
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  Serial.print("Temp: ");
  Serial.print(temp);
  Serial.print(" C\t");
  Serial.print("Pressure: ");
  Serial.print((pressure / 100.0) * 0.0009869233);
  Serial.println(" atm");
  Serial.print("\n");

  Serial.print("Accel: ");
  Serial.print("X= ");
  Serial.print(ax);
  Serial.print(" ");
  Serial.print("Y= ");
  Serial.print(ay);
  Serial.print(" ");
  Serial.print("Z= ");
  Serial.print(az);
  Serial.print("\n");

  Serial.print("Gyro: ");
  Serial.print("X= ");
  Serial.print(gx);
  Serial.print(" ");
  Serial.print("Y= ");
  Serial.print(gy);
  Serial.print(" ");
  Serial.print("Z= ");
  Serial.print(gz);
  Serial.print("\n");
  
  delay(1900);
}
