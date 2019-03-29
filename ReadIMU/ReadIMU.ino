/*
ReadIMU.inu
Namita Anil Kumar
Basin_I2C.ino is used to read and proces data from MPU 9150 and 9250. This script filter the
processed data using complementary filters and prints them on to the serial monitor. 
Copyright information regarding Basic_I2C.ino can be found below:

Basic_I2C.ino
Brian R Taylor
brian.taylor@bolderflight.com

Copyright (c) 2017 Bolder Flight Systems

Permission is hereby granted, free of charge, to any person obtaining a copy of this software 
and associated documentation files (the "Software"), to deal in the Software without restriction, 
including without limitation the rights to use, copy, modify, merge, publish, distribute, 
sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is 
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or 
substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING 
BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND 
NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, 
DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, 
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#include "MPU9250.h"
#include "Math.h"

// an MPU9250 object with the MPU-9250 sensor on I2C bus 0 with address 0x68
MPU9250 IMU(Wire2,0x68);
int status;

float ax, ay, az, gx, gy, gz, mx, my, mz; // variables to hold latest sensor data values 
float timer = 0.0;
float delta_t = 0.0;
float pitch = 0.0;
float roll = 0.0;
float yaw = 0.0;
float pitchAcc, rollAcc, yawAcc;

void setup() {
  // serial to display data
  Serial.begin(38400);

  // start communication with IMU 
  status = IMU.begin();
  if (status < 0) {
    Serial.println("IMU initialization unsuccessful");
    Serial.println("Check IMU wiring or try cycling power");
    Serial.print("Status: ");
    Serial.println(status);
    while(1) {}
  }

}

void loop() {
  timer = micros();
   
  // read the sensor
  IMU.readSensor();
    
  //store data in variables. acceleration is in m/s2, gyroscope in rad/s
  //magnetometer in uT
  ax = IMU.getAccelX_mss();
  ay = IMU.getAccelY_mss();
  az = IMU.getAccelZ_mss();
  gx = IMU.getGyroX_rads();
  gy = IMU.getGyroY_rads();
  gz = IMU.getGyroZ_rads();
  mx = IMU.getMagX_uT();
  my = IMU.getMagY_uT();
  mz = IMU.getMagZ_uT();

  //time elapsed for integration
  delta_t = micros() - timer;
  //get filtered data
  roll = ComplementaryFilter_roll(ax, ay, az, gx, roll);
  pitch = ComplementaryFilter_pitch(ax, ay, az, gy, pitch);
  yaw = ComplementaryFilter_yaw(mx, my, mz, roll, pitch, yaw,gz);
    
  Serial.print(millis());
  Serial.print(",");
  Serial.print(roll);
  Serial.print(",");
  Serial.print(pitch);
  Serial.print(",");
  Serial.println(yaw);

}

float ComplementaryFilter_roll(float ax, float ay, float az, float gy, float roll) {
  long squaresum =(long)ay*ay+(long)az*az;
  roll += gx*(delta_t/1000000.0f); 
  rollAcc = atan(ax/sqrt(squaresum))*180/PI;
  roll = 0.98*roll + 0.02*rollAcc;
  return roll;
}

float ComplementaryFilter_pitch(float ax, float ay, float az, float gy, float pitch) {
  long squaresum =(long)ax*ax+(long)az*az;
  pitch += gy*(delta_t/1000000.0f);
  pitchAcc = -atan(ay/sqrt(squaresum))*180/PI;
  pitch = 0.98*pitch + 0.02*pitchAcc;
  return pitch;
}

float ComplementaryFilter_yaw(float mx, float my, float mz, float roll, float pitch, float yaw, float gz) {
  yaw += gz*(delta_t/1000000.0f);
  long magx = mz*sin(roll*PI/180) - my*cos(roll*PI/180);
  long magy = mx*cos(pitch*PI/180) + my*sin(pitch*PI/180)*sin(roll*PI/180) + mz*sin(pitch*PI/180)*cos(roll*PI/180);
  yaw = 0.8*yaw + 0.2*atan(magx/magy)*180/PI;
  return yaw;
}
