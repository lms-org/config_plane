
#include "ownI2C.h"
#include "Tiny10DOF.h"

/* this library reads any 10DOF with this configuration:
- MPU 6050
- MS561101BA
- HMC5883 on MPU-AUX

make sure you installed the ownI2C lib.

The library returns pitch, roll, yaw filtered by a complementary filter as well as the absolute altitude above zero.
*/

tiny10dof imu;

void setup() {
  setup_i2c(); 
  Serial.begin(115200);
  Serial.println("Init Sensors");
  imu.initSensors();
  Serial.println("Begin calibration. Don't move");
  imu.calSensor(MPU);
  imu.calSensor(BARO);
  Serial.println("calibration done");
}

int16_t pitch, roll;
uint16_t yaw;
int32_t alt;

void loop() {
  imu.getPRY(&pitch, &roll, &yaw);
  imu.getAlt(&alt);
  Serial.print(pitch); Serial.print("\t");
  Serial.print(roll); Serial.print("\t");
  Serial.print(yaw); Serial.print("\t");
  Serial.println(alt);
  delay(5);
}
