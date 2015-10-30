#include "Arduino.h"
#include <ownI2C.h>
#include "Tiny10DOF.h"





tiny10dof::tiny10dof() {
}


void tiny10dof::initSensors() {
#if defined (MPU6050)
  Gyro_init();
#if defined (MPU6050_I2C_AUX_MASTER) && defined (HMC5883) // init i2c bypass on mpu6050
  i2c_writeReg(MPU6050_ADDRESS, 0x37, 0x02);           //INT_PIN_CFG   -- INT_LEVEL=0 ; INT_OPEN=0 ; LATCH_INT_EN=0 ; INT_RD_CLEAR=0 ; FSYNC_INT_LEVEL=0 ; FSYNC_INT_EN=0 ; I2C_BYPASS_EN=1 ; CLKOUT_EN=0
#endif // MPU6050_I2C_AUX_MASTER
#endif // MPU6050
#if defined (BMP085) || defined (MS561101BA)
  Baro_init();
#endif // BMP085 || MS561101BA
#if defined (HMC5883)
  Mag_init();
#endif // HMC5883
#if defined (MPU6050)
  ACC_init();
#endif // MPU6050
}

void tiny10dof::get_IMU() {
#if defined (MPU6050)
  Gyro_getADC();
  ACC_getADC();
#endif
#if defined (HMC5883)
  Mag_getADC();
#endif
#if defined (BMP085) || defined (MS561101BA)
  Baro_update();
  getEstimatedAltitude();
#endif
  getEstimatedAttitude(); // att.angle[PITCH], att.angle[ROLL]
}


// **************
// gyro+acc IMU
// **************

void tiny10dof::i2c_getSixRawADC(uint8_t add, uint8_t reg) {
  i2c_read_reg_to_buf(add, reg, &rawADC, 6);
}

// ****************
// GYRO common part
// ****************
void tiny10dof::GYRO_Common() {
  static int16_t previousGyroADC[3] = {0, 0, 0};
  static int32_t g[3];
  uint8_t axis, tilt = 0;

  if (calibratingG > 0) {
    for (axis = 0; axis < 3; axis++) {
      // Reset g[axis] at start of calibration
      if (calibratingG == 512) {
        g[axis] = 0;
      }
      // Sum up 512 readings
      g[axis] += imu.gyroADC[axis];
      // Clear global variables for next reading
      imu.gyroADC[axis] = 0;
      gyroZero[axis] = 0;
      if (calibratingG == 1) {
        gyroZero[axis] = (g[axis] + 256) >> 9;
      }
    }
    calibratingG--;
  }
  for (axis = 0; axis < 3; axis++) {
    imu.gyroADC[axis]  -= gyroZero[axis];
    //anti gyro glitch, limit the variation between two consecutive readings
    imu.gyroADC[axis] = constrain(imu.gyroADC[axis], previousGyroADC[axis] - 800, previousGyroADC[axis] + 800);
    previousGyroADC[axis] = imu.gyroADC[axis];
  }
}

// ****************
// ACC common part
// ****************
void tiny10dof::ACC_Common() {
  static int32_t a[3];
  if (calibratingA > 0) {
    for (uint8_t axis = 0; axis < 3; axis++) {
      // Reset a[axis] at start of calibration
      if (calibratingA == 512) a[axis] = 0;
      // Sum up 512 readings
      a[axis] += imu.accADC[axis];
      // Clear global variables for next reading
      imu.accADC[axis] = 0;
      global_conf.accZero[axis] = 0;
    }
    // Calculate average, shift Z down by ACC_1G and store values in EEPROM at end of calibration
    if (calibratingA == 1) {
      global_conf.accZero[ROLL]  = (a[ROLL] + 256) >> 9;
      global_conf.accZero[PITCH] = (a[PITCH] + 256) >> 9;
      global_conf.accZero[YAW]   = ((a[YAW] + 256) >> 9) - ACC_1G; // for nunchuk 200=1G
      conf.angleTrim[ROLL]   = 0;
      conf.angleTrim[PITCH]  = 0;
    }
    calibratingA--;
  }
  imu.accADC[ROLL]  -=  global_conf.accZero[ROLL] ;
  imu.accADC[PITCH] -=  global_conf.accZero[PITCH];
  imu.accADC[YAW]   -=  global_conf.accZero[YAW] ;
}

// ************************************************************************************************************
// I2C Gyroscope and Accelerometer MPU6050
// ************************************************************************************************************
#if defined (MPU6050)
void tiny10dof::Gyro_init() {
  TWBR = ((F_CPU / 400000L) - 16) / 2; // change the I2C clock rate to 400kHz
  i2c_writeReg(MPU6050_ADDRESS, 0x6B, 0x80);             //PWR_MGMT_1    -- DEVICE_RESET 1
  delay(5);
  i2c_writeReg(MPU6050_ADDRESS, 0x6B, 0x03);             //PWR_MGMT_1    -- SLEEP 0; CYCLE 0; TEMP_DIS 0; CLKSEL 3 (PLL with Z Gyro reference)
  i2c_writeReg(MPU6050_ADDRESS, 0x1A, MPU6050_DLPF_CFG); //CONFIG        -- EXT_SYNC_SET 0 (disable input pin for data sync) ; default DLPF_CFG = 0 => ACC bandwidth = 260Hz  GYRO bandwidth = 256Hz)
  i2c_writeReg(MPU6050_ADDRESS, 0x1B, 0x18);             //GYRO_CONFIG   -- FS_SEL = 3: Full scale set to 2000 deg/sec
}

void tiny10dof::Gyro_getADC () {
  i2c_getSixRawADC(MPU6050_ADDRESS, 0x43);
  GYRO_ORIENTATION( ((rawADC[0] << 8) | rawADC[1]) >> 2 , // range: +/- 8192; +/- 2000 deg/sec
                    ((rawADC[2] << 8) | rawADC[3]) >> 2 ,
                    ((rawADC[4] << 8) | rawADC[5]) >> 2 );
  GYRO_Common();
}

void tiny10dof::ACC_init () {
  i2c_writeReg(MPU6050_ADDRESS, 0x1C, 0x10);             //ACCEL_CONFIG  -- AFS_SEL=2 (Full Scale = +/-8G)  ; ACCELL_HPF=0   //note something is wrong in the spec.
  //note: something seems to be wrong in the spec here. With AFS=2 1G = 4096 but according to my measurement: 1G=2048 (and 2048/8 = 256)
  //confirmed here: http://www.multiwii.com/forum/viewtopic.php?f=8&t=1080&start=10#p7480

#if defined (MPU6050_I2C_AUX_MASTER)
  //at this stage, the MAG is configured via the original MAG init function in I2C bypass mode
  //now we configure MPU as a I2C Master device to handle the MAG via the I2C AUX port (done here for HMC5883)
  i2c_writeReg(MPU6050_ADDRESS, 0x6A, 0b00100000);       //USER_CTRL     -- DMP_EN=0 ; FIFO_EN=0 ; I2C_MST_EN=1 (I2C master mode) ; I2C_IF_DIS=0 ; FIFO_RESET=0 ; I2C_MST_RESET=0 ; SIG_COND_RESET=0
  i2c_writeReg(MPU6050_ADDRESS, 0x37, 0x00);             //INT_PIN_CFG   -- INT_LEVEL=0 ; INT_OPEN=0 ; LATCH_INT_EN=0 ; INT_RD_CLEAR=0 ; FSYNC_INT_LEVEL=0 ; FSYNC_INT_EN=0 ; I2C_BYPASS_EN=0 ; CLKOUT_EN=0
  i2c_writeReg(MPU6050_ADDRESS, 0x24, 0x0D);             //I2C_MST_CTRL  -- MULT_MST_EN=0 ; WAIT_FOR_ES=0 ; SLV_3_FIFO_EN=0 ; I2C_MST_P_NSR=0 ; I2C_MST_CLK=13 (I2C slave speed bus = 400kHz)
  i2c_writeReg(MPU6050_ADDRESS, 0x25, 0x80 | MAG_ADDRESS); //I2C_SLV0_ADDR -- I2C_SLV4_RW=1 (read operation) ; I2C_SLV4_ADDR=MAG_ADDRESS
  i2c_writeReg(MPU6050_ADDRESS, 0x26, MAG_DATA_REGISTER);//I2C_SLV0_REG  -- 6 data bytes of MAG are stored in 6 registers. First register address is MAG_DATA_REGISTER
  i2c_writeReg(MPU6050_ADDRESS, 0x27, 0x86);             //I2C_SLV0_CTRL -- I2C_SLV0_EN=1 ; I2C_SLV0_BYTE_SW=0 ; I2C_SLV0_REG_DIS=0 ; I2C_SLV0_GRP=0 ; I2C_SLV0_LEN=3 (3x2 bytes)
#endif
}

void tiny10dof::ACC_getADC () {
  i2c_getSixRawADC(MPU6050_ADDRESS, 0x3B);
  ACC_ORIENTATION( ((rawADC[0] << 8) | rawADC[1]) >> 3 ,
                   ((rawADC[2] << 8) | rawADC[3]) >> 3 ,
                   ((rawADC[4] << 8) | rawADC[5]) >> 3 );
  ACC_Common();
}

//The MAG acquisition function must be replaced because we now talk to the MPU device
#if defined (MPU6050_I2C_AUX_MASTER)
void tiny10dof::Device_Mag_getADC() {
  i2c_getSixRawADC(MPU6050_ADDRESS, 0x49);               //0x49 is the first memory room for EXT_SENS_DATA
  MAG_ORIENTATION( ((rawADC[0] << 8) | rawADC[1]) ,
                   ((rawADC[4] << 8) | rawADC[5]) ,
                   ((rawADC[2] << 8) | rawADC[3]) );
}
#endif
#endif

int16_t tiny10dof::_atan2(int32_t y, int32_t x) {
  float z = (float)y / x;
  int16_t a;
  if ( abs(y) < abs(x) ) {
    a = 573 * z / (1.0f + 0.28f * z * z);
    if (x < 0) {
      if (y < 0) a -= 1800;
      else a += 1800;
    }
  } else {
    a = 900 - 573 * z / (z * z + 0.28f);
    if (y < 0) a -= 1800;
  }
  return a;
}

float tiny10dof::InvSqrt (float x) {
  union {
    int32_t i;
    float   f;
  } conv;
  conv.f = x;
  conv.i = 0x5f3759df - (conv.i >> 1);
  return 0.5f * conv.f * (3.0f - x * conv.f * conv.f);
}

// Rotate Estimated vector(s) with small angle approximation, according to the gyro data
void tiny10dof::rotateV(struct fp_vector *v, float* delta) {
  fp_vector v_tmp = *v;
  v->Z -= delta[ROLL]  * v_tmp.X + delta[PITCH] * v_tmp.Y;
  v->X += delta[ROLL]  * v_tmp.Z - delta[YAW]   * v_tmp.Y;
  v->Y += delta[PITCH] * v_tmp.Z + delta[YAW]   * v_tmp.X;
}

void tiny10dof::getEstimatedAttitude() {
  uint8_t axis;
  int32_t accMag = 0;
  float scale, deltaGyroAngle[3];
  uint8_t validAcc;
  static uint16_t previousT;
  uint16_t currentT = micros();

  scale = (currentT - previousT) * GYRO_SCALE; // GYRO_SCALE unit: radian/microsecond
  previousT = currentT;

  // Initialization
  for (axis = 0; axis < 3; axis++) {
    deltaGyroAngle[axis] = imu.gyroADC[axis]  * scale; // radian

    accLPF32[axis]    -= accLPF32[axis] >> ACC_LPF_FACTOR;
    accLPF32[axis]    += imu.accADC[axis];
    imu.accSmooth[axis]    = accLPF32[axis] >> ACC_LPF_FACTOR;

    accMag += (int32_t)imu.accSmooth[axis] * imu.accSmooth[axis] ;
  }

  rotateV(&EstG.V, deltaGyroAngle);
#if defined (HMC5883)
  rotateV(&EstM.V, deltaGyroAngle);
#endif

  accMag = accMag * 100 / ((int32_t)ACC_1G * ACC_1G);
  validAcc = 72 < (uint16_t)accMag && (uint16_t)accMag < 133;
  // Apply complimentary filter (Gyro drift correction)
  // If accel magnitude >1.15G or <0.85G and ACC vector outside of the limit range => we neutralize the effect of accelerometers in the angle estimation.
  // To do that, we just skip filter, as EstV already rotated by Gyro
  for (axis = 0; axis < 3; axis++) {
    if ( validAcc )
      EstG.A[axis] = (EstG.A[axis] * GYR_CMPF_FACTOR + imu.accSmooth[axis]) * INV_GYR_CMPF_FACTOR;
    EstG32.A[axis] = EstG.A[axis]; //int32_t cross calculation is a little bit faster than float
#if defined (HMC5883)
    EstM.A[axis] = (EstM.A[axis] * GYR_CMPFM_FACTOR  + imu.magADC[axis]) * INV_GYR_CMPFM_FACTOR;
    EstM32.A[axis] = EstM.A[axis];
#endif
  }

  // Attitude of the estimated vector
  int32_t sqGX_sqGZ = sq(EstG32.V.X) + sq(EstG32.V.Z);
  invG = InvSqrt(sqGX_sqGZ + sq(EstG32.V.Y));
  att.angle[PITCH]  = _atan2(EstG32.V.X , EstG32.V.Z);
  att.angle[ROLL] = _atan2(EstG32.V.Y , InvSqrt(sqGX_sqGZ) * sqGX_sqGZ);
#if defined (HMC5883)
  att.heading = _atan2(
                  EstM32.V.Z * EstG32.V.X - EstM32.V.X * EstG32.V.Z,
                  (EstM.V.Y * sqGX_sqGZ  - (EstM32.V.X * EstG32.V.X + EstM32.V.Z * EstG32.V.Z) * EstG.V.Y) * invG );
  att.heading += conf.mag_declination; // Set from GUI
  att.heading /= 10;
  if (att.heading < 0) att.trueHeading = att.heading + 360;
  else att.trueHeading = att.heading;
#endif
}

#if defined (HMC5883)
void tiny10dof::Mag_getADC() { // return 1 when news values are available, 0 otherwise
  static uint32_t t;
  static int16_t magZeroTempMin[3];
  static int16_t magZeroTempMax[3];
  uint8_t axis;
  if ( cur_cycle < t ) return; //each read is spaced by 100ms
  t = cur_cycle + 100000;
  TWBR = ((F_CPU / 400000L) - 16) / 2; // change the I2C clock rate to 400kHz
  Device_Mag_getADC();
  imu.magADC[ROLL]  = imu.magADC[ROLL]  * magGain[ROLL];
  imu.magADC[PITCH] = imu.magADC[PITCH] * magGain[PITCH];
  imu.magADC[YAW]   = imu.magADC[YAW]   * magGain[YAW];
  if (calibratingM) {
    calibrating_Mag = t;
    for (axis = 0; axis < 3; axis++) {
      global_conf.magZero[axis] = 0;
      magZeroTempMin[axis] = imu.magADC[axis];
      magZeroTempMax[axis] = imu.magADC[axis];
    }
    calibratingM = 0;
  }
  if (magInit) { // we apply offset only once mag calibration is done
    imu.magADC[ROLL]  -= global_conf.magZero[ROLL];
    imu.magADC[PITCH] -= global_conf.magZero[PITCH];
    imu.magADC[YAW]   -= global_conf.magZero[YAW];
  }

  if (calibrating_Mag != 0) {
    if ((t - calibrating_Mag) < 30000000) { // 30s: you have 30s to turn the multi in all directions
      for (axis = 0; axis < 3; axis++) {
        if (imu.magADC[axis] < magZeroTempMin[axis]) magZeroTempMin[axis] = imu.magADC[axis];
        if (imu.magADC[axis] > magZeroTempMax[axis]) magZeroTempMax[axis] = imu.magADC[axis];
      }
    }
    else {
      calibrating_Mag = 0;
      for (axis = 0; axis < 3; axis++)
        global_conf.magZero[axis] = (magZeroTempMin[axis] + magZeroTempMax[axis]) >> 1;
    }
  }
}

void tiny10dof::Mag_init() {
  int32_t xyz_total[3] = {0, 0, 0}; // 32 bit totals so they won't overflow.
  bool bret = true;              // Error indicator

  delay(50);  //Wait before start
  i2c_writeReg(MAG_ADDRESS, HMC58X3_R_CONFA, 0x010 + HMC_POS_BIAS); // Reg A DOR=0x010 + MS1,MS0 set to pos bias

  // Note that the  very first measurement after a gain change maintains the same gain as the previous setting.
  // The new gain setting is effective from the second measurement and on.

  i2c_writeReg(MAG_ADDRESS, HMC58X3_R_CONFB, 2 << 5);  //Set the Gain
  i2c_writeReg(MAG_ADDRESS, HMC58X3_R_MODE, 1);
  delay(100);
  getADC();  //Get one sample, and discard it

  for (uint8_t i = 0; i < 10; i++) { //Collect 10 samples
    i2c_writeReg(MAG_ADDRESS, HMC58X3_R_MODE, 1);
    delay(100);
    getADC();   // Get the raw values in case the scales have already been changed.

    // Since the measurements are noisy, they should be averaged rather than taking the max.
    xyz_total[0] += imu.magADC[0];
    xyz_total[1] += imu.magADC[1];
    xyz_total[2] += imu.magADC[2];

    // Detect saturation.
    if (-(1 << 12) >= min(imu.magADC[0], min(imu.magADC[1], imu.magADC[2]))) {
      bret = false;
      break;  // Breaks out of the for loop.  No sense in continuing if we saturated.
    }
  }

  // Apply the negative bias. (Same gain)
  i2c_writeReg(MAG_ADDRESS, HMC58X3_R_CONFA, 0x010 + HMC_NEG_BIAS); // Reg A DOR=0x010 + MS1,MS0 set to negative bias.
  for (uint8_t i = 0; i < 10; i++) {
    i2c_writeReg(MAG_ADDRESS, HMC58X3_R_MODE, 1);
    delay(100);
    getADC();  // Get the raw values in case the scales have already been changed.

    // Since the measurements are noisy, they should be averaged.
    xyz_total[0] -= imu.magADC[0];
    xyz_total[1] -= imu.magADC[1];
    xyz_total[2] -= imu.magADC[2];

    // Detect saturation.
    if (-(1 << 12) >= min(imu.magADC[0], min(imu.magADC[1], imu.magADC[2]))) {
      bret = false;
      break;  // Breaks out of the for loop.  No sense in continuing if we saturated.
    }
  }

  magGain[0] = fabs(820.0 * HMC58X3_X_SELF_TEST_GAUSS * 2.0 * 10.0 / xyz_total[0]);
  magGain[1] = fabs(820.0 * HMC58X3_Y_SELF_TEST_GAUSS * 2.0 * 10.0 / xyz_total[1]);
  magGain[2] = fabs(820.0 * HMC58X3_Z_SELF_TEST_GAUSS * 2.0 * 10.0 / xyz_total[2]);

  // leave test mode
  i2c_writeReg(MAG_ADDRESS , HMC58X3_R_CONFA , 0x70 ); //Configuration Register A  -- 0 11 100 00  num samples: 8 ; output rate: 15Hz ; normal measurement mode
  i2c_writeReg(MAG_ADDRESS , HMC58X3_R_CONFB , 0x20 ); //Configuration Register B  -- 001 00000    configuration gain 1.3Ga
  i2c_writeReg(MAG_ADDRESS , HMC58X3_R_MODE  , 0x00 ); //Mode register             -- 000000 00    continuous Conversion Mode
  delay(100);
  magInit = 1;

  if (!bret) { //Something went wrong so get a best guess
    magGain[0] = 1.0;
    magGain[1] = 1.0;
    magGain[2] = 1.0;
  }
} //  Mag_init().
#endif

#if defined (HMC5883)
void tiny10dof::getADC() {
  i2c_getSixRawADC(MAG_ADDRESS, MAG_DATA_REGISTER);
  MAG_ORIENTATION( ((rawADC[0] << 8) | rawADC[1]) ,
                   ((rawADC[4] << 8) | rawADC[5]) ,
                   ((rawADC[2] << 8) | rawADC[3]) );
}
#endif

#if !defined (MPU6050_I2C_AUX_MASTER)
void tiny10dof::Device_Mag_getADC() {
  getADC();
}
#endif

#if defined (BMP085) || defined (MS561101BA)
void tiny10dof::Baro_Common() {
  static int32_t baroHistTab[BARO_TAB_SIZE];
  static uint8_t baroHistIdx;

  uint8_t indexplus1 = (baroHistIdx + 1);
  if (indexplus1 == BARO_TAB_SIZE) indexplus1 = 0;
  baroHistTab[baroHistIdx] = baroPressure;
  baroPressureSum += baroHistTab[baroHistIdx];
  baroPressureSum -= baroHistTab[indexplus1];
  baroHistIdx = indexplus1;
}

void tiny10dof::getEstimatedAltitude() {
  int32_t  BaroAlt;
  static float baroGroundTemperatureScale, logBaroGroundPressureSum;
  static float vel = 0.0f;
  static uint16_t previousT;
  uint16_t currentT = micros();
  uint16_t dTime;

  dTime = currentT - previousT;
  if (dTime < UPDATE_INTERVAL) return;
  previousT = currentT;

  if (calibratingB > 0) {
    logBaroGroundPressureSum = log(102325 * 20);    // baroPressureSum
    //    Serial.println(baroPressureSum/20);
    baroGroundTemperatureScale = (baroTemperature + 27315) *  29.271267f;
    calibratingB--;
  }

  // baroGroundPressureSum is not supposed to be 0 here
  // see: https://code.google.com/p/ardupilot-mega/source/browse/libraries/AP_Baro/AP_Baro.cpp
  BaroAlt = ( logBaroGroundPressureSum - log(baroPressureSum) ) * baroGroundTemperatureScale;

  alt.EstAlt = (alt.EstAlt * 6 + BaroAlt * 2) >> 3; // additional LPF to reduce baro noise (faster by 30 µs)

  //#if (defined (VARIOMETER) && (VARIOMETER != 2)) || !defined (SUPPRESS_BARO_ALTHOLD)
  //  //P
  //  int16_t error16 = constrain(AltHold - alt.EstAlt, -300, 300);
  //  applyDeadband(error16, 10); //remove small P parametr to reduce noise near zero position
  //  BaroPID = constrain((conf.pid[PIDALT].P8 * error16 >> 7), -150, +150);
  //
  //  //I
  //  errorAltitudeI += conf.pid[PIDALT].I8 * error16 >> 6;
  //  errorAltitudeI = constrain(errorAltitudeI, -30000, 30000);
  //  BaroPID += errorAltitudeI >> 9; //I in range +/-60
  //
  //  // projection of ACC vector to global Z, with 1G subtructed
  //  // Math: accZ = A * G / |G| - 1G
  //  int16_t accZ = (imu.accSmooth[ROLL] * EstG32.V.X + imu.accSmooth[PITCH] * EstG32.V.Y + imu.accSmooth[YAW] * EstG32.V.Z) * invG;
  //
  //  static int16_t accZoffset = 0;
  //  if (!f.ARMED) {
  //    accZoffset -= accZoffset >> 3;
  //    accZoffset += accZ;
  //  }
  //  accZ -= accZoffset >> 3;
  //  applyDeadband(accZ, ACC_Z_DEADBAND);
  //
  //  static int32_t lastBaroAlt;
  //  //int16_t baroVel = (alt.EstAlt - lastBaroAlt) * 1000000.0f / dTime;
  //  int16_t baroVel = (alt.EstAlt - lastBaroAlt) * (1000000 / UPDATE_INTERVAL);
  //  lastBaroAlt = alt.EstAlt;
  //
  //  baroVel = constrain(baroVel, -300, 300); // constrain baro velocity +/- 300cm/s
  //  applyDeadband(baroVel, 10); // to reduce noise near zero
  //
  //  // Integrator - velocity, cm/sec
  //  vel += accZ * ACC_VelScale * dTime;
  //
  //  // apply Complimentary Filter to keep the calculated velocity based on baro velocity (i.e. near real velocity).
  //  // By using CF it's possible to correct the drift of integrated accZ (velocity) without loosing the phase, i.e without delay
  //  vel = vel * 0.985f + baroVel * 0.015f;
  //
  //  //D
  //  alt.vario = vel;
  //  applyDeadband(alt.vario, 5);
  //  BaroPID -= constrain(conf.pid[PIDALT].D8 * alt.vario >> 4, -150, 150);
  //#endif
//  return 1;
}
#endif // BMP085 || MS561101BA

#if defined (BMP085)
void tiny10dof::i2c_BMP085_readCalibration() {
  delay(10);
  //read calibration data in one go
  size_t s_bytes = (uint8_t*)&bmp085_ctx.md - (uint8_t*)&bmp085_ctx.ac1 + sizeof(bmp085_ctx.ac1);
  i2c_read_reg_to_buf(BMP085_ADDRESS, 0xAA, &bmp085_ctx.ac1, s_bytes);
  // now fix endianness
  int16_t *p;
  for (p = &bmp085_ctx.ac1; p <= &bmp085_ctx.md; p++) {
    swap_endianness(p, sizeof(*p));
  }
}

void  tiny10dof::Baro_init() {
  delay(10);
  i2c_BMP085_readCalibration();
  delay(5);
  i2c_BMP085_UT_Start();
  bmp085_ctx.deadline = cur_cycle + 5000;
}

// read uncompensated temperature value: send command first
void tiny10dof::i2c_BMP085_UT_Start(void) {
  i2c_writeReg(BMP085_ADDRESS, 0xf4, 0x2e);
  i2c_rep_start(BMP085_ADDRESS << 1);
  i2c_write(0xF6);
  i2c_stop();
}

// read uncompensated pressure value: send command first)
void tiny10dof::i2c_BMP085_UP_Start () {
  i2c_writeReg(BMP085_ADDRESS, 0xf4, 0x34 + (OSS << 6)); // control register value for oversampling setting 3
  i2c_rep_start(BMP085_ADDRESS << 1); //I2C write direction => 0
  i2c_write(0xF6);
  i2c_stop();
}

// read uncompensated pressure value: read result bytes
// the datasheet suggests a delay of 25.5 ms (oversampling settings 3) after the send command
void tiny10dof::i2c_BMP085_UP_Read () {
  i2c_rep_start((BMP085_ADDRESS << 1) | 1); //I2C read direction => 1
  bmp085_ctx.up.raw[2] = i2c_readAck();
  bmp085_ctx.up.raw[1] = i2c_readAck();
  bmp085_ctx.up.raw[0] = i2c_readNak();
}

// read uncompensated temperature value: read result bytes
// the datasheet suggests a delay of 4.5 ms after the send command
void tiny10dof::i2c_BMP085_UT_Read() {
  i2c_rep_start((BMP085_ADDRESS << 1) | 1); //I2C read direction => 1
  bmp085_ctx.ut.raw[1] = i2c_readAck();
  bmp085_ctx.ut.raw[0] = i2c_readNak();
}

void tiny10dof::i2c_BMP085_Calculate() {
  int32_t  x1, x2, x3, b3, b5, b6, p, tmp;
  uint32_t b4, b7;
  // Temperature calculations
  x1 = ((int32_t)bmp085_ctx.ut.val - bmp085_ctx.ac6) * bmp085_ctx.ac5 >> 15;
  x2 = ((int32_t)bmp085_ctx.mc << 11) / (x1 + bmp085_ctx.md);
  b5 = x1 + x2;
  baroTemperature = (b5 * 10 + 8) >> 4; // in 0.01 degC (same as MS561101BA temperature)
  // Pressure calculations
  b6 = b5 - 4000;
  x1 = (bmp085_ctx.b2 * (b6 * b6 >> 12)) >> 11;
  x2 = bmp085_ctx.ac2 * b6 >> 11;
  x3 = x1 + x2;
  tmp = bmp085_ctx.ac1;
  tmp = (tmp * 4 + x3) << OSS;
  b3 = (tmp + 2) / 4;
  x1 = bmp085_ctx.ac3 * b6 >> 13;
  x2 = (bmp085_ctx.b1 * (b6 * b6 >> 12)) >> 16;
  x3 = ((x1 + x2) + 2) >> 2;
  b4 = (bmp085_ctx.ac4 * (uint32_t)(x3 + 32768)) >> 15;
  b7 = ((uint32_t) (bmp085_ctx.up.val >> (8 - OSS)) - b3) * (50000 >> OSS);
  p = b7 < 0x80000000 ? (b7 * 2) / b4 : (b7 / b4) * 2;
  x1 = (p >> 8) * (p >> 8);
  x1 = (x1 * 3038) >> 16;
  x2 = (-7357 * p) >> 16;
  baroPressure = p + ((x1 + x2 + 3791) >> 4);
}

//return 0: no data available, no computation ;  1: new value available  ; 2: no new value, but computation time
void tiny10dof::Baro_update() {                   // first UT conversion is started in init procedure
  if (cur_cycle < bmp085_ctx.deadline) return 0;
  bmp085_ctx.deadline = cur_cycle + 6000; // 1.5ms margin according to the spec (4.5ms T convetion time)
  TWBR = ((F_CPU / 400000L) - 16) / 2; // change the I2C clock rate to 400kHz, BMP085 is ok with this speed
  if (bmp085_ctx.state == 0) {
    i2c_BMP085_UT_Read();
    i2c_BMP085_UP_Start();
    bmp085_ctx.state = 1;
    Baro_Common();
    bmp085_ctx.deadline += 21000;   // 6000+21000=27000 1.5ms margin according to the spec (25.5ms P convetion time with OSS=3)
  } else {
    i2c_BMP085_UP_Read();
    i2c_BMP085_UT_Start();
    i2c_BMP085_Calculate();
    bmp085_ctx.state = 0;
  }
}
#endif // BMP085

#if defined (MS561101BA)
void tiny10dof::i2c_MS561101BA_reset() {
  i2c_writeReg(MS561101BA_ADDRESS, MS561101BA_RESET, 0);
}

void tiny10dof::i2c_MS561101BA_readCalibration() {
  union {
    uint16_t val;
    uint8_t raw[2];
  } data;
  for (uint8_t i = 0; i < 6; i++) {
    i2c_rep_start(MS561101BA_ADDRESS << 1);
    i2c_write(0xA2 + 2 * i);
    delay(10);
    i2c_rep_start((MS561101BA_ADDRESS << 1) | 1); //I2C read direction => 1
    delay(10);
    data.raw[1] = i2c_readAck();  // read a 16 bit register
    data.raw[0] = i2c_readNak();
    ms561101ba_ctx.c[i + 1] = data.val;
  }
}

void  tiny10dof::Baro_init() {
  delay(10);
  i2c_MS561101BA_reset();
  delay(100);
  i2c_MS561101BA_readCalibration();
  delay(10);
  i2c_MS561101BA_UT_Start();
  ms561101ba_ctx.deadline = cur_cycle + 10000;
}

// read uncompensated temperature value: send command first
void tiny10dof::i2c_MS561101BA_UT_Start() {
  i2c_rep_start(MS561101BA_ADDRESS << 1);    // I2C write direction
  i2c_write(MS561101BA_TEMPERATURE + OSR);  // register selection
  i2c_stop();
}

// read uncompensated pressure value: send command first
void tiny10dof::i2c_MS561101BA_UP_Start() {
  i2c_rep_start(MS561101BA_ADDRESS << 1);    // I2C write direction
  i2c_write(MS561101BA_PRESSURE + OSR);     // register selection
  i2c_stop();
}

// read uncompensated pressure value: read result bytes
void tiny10dof::i2c_MS561101BA_UP_Read() {
  i2c_rep_start(MS561101BA_ADDRESS << 1);
  i2c_write(0);
  i2c_rep_start((MS561101BA_ADDRESS << 1) | 1);
  ms561101ba_ctx.up.raw[2] = i2c_readAck();
  ms561101ba_ctx.up.raw[1] = i2c_readAck();
  ms561101ba_ctx.up.raw[0] = i2c_readNak();
}

// read uncompensated temperature value: read result bytes
void tiny10dof::i2c_MS561101BA_UT_Read() {
  i2c_rep_start(MS561101BA_ADDRESS << 1);
  i2c_write(0);
  i2c_rep_start((MS561101BA_ADDRESS << 1) | 1);
  ms561101ba_ctx.ut.raw[2] = i2c_readAck();
  ms561101ba_ctx.ut.raw[1] = i2c_readAck();
  ms561101ba_ctx.ut.raw[0] = i2c_readNak();
}

void tiny10dof::i2c_MS561101BA_Calculate() {
  int32_t off2, sens2, delt;

  int64_t dT       = (int32_t)ms561101ba_ctx.ut.val - ((int32_t)ms561101ba_ctx.c[5] << 8);
  baroTemperature  = 2000 + ((dT * ms561101ba_ctx.c[6]) >> 23);
  int64_t off      = ((uint32_t)ms561101ba_ctx.c[2] << 16) + ((dT * ms561101ba_ctx.c[4]) >> 7);
  int64_t sens     = ((uint32_t)ms561101ba_ctx.c[1] << 15) + ((dT * ms561101ba_ctx.c[3]) >> 8);

  if (baroTemperature < 2000) { // temperature lower than 20st.C
    delt = baroTemperature - 2000;
    delt  = 5 * delt * delt;
    off2  = delt >> 1;
    sens2 = delt >> 2;
    if (baroTemperature < -1500) { // temperature lower than -15st.C
      delt  = baroTemperature + 1500;
      delt  = delt * delt;
      off2  += 7 * delt;
      sens2 += (11 * delt) >> 1;
    }
    off  -= off2;
    sens -= sens2;
  }

  baroPressure     = (( (ms561101ba_ctx.up.val * sens ) >> 21) - off) >> 15;
}

//return 0: no data available, no computation ;  1: new value available  ; 2: no new value, but computation time
void tiny10dof::Baro_update() {                            // first UT conversion is started in init procedure
  if (cur_cycle < ms561101ba_ctx.deadline) return;
  ms561101ba_ctx.deadline = cur_cycle + 10000; // UT and UP conversion take 8.5ms so we do next reading after 10ms
  TWBR = ((F_CPU / 400000L) - 16) / 2;          // change the I2C clock rate to 400kHz, MS5611 is ok with this speed
  if (ms561101ba_ctx.state == 0) {
    i2c_MS561101BA_UT_Read();
    i2c_MS561101BA_UP_Start();
    Baro_Common();                              // moved here for less timecycle spike
    ms561101ba_ctx.state = 1;
  } else {
    i2c_MS561101BA_UP_Read();
    i2c_MS561101BA_UT_Start();
    i2c_MS561101BA_Calculate();
    ms561101ba_ctx.state = 0;
  }
}
#endif // MS561101BA

void tiny10dof::getPRY(int16_t *pit, int16_t *rol, uint16_t *yaw) {
  calc_cycleTime();
#if defined (MPU6050)
  Gyro_getADC();
  ACC_getADC();
#endif
#if defined (HMC5883)
  Mag_getADC();
#endif
  getEstimatedAttitude();
  *pit = att.angle[PITCH];
  *rol = att.angle[ROLL];
  *yaw = att.trueHeading;
}


void tiny10dof::calc_cycleTime() {
  cur_cycle = micros();
}

void tiny10dof::calSensor(uint8_t sensor) {
  switch (sensor) {
    case MPU:
      calibratingG = 512;
      calibratingA = 512;
      break;
    case BARO:
      calibratingB = 2;
      break;
    case MAG:
      calibratingM = 1;
      break;
    default:
      break;
  }
  while (calibratingA || calibratingG || calibratingB || calibrating_Mag) {
    calc_cycleTime();
    get_IMU();
  }
}

void tiny10dof::getAlt(int32_t *alti) {
  calc_cycleTime();
  Baro_update();
  getEstimatedAltitude();
  *alti = alt.EstAlt;
}
