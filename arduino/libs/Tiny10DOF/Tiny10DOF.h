#ifndef TINY10DOF_H
#define TINY10DOF_H

#include "Arduino.h"

#define MPU6050                   // Acc + Gyro
#define HMC5883                   // magnetometer
//#define BMP085                  // barometer
#define MS561101BA                // barometer
#define MPU6050_I2C_AUX_MASTER    // magneto connected to aux-i2c of mpu 6050

/************************************/
/*          DEFINITIONS             */
/************************************/

#if defined (__AVR_ATmega168__) || defined (__AVR_ATmega328P__)
#define ATMEGA328
#elif defined (__AVR_ATmega32U4__) || defined (TEENSY20)
#define PROMICRO
#elif defined (__AVR_ATmega1280__) || defined (__AVR_ATmega1281__) || defined (__AVR_ATmega2560__) || defined (__AVR_ATmega2561__)
#define ATMEGA2560
#endif

enum axis {
  ROLL,
  PITCH,
  YAW
};


#define ACC_1G 512

//MPU6050 Gyro LPF setting
#define MPU6050_DLPF_CFG   0

#define MPU6050_ADDRESS     0x68
#define BMP085_ADDRESS      0x77
#define MAG_ADDRESS         0x1E
#define MS561101BA_ADDRESS 0x77 //CBR=0 0xEE I2C address when pin CSB is connected to LOW (GND)
//#define MS561101BA_ADDRESS 0x76 //CBR=1 0xEC I2C address when pin CSB is connected to HIGH (VCC)

#define GYRO_SCALE (4 / 16.4 * PI / 180.0 / 1000000.0)   //MPU6050 and MPU3050   16.4 LSB/(deg/s) and we ignore the last 2 bits

#if defined (HMC5883)
#define HMC58X3_R_CONFA 0
#define HMC58X3_R_CONFB 1
#define HMC58X3_R_MODE 2
#define HMC58X3_X_SELF_TEST_GAUSS (+1.16)                       //!< X axis level when bias current is applied.
#define HMC58X3_Y_SELF_TEST_GAUSS (+1.16)   //!< Y axis level when bias current is applied.
#define HMC58X3_Z_SELF_TEST_GAUSS (+1.08)                       //!< Y axis level when bias current is applied.
#define SELF_TEST_LOW_LIMIT  (243.0/390.0)   //!< Low limit when gain is 5.
#define SELF_TEST_HIGH_LIMIT (575.0/390.0)   //!< High limit when gain is 5.
#define HMC_POS_BIAS 1
#define HMC_NEG_BIAS 2
#define MAG_DATA_REGISTER 0x03
#endif

#if defined (BMP085) || defined (MS561101BA)
#define UPDATE_INTERVAL 25000
#define BARO_TAB_SIZE   21
#define OSS 3
#define ACC_Z_DEADBAND (ACC_1G>>5) // was 40 instead of 32 now
#define ACC_VelScale (9.80665f / 10000.0f / ACC_1G)
#endif

#if defined (MS561101BA)
// registers of the device
#define MS561101BA_PRESSURE    0x40
#define MS561101BA_TEMPERATURE 0x50
#define MS561101BA_RESET       0x1E

// OSR (Over Sampling Ratio) constants
#define MS561101BA_OSR_256  0x00
#define MS561101BA_OSR_512  0x02
#define MS561101BA_OSR_1024 0x04
#define MS561101BA_OSR_2048 0x06
#define MS561101BA_OSR_4096 0x08

#define OSR MS561101BA_OSR_4096
#endif // MS561101BA

//******  advanced users settings *******************
/* Set the Low Pass Filter factor for ACC
   Increasing this value would reduce ACC noise (visible in GUI), but would increase ACC lag time
   Comment this if  you do not want filter at all.
   unit = n power of 2 */
// this one is also used for ALT HOLD calculation, should not be changed
#ifndef ACC_LPF_FACTOR
#define ACC_LPF_FACTOR 4 // that means a LPF of 16
#endif

/* Set the Gyro Weight for Gyro/Acc complementary filter
   Increasing this value would reduce and delay Acc influence on the output of the filter*/
#ifndef GYR_CMPF_FACTOR
#define GYR_CMPF_FACTOR 600
#endif

/* Set the Gyro Weight for Gyro/Magnetometer complementary filter
   Increasing this value would reduce and delay Magnetometer influence on the output of the filter*/
#define GYR_CMPFM_FACTOR 250

//****** end of advanced users settings *************
#define INV_GYR_CMPF_FACTOR   (1.0f / (GYR_CMPF_FACTOR  + 1.0f))
#define INV_GYR_CMPFM_FACTOR  (1.0f / (GYR_CMPFM_FACTOR + 1.0f))

enum {
  MPU,
  BARO,
  MAG
};


// IMU
#define ACC_ORIENTATION(X, Y, Z)  {imu.accADC[ROLL]  = -Y; imu.accADC[PITCH]  =  -X; imu.accADC[YAW]  =  Z;}
#define GYRO_ORIENTATION(X, Y, Z) {imu.gyroADC[ROLL] = -X; imu.gyroADC[PITCH] =   Y; imu.gyroADC[YAW] = -Z;}
#define MAG_ORIENTATION(X, Y, Z)  {imu.magADC[ROLL]  =  X; imu.magADC[PITCH]  =   Y; imu.magADC[YAW]  = -Z;}


// I2C
#define STANDARD_I2C_SPEED 400000L
//#define INTERNAL_I2C_PULLUPS


#define applyDeadband(value, deadband)  \
  if(abs(value) < deadband) {           \
    value = 0;                          \
  } else if(value > 0){                 \
    value -= deadband;                  \
  } else if(value < 0){                 \
    value += deadband;                  \
  }


typedef struct {
  int16_t  accSmooth[3];
  int16_t  gyroData[3];
  int16_t  magADC[3];
  int16_t  gyroADC[3];
  int16_t  accADC[3];
} imu_t;
typedef struct {
  int16_t angle[2];            // absolute angle inclination in multiple of 0.1 degree    180 deg = 1800
  int16_t heading;             // variometer in cm/s
  uint16_t trueHeading;
} att_t;
typedef struct {
  int16_t angleTrim[2];
  int16_t mag_declination;
} conf_t;
typedef struct {
  int16_t accZero[3];
  int16_t magZero[3];
} global_conf_t;
typedef struct fp_vector {
  float X, Y, Z;
} t_fp_vector_def;
typedef union {
  float A[3];
  t_fp_vector_def V;
} t_fp_vector;
typedef struct int32_t_vector {
  int32_t X, Y, Z;
} t_int32_t_vector_def;
typedef union {
  int32_t A[3];
  t_int32_t_vector_def V;
} t_int32_t_vector;
typedef struct {
  int32_t  EstAlt;             // in cm
  int16_t  vario;              // variometer in cm/s
} alt_t;
struct {
  // sensor registers from the MS561101BA datasheet
  uint16_t c[7];
  union {
    uint32_t val;
    uint8_t raw[4];
  } ut; //uncompensated T
  union {
    uint32_t val;
    uint8_t raw[4];
  } up; //uncompensated P
  uint8_t  state;
  uint32_t deadline;
} ms561101ba_ctx;
struct {
  // sensor registers from the BOSCH BMP085 datasheet
  int16_t  ac1, ac2, ac3;
  uint16_t ac4, ac5, ac6;
  int16_t  b1, b2, mb, mc, md;
  union {
    uint16_t val;
    uint8_t raw[2];
  } ut; //uncompensated T
  union {
    uint32_t val;
    uint8_t raw[4];
  } up; //uncompensated P
  uint8_t  state;
  uint32_t deadline;
} bmp085_ctx;

class tiny10dof
{
  public:
    tiny10dof();
    void initSensors();
    void calSensor(uint8_t sensor);
    void getPRY(int16_t *pit, int16_t *rol, uint16_t *yaw);
    void getAlt(int32_t *alti);
  private:

    void get_IMU();
    void calc_cycleTime();
    void ACC_init ();
    void ACC_Common();
    void ACC_getADC ();

    void Gyro_init();
    void GYRO_Common();
    void Gyro_getADC ();

    void Baro_init();
    void Baro_Common();
    void Baro_update();

    void Mag_init();
    void Device_Mag_getADC();
    void Mag_getADC();

    void getADC();
    void i2c_getSixRawADC(uint8_t add, uint8_t reg);
    void getEstimatedAttitude();
    void getEstimatedAltitude();

    uint8_t calculate_sum(uint8_t *cb , uint8_t siz);
    int16_t _atan2(int32_t y, int32_t x);
    void rotateV(struct fp_vector *v, float* delta);
    float InvSqrt (float x);
#if defined (MS561101BA)
    void i2c_MS561101BA_Calculate();
    void i2c_MS561101BA_UT_Read();
    void i2c_MS561101BA_UP_Read();
    void i2c_MS561101BA_UP_Start();
    void i2c_MS561101BA_UT_Start();
    void i2c_MS561101BA_readCalibration();
    void i2c_MS561101BA_reset();
#elif defined (BMP085)
    void i2c_BMP085_Calculate();
    void i2c_BMP085_UT_Read();
    void i2c_BMP085_UP_Read ();
    void i2c_BMP085_UP_Start ();
    void i2c_BMP085_UT_Start(void);
    void i2c_BMP085_readCalibration();
#endif

    uint32_t cur_cycle, last_cycle;
    uint16_t calibratingG;
    uint16_t calibratingA;
    uint16_t calibratingB;  // baro calibration = get new ground pressure value
    uint32_t calibrating_Mag;
    boolean calibratingM;
    uint8_t rawADC[6];
    uint32_t neutralizeTime = 0;
    int16_t gyroZero[3] = {0, 0, 0};
    att_t att;
    global_conf_t global_conf;
    conf_t conf;
    int32_t accLPF32[3]    = {0, 0, 1};
    float invG; // 1/|G|
    t_fp_vector EstG;
    t_int32_t_vector EstG32;
    t_int32_t_vector EstM32;
    t_fp_vector EstM;
    float   magGain[3] = {1.0, 1.0, 1.0}; // gain for each axis, populated at sensor init
    uint8_t magInit = 0;
    int32_t baroPressure;
    int32_t baroTemperature;
    int32_t baroPressureSum;
    alt_t alt;
    int32_t  AltHold; // in cm
    int16_t  BaroPID = 0;
    int16_t  errorAltitudeI;
    imu_t imu;
};

#endif // TINY10DOF_H
