/*
  Arduino code for transmit IMU informations using websocket.

  Author : Sully VITRY
  date: February 22, 2020
  license: GPL-3.0.

  This dev is base on the Kris Winer code for MPU-9250.
*/

#include <SPI.h>
#include <Wire.h>
#include <M5Stack.h>
//multiboot SD card
#include "M5StackUpdater.h"
//#include <WiFi.h>
//#include <WiFiClient.h>
//#include <WebServer.h>
//#include <ESPmDNS.h>
//#include <WiFiUdp.h>
//#include <ArduinoOTA.h>
//#include <WebSocketsServer.h>
#include <U8g2lib.h>
//#include <Adafruit_NeoPixel.h>
#include <FastLED.h>

//load SD card for M5STACK
#include "FS.h"
#include "SPIFFS.h"

//SD card function
#include <SPI.h>
#include <SD.h>
const int chipSelect = 4;

/*
  ________          _____.__
  \______ \   _____/ ____\__| ____   ____
   |    |  \_/ __ \   __\|  |/    \_/ __ \
   |    `   \  ___/|  |  |  |   |  \  ___/
  /_______  /\___  >__|  |__|___|  /\___  >
          \/     \/              \/     \/
*/
;  //this line solves everything with define pb arduino!
#define WAIT_CYCLE      10
#define MEAN_CYCLE      20
#define TIME_SCREEN_ON  3000
#define SCREEN_M5STACK  true          // ---> MODIFY depending DEV BOARD
#define SCREEN_M5STICK  false         // ---> MODIFY depending DEV BOARD
#define PERIOD_MEAS     200  //20ms
#define TEST_WIFI       20
#define UART_VIEW_DATA  false
#define SERIAL_DEBUG    false   // set to true to get Serial output for debugging
#define BtnPin 35       //M5STICK
#define BtnAPin 39      //M5STACK
#define BtnBPin 38      //M5STACK
#define BtnCPin 37      //M5STACK
#define LCDBACKLIGHT 32 //M5STACK  32  33:rst_lcd
#define DEVICE  1       //1_M5STACK_FIRE  2_M5STICK      // ---> MODIFY depending DEV BOARD
#define GYR_THRESHOLD 20
#define WAIT_DIAPORAMA 5000
#define NB_PICTURES   11

#define M5STACK_FIRE_NEO_NUM_LEDS 10
#define M5STACK_FIRE_NEO_DATA_PIN 15
#define NEOPIXEL_NUMBER 10

#define M5STICK_NUM_LEDS 10 //20-M5STICK   10-M5STACK       ---> MODIFY depending DEV BOARD
#define M5STICK_DATA_PIN 15 //25-M5STICK  15-M5STACK        ---> MODIFY depending DEV BOARD

#define DATA_PIN    15     //25-M5STICK  15-M5STACK         ---> MODIFY depending DEV BOARD
#define LED_TYPE    WS2811
#define COLOR_ORDER GRB
#define NUM_LEDS    10     //20-M5STICK   10-M5STACK        ---> MODIFY depending DEV BOARD
#define BRIGHTNESS         255  //10-M5STICK   255-M5STACK   ---> MODIFY depending DEV BOARD
#define FRAMES_PER_SECOND  120

// See also MPU-9250 Register Map and Descriptions, Revision 4.0, RM-MPU-9250A-00, Rev. 1.4, 9/9/2013 for registers not listed in
// above document; the MPU9250 and MPU9150 are virtually identical but the latter has a different register map
//
//Magnetometer Registers
#define AK8963_ADDRESS   0x0C
#define AK8963_WHO_AM_I  0x00 // should return 0x48
#define AK8963_INFO      0x01
#define AK8963_ST1       0x02  // data ready status bit 0
#define AK8963_XOUT_L     0x03  // data
#define AK8963_XOUT_H    0x04
#define AK8963_YOUT_L    0x05
#define AK8963_YOUT_H    0x06
#define AK8963_ZOUT_L    0x07
#define AK8963_ZOUT_H    0x08
#define AK8963_ST2       0x09  // Data overflow bit 3 and data read error status bit 2
#define AK8963_CNTL      0x0A  // Power down (0000), single-measurement (0001), self-test (1000) and Fuse ROM (1111) modes on bits 3:0
#define AK8963_ASTC      0x0C  // Self test control
#define AK8963_I2CDIS    0x0F  // I2C disable
#define AK8963_ASAX      0x10  // Fuse ROM x-axis sensitivity adjustment value
#define AK8963_ASAY      0x11  // Fuse ROM y-axis sensitivity adjustment value
#define AK8963_ASAZ      0x12  // Fuse ROM z-axis sensitivity adjustment value

#define SELF_TEST_X_GYRO 0x00
#define SELF_TEST_Y_GYRO 0x01
#define SELF_TEST_Z_GYRO 0x02

/*#define X_FINE_GAIN      0x03 // [7:0] fine gain
  #define Y_FINE_GAIN      0x04
  #define Z_FINE_GAIN      0x05
  #define XA_OFFSET_H      0x06 // User-defined trim values for accelerometer
  #define XA_OFFSET_L_TC   0x07
  #define YA_OFFSET_H      0x08
  #define YA_OFFSET_L_TC   0x09
  #define ZA_OFFSET_H      0x0A
  #define ZA_OFFSET_L_TC   0x0B */

#define SELF_TEST_X_ACCEL 0x0D
#define SELF_TEST_Y_ACCEL 0x0E
#define SELF_TEST_Z_ACCEL 0x0F

#define SELF_TEST_A      0x10

#define XG_OFFSET_H      0x13  // User-defined trim values for gyroscope
#define XG_OFFSET_L      0x14
#define YG_OFFSET_H      0x15
#define YG_OFFSET_L      0x16
#define ZG_OFFSET_H      0x17
#define ZG_OFFSET_L      0x18
#define SMPLRT_DIV       0x19
#define CONFIG           0x1A
#define GYRO_CONFIG      0x1B
#define ACCEL_CONFIG     0x1C
#define ACCEL_CONFIG2    0x1D
#define LP_ACCEL_ODR     0x1E
#define WOM_THR          0x1F

#define MOT_DUR          0x20  // Duration counter threshold for motion interrupt generation, 1 kHz rate, LSB = 1 ms
#define ZMOT_THR         0x21  // Zero-motion detection threshold bits [7:0]
#define ZRMOT_DUR        0x22  // Duration counter threshold for zero motion interrupt generation, 16 Hz rate, LSB = 64 ms

#define FIFO_EN          0x23
#define I2C_MST_CTRL     0x24
#define I2C_SLV0_ADDR    0x25
#define I2C_SLV0_REG     0x26
#define I2C_SLV0_CTRL    0x27
#define I2C_SLV1_ADDR    0x28
#define I2C_SLV1_REG     0x29
#define I2C_SLV1_CTRL    0x2A
#define I2C_SLV2_ADDR    0x2B
#define I2C_SLV2_REG     0x2C
#define I2C_SLV2_CTRL    0x2D
#define I2C_SLV3_ADDR    0x2E
#define I2C_SLV3_REG     0x2F
#define I2C_SLV3_CTRL    0x30
#define I2C_SLV4_ADDR    0x31
#define I2C_SLV4_REG     0x32
#define I2C_SLV4_DO      0x33
#define I2C_SLV4_CTRL    0x34
#define I2C_SLV4_DI      0x35
#define I2C_MST_STATUS   0x36
#define INT_PIN_CFG      0x37
#define INT_ENABLE       0x38
#define DMP_INT_STATUS   0x39  // Check DMP interrupt
#define INT_STATUS       0x3A
#define ACCEL_XOUT_H     0x3B
#define ACCEL_XOUT_L     0x3C
#define ACCEL_YOUT_H     0x3D
#define ACCEL_YOUT_L     0x3E
#define ACCEL_ZOUT_H     0x3F
#define ACCEL_ZOUT_L     0x40
#define TEMP_OUT_H       0x41
#define TEMP_OUT_L       0x42
#define GYRO_XOUT_H      0x43
#define GYRO_XOUT_L      0x44
#define GYRO_YOUT_H      0x45
#define GYRO_YOUT_L      0x46
#define GYRO_ZOUT_H      0x47
#define GYRO_ZOUT_L      0x48
#define EXT_SENS_DATA_00 0x49
#define EXT_SENS_DATA_01 0x4A
#define EXT_SENS_DATA_02 0x4B
#define EXT_SENS_DATA_03 0x4C
#define EXT_SENS_DATA_04 0x4D
#define EXT_SENS_DATA_05 0x4E
#define EXT_SENS_DATA_06 0x4F
#define EXT_SENS_DATA_07 0x50
#define EXT_SENS_DATA_08 0x51
#define EXT_SENS_DATA_09 0x52
#define EXT_SENS_DATA_10 0x53
#define EXT_SENS_DATA_11 0x54
#define EXT_SENS_DATA_12 0x55
#define EXT_SENS_DATA_13 0x56
#define EXT_SENS_DATA_14 0x57
#define EXT_SENS_DATA_15 0x58
#define EXT_SENS_DATA_16 0x59
#define EXT_SENS_DATA_17 0x5A
#define EXT_SENS_DATA_18 0x5B
#define EXT_SENS_DATA_19 0x5C
#define EXT_SENS_DATA_20 0x5D
#define EXT_SENS_DATA_21 0x5E
#define EXT_SENS_DATA_22 0x5F
#define EXT_SENS_DATA_23 0x60
#define MOT_DETECT_STATUS 0x61
#define I2C_SLV0_DO      0x63
#define I2C_SLV1_DO      0x64
#define I2C_SLV2_DO      0x65
#define I2C_SLV3_DO      0x66
#define I2C_MST_DELAY_CTRL 0x67
#define SIGNAL_PATH_RESET  0x68
#define MOT_DETECT_CTRL  0x69
#define USER_CTRL        0x6A  // Bit 7 enable DMP, bit 3 reset DMP
#define PWR_MGMT_1       0x6B // Device defaults to the SLEEP mode
#define PWR_MGMT_2       0x6C
#define DMP_BANK         0x6D  // Activates a specific bank in the DMP
#define DMP_RW_PNT       0x6E  // Set read/write pointer to a specific start address in specified DMP bank
#define DMP_REG          0x6F  // Register in DMP from which to read or to which to write
#define DMP_REG_1        0x70
#define DMP_REG_2        0x71
#define FIFO_COUNTH      0x72
#define FIFO_COUNTL      0x73
#define FIFO_R_W         0x74
#define WHO_AM_I_MPU9250 0x75 // Should return 0x71
#define XA_OFFSET_H      0x77
#define XA_OFFSET_L      0x78
#define YA_OFFSET_H      0x7A
#define YA_OFFSET_L      0x7B
#define ZA_OFFSET_H      0x7D
#define ZA_OFFSET_L      0x7E

// Using the MSENSR-9250 breakout board, ADO is set to 0
// Seven-bit device address is 110100 for ADO = 0 and 110101 for ADO = 1
#define ADO 0
#if ADO
#define MPU9250_ADDRESS 0x69  // Device address when ADO = 1
#else
#define MPU9250_ADDRESS 0x68  // Device address when ADO = 0
#define AK8963_ADDRESS 0x0C   //  Address of magnetometer
#endif

#define AHRS true         // set to false for basic data read

// Set initial input parameters
enum Ascale {
  AFS_2G = 0,
  AFS_4G,
  AFS_8G,
  AFS_16G
};

enum Gscale {
  GFS_250DPS = 0,
  GFS_500DPS,
  GFS_1000DPS,
  GFS_2000DPS
};

enum Mscale {
  MFS_14BITS = 0, // 0.6 mG per LSB
  MFS_16BITS      // 0.15 mG per LSB
};



/*
  ____   ____            .__      ___.   .__
  \   \ /   /____ _______|__|____ \_ |__ |  |   ____   ______
   \   Y   /\__  \\_  __ \  \__  \ | __ \|  | _/ __ \ /  ___/
    \     /  / __ \|  | \/  |/ __ \| \_\ \  |_\  ___/ \___ \
     \___/  (____  /__|  |__(____  /___  /____/\___  >____  >
                 \/              \/    \/          \/     \/
*/

//Screen on M5Stick U8G2_R0  U8G2_R1:90 U8G2_R2:180 U8G2_R3:270
U8G2_SH1107_64X128_F_4W_HW_SPI u8g2(U8G2_R1, 14, /* dc=*/ 27, /* reset=*/ 33);

//NEOPIXEL
//Adafruit_NeoPixel pixels = Adafruit_NeoPixel(M5STICK_NUM_LEDS, M5STICK_DATA_PIN, NEO_GRB + NEO_KHZ800);
CRGB leds[M5STICK_NUM_LEDS];

//websocket buffer used for sending datas
char buffer_ws[50];
char buffer_screen[50];

//save file
File dataFile;
char name_file[50];
char data_to_file[250];
bool bool_sdcard_error = true;

//ESP32 internal temperature
#ifdef __cplusplus
extern "C" {
#endif
uint8_t temprature_sens_read();
#ifdef __cplusplus
}
#endif

uint8_t temprature_sens_read();
int int_intTemp_meas = 0;
unsigned long ulong_time_int_Temp = 0;
int period_int_temp = 1000; //int temp mesure each 1s

//global variables
int int_cmpt_mean = 0;
float ax_comp = 0;
float ay_comp = 0;
float az_comp = 0;
float gx_comp = 0;
float gy_comp = 0;
float gz_comp = 0;
float mx_comp = 0;
float my_comp = 0;
float mz_comp = 0;
int int_acc_x = 0;
int int_acc_y = 0;
int int_acc_z = 0;
int int_gyr_x = 0;
int int_gyr_y = 0;
int int_gyr_z = 0;
int int_mag_x = 0;
int int_mag_y = 0;
int int_mag_z = 0;
int btnState = 0;

int int_i = 0;
int red = 0;
int green = 0;
int blue = 0;

int rot_img = 0;

/* smiley */
const uint8_t logo16x16_smiley[] = {
  B11100000,  B00000111,
  B00011000,  B00011000,
  B00000100,  B00100000,
  B00000010,  B01000000,
  B01100010,  B01000110,
  B01100001,  B10000110,
  B00000001,  B10000000,
  B00000001,  B10000000,
  B00000001,  B10000000,
  B00001001,  B10010000,
  B00010001,  B10001000,
  B11100010,  B01000111,
  B00000010,  B01000000,
  B00000100,  B00100000,
  B00011000,  B00011000,
  B11100000,  B00000111,
};

/* smiley bored */
const uint8_t logo16x16_smiley_bored[] = {
  B11100000,  B00000111,
  B00011000,  B00011000,
  B00000100,  B00100000,
  B00000010,  B01000000,
  B00000010,  B01000000,
  B01111001,  B10011110,
  B00000001,  B10000000,
  B00000001,  B10000000,
  B00000001,  B10000000,
  B00000001,  B10000000,
  B00000001,  B10000000,
  B11100010,  B01000111,
  B00000010,  B01000000,
  B00000100,  B00100000,
  B00011000,  B00011000,
  B11100000,  B00000111,
};

unsigned long ulong_time_old = 0;
unsigned long ulong_time_old_screen = 0;
unsigned long ulong_time_old_animate = 0;
unsigned long ulong_time_now = 0;
unsigned long ulong_time_cycle = 0;
long int delay_cycle = 0;

unsigned long ulong_time_min_tilt = 0;
unsigned long ulong_time_max_tilt = 0;

unsigned long ulong_time_min_acc = 0;
unsigned long ulong_time_max_acc = 0;

bool bool_active_screen = false;
bool bool_onetime_screen = false;
bool bool_motion_detected = false;
bool bool_onetime_static_mode = false;

//variable used for screen positionning
int x = 64 + 10;
int y = 128 + 20;
int z = 192 + 30;

//Variable uart
int incomingByteUart = 0;
int uart_mode = 0;

// Specify sensor full scale
uint8_t Gscale = GFS_250DPS;  //GFS_250DPS;
uint8_t Ascale = AFS_2G;
uint8_t Mscale = MFS_16BITS; // Choose either 14-bit or 16-bit magnetometer resolution
uint8_t Mmode = 0x06;  //0x02;        // 2 for 8 Hz, 6 for 100 Hz continuous magnetometer data read
float aRes, gRes, mRes;      // scale resolutions per LSB for the sensors

// Pin definitions
//int intPin = 12;  // These can be changed, 2 and 3 are the Arduinos ext int pins
//int myLed = 13; // Set up pin 13 led for toggling

#define A4 21
#define A5 22

int16_t accelCount[3];  // Stores the 16-bit signed accelerometer sensor output
int16_t gyroCount[3];   // Stores the 16-bit signed gyro sensor output
int16_t magCount[3];    // Stores the 16-bit signed magnetometer sensor output
float magCalibration[3] = {0, 0, 0}, magbias[3] = {0, 0, 0};  // Factory mag calibration and mag bias
float gyroBias[3] = {0, 0, 0}, accelBias[3] = {0, 0, 0};      // Bias corrections for gyro and accelerometer
int16_t tempCount;      // temperature raw count output
float   temperature;    // Stores the real internal chip temperature in degrees Celsius
float   SelfTest[6];    // holds results of gyro and accelerometer self test

// global constants for 9 DoF fusion and AHRS (Attitude and Heading Reference System)
float GyroMeasError = PI * (40.0f / 180.0f);   // gyroscope measurement error in rads/s (start at 40 deg/s)
float GyroMeasDrift = PI * (0.0f  / 180.0f);   // gyroscope measurement drift in rad/s/s (start at 0.0 deg/s/s)
// There is a tradeoff in the beta parameter between accuracy and response speed.
// In the original Madgwick study, beta of 0.041 (corresponding to GyroMeasError of 2.7 degrees/s) was found to give optimal accuracy.
// However, with this value, the LSM9SD0 response time is about 10 seconds to a stable initial quaternion.
// Subsequent changes also require a longish lag time to a stable output, not fast enough for a quadcopter or robot car!
// By increasing beta (GyroMeasError) by about a factor of fifteen, the response time constant is reduced to ~2 sec
// I haven't noticed any reduction in solution accuracy. This is essentially the I coefficient in a PID control sense;
// the bigger the feedback coefficient, the faster the solution converges, usually at the expense of accuracy.
// In any case, this is the free parameter in the Madgwick filtering and fusion scheme.
float beta = sqrt(3.0f / 4.0f) * GyroMeasError;   // compute beta
float zeta = sqrt(3.0f / 4.0f) * GyroMeasDrift;   // compute zeta, the other free parameter in the Madgwick scheme usually set to a small or zero value
#define Kp 2.0f * 5.0f // these are the free parameters in the Mahony filter and fusion scheme, Kp for proportional feedback, Ki for integral
#define Ki 0.0f

uint32_t delt_t = 0; // used to control display output rate
uint32_t count = 0, sumCount = 0; // used to control display output rate
float pitch, yaw, roll;
float cppitch, cpyaw, cproll;

float cppitch_min = 0, cppitch_max = 0;

float deltat = 0.0f, sum = 0.0f;        // integration interval for both filter schemes
uint32_t lastUpdate = 0, firstUpdate = 0; // used to calculate integration interval
uint32_t Now = 0;        // used to calculate integration interval

float ax, ay, az, gx, gy, gz, mx, my, mz; // variables to hold latest sensor data values
float quaternion_vect[4] = {1.0f, 0.0f, 0.0f, 0.0f};    // vector to hold quaternion
float eInt[3] = {0.0f, 0.0f, 0.0f};       // vector to hold integral error for Mahony method



//function declaration zone
void readMagData(int16_t * destination);


/*
  ________                __                      .__
  \_____  \  __ _______ _/  |_  ___________  ____ |__| ____   ____
   /  / \  \|  |  \__  \\   __\/ __ \_  __ \/    \|  |/  _ \ /    \
  /   \_/.  \  |  // __ \|  | \  ___/|  | \/   |  \  (  <_> )   |  \
  \_____\ \_/____/(____  /__|  \___  >__|  |___|  /__|\____/|___|  /
        \__>          \/          \/           \/               \/
*/
// Implementation of Sebastian Madgwick's "...efficient orientation filter for... inertial/magnetic sensor arrays"
// (see http://www.x-io.co.uk/category/open-source/ for examples and more details)
// which fuses acceleration, rotation rate, and magnetic moments to produce a quaternion-based estimate of absolute
// device orientation -- which can be converted to yaw, pitch, and roll. Useful for stabilizing quadcopters, etc.
// The performance of the orientation filter is at least as good as conventional Kalman-based filtering algorithms
// but is much less computationally intensive---it can be performed on a 3.3 V Pro Mini operating at 8 MHz!

float norm_acc = 0;

void MadgwickQuaternionUpdate(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz)
{
  float q1 = quaternion_vect[0], q2 = quaternion_vect[1], q3 = quaternion_vect[2], q4 = quaternion_vect[3];   // short name local variable for readability

  float hx, hy, _2bx, _2bz;
  float s1, s2, s3, s4;
  float qDot1, qDot2, qDot3, qDot4;
  float norm;
  // Auxiliary variables to avoid repeated arithmetic
  float _2q1mx;
  float _2q1my;
  float _2q1mz;
  float _2q2mx;
  float _4bx;
  float _4bz;
  float _2q1 = 2.0f * q1;
  float _2q2 = 2.0f * q2;
  float _2q3 = 2.0f * q3;
  float _2q4 = 2.0f * q4;
  float _2q1q3 = 2.0f * q1 * q3;
  float _2q3q4 = 2.0f * q3 * q4;
  float q1q1 = q1 * q1;
  float q1q2 = q1 * q2;
  float q1q3 = q1 * q3;
  float q1q4 = q1 * q4;
  float q2q2 = q2 * q2;
  float q2q3 = q2 * q3;
  float q2q4 = q2 * q4;
  float q3q3 = q3 * q3;
  float q3q4 = q3 * q4;
  float q4q4 = q4 * q4;

  // Normalise accelerometer measurement
  norm = sqrtf(ax * ax + ay * ay + az * az);
  norm_acc = norm;
  if (norm == 0.0f) return; // handle NaN
  norm = 1.0f / norm;
  ax *= norm;
  ay *= norm;
  az *= norm;

  // Normalise magnetometer measurement
  norm = sqrtf(mx * mx + my * my + mz * mz);
  if (norm == 0.0f) return; // handle NaN
  norm = 1.0f / norm;
  mx *= norm;
  my *= norm;
  mz *= norm;

  // Reference direction of Earth's magnetic field
  _2q1mx = 2.0f * q1 * mx;
  _2q1my = 2.0f * q1 * my;
  _2q1mz = 2.0f * q1 * mz;
  _2q2mx = 2.0f * q2 * mx;
  hx = mx * q1q1 - _2q1my * q4 + _2q1mz * q3 + mx * q2q2 + _2q2 * my * q3 + _2q2 * mz * q4 - mx * q3q3 - mx * q4q4;
  hy = _2q1mx * q4 + my * q1q1 - _2q1mz * q2 + _2q2mx * q3 - my * q2q2 + my * q3q3 + _2q3 * mz * q4 - my * q4q4;
  _2bx = sqrtf(hx * hx + hy * hy);
  _2bz = -_2q1mx * q3 + _2q1my * q2 + mz * q1q1 + _2q2mx * q4 - mz * q2q2 + _2q3 * my * q4 - mz * q3q3 + mz * q4q4;
  _4bx = 2.0f * _2bx;
  _4bz = 2.0f * _2bz;

  // Gradient decent algorithm corrective step
  s1 = -_2q3 * (2.0f * q2q4 - _2q1q3 - ax) + _2q2 * (2.0f * q1q2 + _2q3q4 - ay) - _2bz * q3 * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (-_2bx * q4 + _2bz * q2) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + _2bx * q3 * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
  s2 = _2q4 * (2.0f * q2q4 - _2q1q3 - ax) + _2q1 * (2.0f * q1q2 + _2q3q4 - ay) - 4.0f * q2 * (1.0f - 2.0f * q2q2 - 2.0f * q3q3 - az) + _2bz * q4 * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (_2bx * q3 + _2bz * q1) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + (_2bx * q4 - _4bz * q2) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
  s3 = -_2q1 * (2.0f * q2q4 - _2q1q3 - ax) + _2q4 * (2.0f * q1q2 + _2q3q4 - ay) - 4.0f * q3 * (1.0f - 2.0f * q2q2 - 2.0f * q3q3 - az) + (-_4bx * q3 - _2bz * q1) * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (_2bx * q2 + _2bz * q4) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + (_2bx * q1 - _4bz * q3) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
  s4 = _2q2 * (2.0f * q2q4 - _2q1q3 - ax) + _2q3 * (2.0f * q1q2 + _2q3q4 - ay) + (-_4bx * q4 + _2bz * q2) * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (-_2bx * q1 + _2bz * q3) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + _2bx * q2 * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
  norm = sqrtf(s1 * s1 + s2 * s2 + s3 * s3 + s4 * s4);    // normalise step magnitude
  norm = 1.0f / norm;
  s1 *= norm;
  s2 *= norm;
  s3 *= norm;
  s4 *= norm;

  // Compute rate of change of quaternion
  qDot1 = 0.5f * (-q2 * gx - q3 * gy - q4 * gz) - beta * s1;
  qDot2 = 0.5f * (q1 * gx + q3 * gz - q4 * gy) - beta * s2;
  qDot3 = 0.5f * (q1 * gy - q2 * gz + q4 * gx) - beta * s3;
  qDot4 = 0.5f * (q1 * gz + q2 * gy - q3 * gx) - beta * s4;

  // Integrate to yield quaternion
  q1 += qDot1 * deltat;
  q2 += qDot2 * deltat;
  q3 += qDot3 * deltat;
  q4 += qDot4 * deltat;
  norm = sqrtf(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);    // normalise quaternion
  norm = 1.0f / norm;
  quaternion_vect[0] = q1 * norm;
  quaternion_vect[1] = q2 * norm;
  quaternion_vect[2] = q3 * norm;
  quaternion_vect[3] = q4 * norm;

}



// Similar to Madgwick scheme but uses proportional and integral filtering on the error between estimated reference vectors and
// measured ones.
void MahonyQuaternionUpdate(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz)
{
  float q1 = quaternion_vect[0], q2 = quaternion_vect[1], q3 = quaternion_vect[2], q4 = quaternion_vect[3];   // short name local variable for readability
  float norm;
  float hx, hy, bx, bz;
  float vx, vy, vz, wx, wy, wz;
  float ex, ey, ez;
  float pa, pb, pc;

  // Auxiliary variables to avoid repeated arithmetic
  float q1q1 = q1 * q1;
  float q1q2 = q1 * q2;
  float q1q3 = q1 * q3;
  float q1q4 = q1 * q4;
  float q2q2 = q2 * q2;
  float q2q3 = q2 * q3;
  float q2q4 = q2 * q4;
  float q3q3 = q3 * q3;
  float q3q4 = q3 * q4;
  float q4q4 = q4 * q4;

  // Normalise accelerometer measurement
  norm = sqrtf(ax * ax + ay * ay + az * az);
  norm_acc = norm;
  if (norm == 0.0f) return; // handle NaN
  norm = 1.0f / norm;        // use reciprocal for division
  ax *= norm;
  ay *= norm;
  az *= norm;

  // Normalise magnetometer measurement
  norm = sqrtf(mx * mx + my * my + mz * mz);
  if (norm == 0.0f) return; // handle NaN
  norm = 1.0f / norm;        // use reciprocal for division
  mx *= norm;
  my *= norm;
  mz *= norm;

  // Reference direction of Earth's magnetic field
  hx = 2.0f * mx * (0.5f - q3q3 - q4q4) + 2.0f * my * (q2q3 - q1q4) + 2.0f * mz * (q2q4 + q1q3);
  hy = 2.0f * mx * (q2q3 + q1q4) + 2.0f * my * (0.5f - q2q2 - q4q4) + 2.0f * mz * (q3q4 - q1q2);
  bx = sqrtf((hx * hx) + (hy * hy));
  bz = 2.0f * mx * (q2q4 - q1q3) + 2.0f * my * (q3q4 + q1q2) + 2.0f * mz * (0.5f - q2q2 - q3q3);

  // Estimated direction of gravity and magnetic field
  vx = 2.0f * (q2q4 - q1q3);
  vy = 2.0f * (q1q2 + q3q4);
  vz = q1q1 - q2q2 - q3q3 + q4q4;
  wx = 2.0f * bx * (0.5f - q3q3 - q4q4) + 2.0f * bz * (q2q4 - q1q3);
  wy = 2.0f * bx * (q2q3 - q1q4) + 2.0f * bz * (q1q2 + q3q4);
  wz = 2.0f * bx * (q1q3 + q2q4) + 2.0f * bz * (0.5f - q2q2 - q3q3);

  // Error is cross product between estimated direction and measured direction of gravity
  ex = (ay * vz - az * vy) + (my * wz - mz * wy);
  ey = (az * vx - ax * vz) + (mz * wx - mx * wz);
  ez = (ax * vy - ay * vx) + (mx * wy - my * wx);
  if (Ki > 0.0f)
  {
    eInt[0] += ex;      // accumulate integral error
    eInt[1] += ey;
    eInt[2] += ez;
  }
  else
  {
    eInt[0] = 0.0f;     // prevent integral wind up
    eInt[1] = 0.0f;
    eInt[2] = 0.0f;
  }

  // Apply feedback terms
  gx = gx + Kp * ex + Ki * eInt[0];
  gy = gy + Kp * ey + Ki * eInt[1];
  gz = gz + Kp * ez + Ki * eInt[2];

  // Integrate rate of change of quaternion
  pa = q2;
  pb = q3;
  pc = q4;
  q1 = q1 + (-q2 * gx - q3 * gy - q4 * gz) * (0.5f * deltat);
  q2 = pa + (q1 * gx + pb * gz - pc * gy) * (0.5f * deltat);
  q3 = pb + (q1 * gy - pa * gz + pc * gx) * (0.5f * deltat);
  q4 = pc + (q1 * gz + pa * gy - pb * gx) * (0.5f * deltat);

  // Normalise quaternion
  norm = sqrtf(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);
  norm = 1.0f / norm;
  quaternion_vect[0] = q1 * norm;
  quaternion_vect[1] = q2 * norm;
  quaternion_vect[2] = q3 * norm;
  quaternion_vect[3] = q4 * norm;

}



/*
  .___        __                                    __
  |   | _____/  |_  __________________ __ _________/  |_
  |   |/    \   __\/ __ \_  __ \_  __ \  |  \____ \   __\
  |   |   |  \  | \  ___/|  | \/|  | \/  |  /  |_> >  |
  |___|___|  /__|  \___  >__|   |__|  |____/|   __/|__|
           \/          \/                   |__|
*/

unsigned long ulong_time_bt1 = 0;

int m5stick_no_led = 0;

//interrupt function for M5Stack (BtnA) and M5Stick board
void pressed()
{
  //sleep mode
  btnState++;
  if (btnState == 1)  ulong_time_bt1 = millis() + 2000;
}


bool flip_btnB_bool = false;
bool flip_btnB_bool_info = false;

//interrupt function for M5Stack board
void pressed_Bbutton()
{
  flip_btnB_bool_info = true;

  if (flip_btnB_bool == false)
  {
    flip_btnB_bool = true;
    FastLED.setBrightness(0);
  }
  else
  {
    flip_btnB_bool = false;
    FastLED.setBrightness(BRIGHTNESS);
  }
}

bool flip_btnC_bool = false;
int int_num_datalog = 0;
bool bool_push_btnc_detect = false;
int btnC_count = 0;
unsigned long ulong_time_btC = 0;

bool bool_block_screen_anim = false;

//interrupt function for M5Stack board
void pressed_Cbutton()
{
  btnC_count++;

  if (btnC_count == 1)
  {
    //flip buttonC value
    flip_btnC_bool ^= true;
    bool_push_btnc_detect = true;
    bool_block_screen_anim = true;
    //fonction double bouton C nombre count en deux secondes
    ulong_time_btC = millis() + 2000;
  }

}



/*
                __
   ______ _____/  |_ __ ________
  /  ___// __ \   __\  |  \____ \
  \___ \\  ___/|  | |  |  /  |_> >
  /____  >\___  >__| |____/|   __/
      \/     \/           |__|

*/

bool bool_spiffs_error = false;

int int_k = 0;

void setup()
{
  M5.begin(true, false, false, false);
  Wire.begin();

  //multiboot SD card for M5STACK FIRE
  if (DEVICE == 1)   //1_M5STACK_FIRE 2_M5STICK
  {
    pinMode(LCDBACKLIGHT, OUTPUT);
    digitalWrite(LCDBACKLIGHT, HIGH); // power on lcd bcklight

    if (digitalRead(BUTTON_A_PIN) == 0) {
      Serial.println("Will Load menu binary");
      updateFromFS(SD);
      ESP.restart();
    }
  }

  //neopixel m5stack fire
  //pixels.begin();

  //fastled init
  FastLED.addLeds<LED_TYPE, DATA_PIN, COLOR_ORDER>(leds, NUM_LEDS).setCorrection(TypicalLEDStrip);

  // set master brightness control
  FastLED.setBrightness(BRIGHTNESS);

  //erase ALL LEDS
  for (int_i = 0; int_i < M5STICK_NUM_LEDS; int_i++)
  {
    leds[int_i] = CRGB::Black;
  }
  FastLED.show();
  delay(5);

  //  TWBR = 12;  // 400 kbit/sec I2C speed
  Serial.begin(115200);

  //information
  Serial.println("Start SETUP...");

  //FLASH LED ON START on M5STACK FIRE
  if (DEVICE == 1)   //1_M5STACK_FIRE 2_M5STICK
  {
    for (int_k = 0; int_k < 3; int_k++)
    {
      //flash on start
      for (int_i = 0; int_i < M5STICK_NUM_LEDS; int_i++)
      {
        //pixels.setPixelColor(int_i, pixels.Color(0, 0 , 0));
        leds[int_i] = CRGB::White;
      }
      FastLED.show();
      delay(20);

      //flash on start
      for (int_i = 0; int_i < M5STICK_NUM_LEDS; int_i++)
      {
        //pixels.setPixelColor(int_i, pixels.Color(0, 0 , 0));
        leds[int_i] = CRGB::Black;
      }
      FastLED.show();
      delay(200);
    }

  }

  if (DEVICE == 1)   //1_M5STACK_FIRE 2_M5STICK
  {

    //test memoire interne
    //---------------------
    if (!SPIFFS.begin(true)) {
      Serial.println("SPIFFS Mount Failed");
      bool_spiffs_error = true;
      return;
    }
    else
    {
      Serial.println("SPIFFS Mount Success");
      bool_spiffs_error = false;
    }

    //test SD card
    //---------------------
    bool_sdcard_error = true;

    Serial.print("Initializing SD card...");

    // see if the card is present and can be initialized:
    if (!SD.begin(chipSelect))
    {
      Serial.println("Card failed, or not present");
      // don't do anything more:
      bool_sdcard_error = true;
    }
    else
    {
      Serial.println("card initialized.");
      bool_sdcard_error = false;
    }

    //IMAGES
    M5.Lcd.drawJpgFile(SPIFFS, "/sully3.jpg", 0, 0);

    delay(2000);

    //infos touches 10secondes
    M5.Lcd.drawJpgFile(SPIFFS, "/infos2.jpg", 0, 0);
    delay(10000);

    M5.Lcd.drawJpgFile(SPIFFS, "/status2.jpg", 0, 0);


    if (SCREEN_M5STACK && (DEVICE == 1))
    {
      //M5.Lcd.fillScreen(BLACK);
      M5.Lcd.setTextColor(WHITE , BLACK);
      M5.Lcd.setTextSize(2);

      M5.Lcd.setCursor(27, 156); M5.Lcd.print("                       ");
      M5.Lcd.setCursor(27, 172); M5.Lcd.print("                       ");
      M5.Lcd.setCursor(27, 188); M5.Lcd.print("                       ");
      M5.Lcd.setCursor(27, 204); M5.Lcd.print("                       ");

      M5.Lcd.setCursor(27, 156); M5.Lcd.print("                  ");
      M5.Lcd.setCursor(27, 172); M5.Lcd.print("   PROGRAM        ");
      M5.Lcd.setCursor(27, 188); M5.Lcd.print("   START          ");
      M5.Lcd.setCursor(27, 204); M5.Lcd.print("                  ");
      delay(1000);

      //Test memoire interne
      if (bool_spiffs_error == true)
      {
        M5.Lcd.setCursor(27, 156); M5.Lcd.print("                  ");
        M5.Lcd.setCursor(27, 172); M5.Lcd.print("   SPIFFS         ");
        M5.Lcd.setCursor(27, 188); M5.Lcd.print("   read !ERROR    ");
        M5.Lcd.setCursor(27, 204); M5.Lcd.print("                  ");
        delay(1000);
      } //if error SD card
      else
      {
        M5.Lcd.setCursor(27, 156); M5.Lcd.print("                  ");
        M5.Lcd.setCursor(27, 172); M5.Lcd.print("   SPIFFS OK      ");
        M5.Lcd.setCursor(27, 188); M5.Lcd.print("                  ");
        M5.Lcd.setCursor(27, 204); M5.Lcd.print("                  ");
        delay(1000);
      } //else if error SD card

      //Test SD card externe
      if (bool_sdcard_error == true)
      {
        M5.Lcd.setCursor(27, 156); M5.Lcd.print("                  ");
        M5.Lcd.setCursor(27, 172); M5.Lcd.print("   SD Card        ");
        M5.Lcd.setCursor(27, 188); M5.Lcd.print("   Not present!!  ");
        M5.Lcd.setCursor(27, 204); M5.Lcd.print("                  ");
        delay(1000);
      } //if error SD card
      else
      {
        M5.Lcd.setCursor(27, 156); M5.Lcd.print("                  ");
        M5.Lcd.setCursor(27, 172); M5.Lcd.print("   SD card OK     ");
        M5.Lcd.setCursor(27, 188); M5.Lcd.print("                  ");
        M5.Lcd.setCursor(27, 204); M5.Lcd.print("                  ");
        delay(1000);
      } //else if error SD card

    } //if screen and M5Stack

  } //if M5 STACK

  Wire.begin();

  if (DEVICE == 1)   //1_M5STACK_FIRE 2_M5STICK
  {
    //information
    Serial.println("Config BtnAPin M5Stack");
    pinMode(BtnAPin, INPUT_PULLUP);
    attachInterrupt(BtnAPin, pressed, FALLING ); /*Check if the button is pushed using interrupt*/

    pinMode(BtnBPin, INPUT_PULLUP);
    attachInterrupt(BtnBPin, pressed_Bbutton, FALLING ); /*Check if the button is pushed using interrupt*/

    pinMode(BtnCPin, INPUT_PULLUP);
    attachInterrupt(BtnCPin, pressed_Cbutton, FALLING ); /*Check if the button is pushed using interrupt*/

    btnState = 0;
  }  //if DEVICE == 1

  if ((DEVICE == 1)) //1_M5STACK_FIRE 2_M5STICK
  {
    if ((SCREEN_M5STACK == true))
    {
      pinMode(LCDBACKLIGHT, OUTPUT);
      digitalWrite(LCDBACKLIGHT, HIGH); // power on lcd bcklight
      //information
      Serial.println("Power on backlight LCD\n");
    }
    else
    {
      pinMode(LCDBACKLIGHT, OUTPUT);
      digitalWrite(LCDBACKLIGHT, LOW); // power off lcd bcklight
      //information
      Serial.println("Power off backlight LCD\n");
    }  //if SCREEN_M5STACK
  } //end if M5STACK

  delay(1000);
  delay(1000);

  //information
  Serial.println("Starting Program\n");

  btnState = 0;

  // Read the WHO_AM_I register, this is a good test of communication
  byte c = readByte(MPU9250_ADDRESS, WHO_AM_I_MPU9250);  // Read WHO_AM_I register for MPU-9250
  Serial.print("MPU9250 "); Serial.print("I AM "); Serial.print(c, HEX); Serial.print(" I should be "); Serial.println(0x71, HEX);
  //display.setCursor(20,0); //display.print("MPU9250");
  //display.setCursor(0,10); //display.print("I AM");
  //display.setCursor(0,20); //display.print(c, HEX);
  //display.setCursor(0,30); //display.print("I Should Be");
  //display.setCursor(0,40); //display.print(0x71, HEX);
  //display.display();
  //delay(1000);

  if (c == 0x71) // WHO_AM_I should always be 0x68
  {
    Serial.println("MPU9250 is online...");

    if (SCREEN_M5STACK && (DEVICE == 1))
    {
      //erase motion message
      M5.Lcd.setTextColor(GREEN , BLACK);
      M5.Lcd.setTextSize(2);

      M5.Lcd.setCursor(27, 156); M5.Lcd.print("                       ");
      M5.Lcd.setCursor(27, 172); M5.Lcd.print("                       ");
      M5.Lcd.setCursor(27, 188); M5.Lcd.print("                       ");
      M5.Lcd.setCursor(27, 204); M5.Lcd.print("                       ");

      M5.Lcd.setCursor(27, 156); M5.Lcd.print("                  ");
      M5.Lcd.setCursor(27, 172); M5.Lcd.print("   MPU9250 OK     ");
      M5.Lcd.setCursor(27, 188); M5.Lcd.print("                  ");
      M5.Lcd.setCursor(27, 204); M5.Lcd.print("                  ");

      //Serial.println("-- MPU9250 OK --");
      delay(1000);
    }

    MPU9250SelfTest(SelfTest); // Start by performing self test and reporting values
    Serial.print("x-axis self test: acceleration trim within : "); Serial.print(SelfTest[0], 1); Serial.println("% of factory value");
    Serial.print("y-axis self test: acceleration trim within : "); Serial.print(SelfTest[1], 1); Serial.println("% of factory value");
    Serial.print("z-axis self test: acceleration trim within : "); Serial.print(SelfTest[2], 1); Serial.println("% of factory value");
    Serial.print("x-axis self test: gyration trim within : "); Serial.print(SelfTest[3], 1); Serial.println("% of factory value");
    Serial.print("y-axis self test: gyration trim within : "); Serial.print(SelfTest[4], 1); Serial.println("% of factory value");
    Serial.print("z-axis self test: gyration trim within : "); Serial.print(SelfTest[5], 1); Serial.println("% of factory value");

    //calibrateMPU9250(gyroBias, accelBias); // Calibrate gyro and accelerometers, load biases in bias registers

    //calibrate MAG MPU9250


    //display.display();
    delay(10);

    if (SCREEN_M5STACK && (DEVICE == 1))
    {
      //erase motion message
      M5.Lcd.setTextColor(GREEN , BLACK);
      M5.Lcd.setTextSize(2);

      M5.Lcd.setCursor(27, 156); M5.Lcd.print("                       ");
      M5.Lcd.setCursor(27, 172); M5.Lcd.print("                       ");
      M5.Lcd.setCursor(27, 188); M5.Lcd.print("                       ");
      M5.Lcd.setCursor(27, 204); M5.Lcd.print("                       ");

      M5.Lcd.setCursor(27, 156); M5.Lcd.print("                  ");
      M5.Lcd.setCursor(27, 172); M5.Lcd.print("  MPU9250         ");
      M5.Lcd.setCursor(27, 188); M5.Lcd.print("  CALIB. OK       ");
      M5.Lcd.setCursor(27, 204); M5.Lcd.print("                  ");

      //Serial.println("-- MPU9250 OK --");
      delay(1000);

      M5.Lcd.setCursor(10, 40); M5.Lcd.print("                    ");
      M5.Lcd.setCursor(10, 65); M5.Lcd.print("                    ");
      M5.Lcd.setCursor(10, 90); M5.Lcd.print("                    ");
      M5.Lcd.setCursor(10, 115); M5.Lcd.print("                    ");

      M5.Lcd.fillScreen(0x0000);
      delay(100);
    }

    initMPU9250();
    Serial.println("MPU9250 initialized for active data mode...."); // Initialize device for active mode read of acclerometer, gyroscope, and temperature

    // Read the WHO_AM_I register of the magnetometer, this is a good test of communication
    byte d = readByte(AK8963_ADDRESS, AK8963_WHO_AM_I);  // Read WHO_AM_I register for AK8963
    Serial.print("AK8963 "); Serial.print("I AM "); Serial.print(d, HEX); Serial.print(" I should be "); Serial.println(0x48, HEX);
    delay(10);

    // Get magnetometer calibration from AK8963 ROM
    initAK8963(magCalibration); Serial.println("AK8963 initialized for active data mode...."); // Initialize device for active mode read of magnetometer

    //    if (SERIAL_DEBUG || (uart_mode == 1)) {
    //  Serial.println("Calibration values: ");
    Serial.print("Mag_X-Axis sensitivity adjustment value "); Serial.println(magCalibration[0], 2);
    Serial.print("Mag_Y-Axis sensitivity adjustment value "); Serial.println(magCalibration[1], 2);
    Serial.print("Mag_Z-Axis sensitivity adjustment value "); Serial.println(magCalibration[2], 2);
    //    }

    delay(10);
  }
  else
  {
    Serial.print("Could not connect to MPU9250: 0x");
    Serial.println(c, HEX);
    while (1) ; // Loop forever if communication doesn't happen
  }

  //screen static mode for IP information
  bool_onetime_static_mode = true;


  //icone no save on startup
  //M5.Lcd.drawJpgFile(SPIFFS, "/file_ok.jpg", 290, 200);
  M5.Lcd.drawJpgFile(SPIFFS, "/file_no.jpg", 290, 200);

  //icone led on by default
  M5.Lcd.drawJpgFile(SPIFFS, "/light_on.jpg", 265, 200);

}



/*
     _____         .__         .____
    /     \ _____  |__| ____   |    |    ____   ____ ______
   /  \ /  \\__  \ |  |/    \  |    |   /  _ \ /  _ \\____ \
  /    Y    \/ __ \|  |   |  \ |    |__(  <_> |  <_> )  |_> >
  \____|__  (____  /__|___|  / |_______ \____/ \____/|   __/
          \/     \/        \/          \/            |__|
*/
bool flip_neopixel_detect = false;
float max_acc = 0;
float ay_old = 0;

int int_first = 1;

int li_x1 = 0;
int li_y1 = 0;
int li_x2 = 0;
int li_y2 = 0;

int li_x1_old = 0;
int li_y1_old = 0;
int li_x2_old = 0;
int li_y2_old = 0;

int one_shot_min = 0;
int li_x1_min = 0;
int li_y1_min = 0;
int li_x2_min = 0;
int li_y2_min = 0;

int one_shot_max = 0;
int li_x1_max = 0;
int li_y1_max = 0;
int li_x2_max = 0;
int li_y2_max = 0;

bool bool_file_create_ok = false;

void loop()
{

  //period with millis()
  ulong_time_now = millis();

  //delay cycle calculation
  delay_cycle = ulong_time_now - ulong_time_cycle;
  ulong_time_cycle = millis();

  //listen UART for special mode
  if (Serial.available() > 0)
  {
    //uart information
    Serial.print("Uart received: ");
    Serial.print("Char nb:");
    Serial.print(Serial.available());
    Serial.print(" - ");

    // read the incoming byte:
    incomingByteUart = Serial.read();

    //uart information
    Serial.println(incomingByteUart, DEC);
    Serial.println(incomingByteUart);
    if (incomingByteUart == 0x31) uart_mode = 1;
    if (incomingByteUart == 0x30) uart_mode = 0;
    if (incomingByteUart == 0x32) uart_mode = 2;

  }



  // Temperature internal sensor
  if (ulong_time_now >= ulong_time_int_Temp)
  {
    //ulong_time_int_Temp = millis() + period_int_temp;
    //int_intTemp_meas = ((temprature_sens_read() - 32) / 1.8);
  }

  //screen auto black
  if (ulong_time_now >= ulong_time_old_screen)
  {
    //turn off screen M5STACK
    if (SCREEN_M5STACK && bool_onetime_static_mode && (DEVICE == 1) && !(bool_motion_detected) )
    {
      /*
        bool_onetime_static_mode = false;
        //erase motion message
        M5.Lcd.setTextColor(GREEN , BLACK);
        M5.Lcd.setTextSize(2);

        M5.Lcd.setCursor(27, 156); M5.Lcd.print("                       ");
        M5.Lcd.setCursor(27, 172); M5.Lcd.print("    Static mode       ");
        M5.Lcd.setCursor(27, 188); M5.Lcd.print("                       ");
        // M5.Lcd.setCursor(27, 188); M5.Lcd.print(WiFi.localIP());
        M5.Lcd.setCursor(27, 204); M5.Lcd.print("                       ");
      */
      //Serial.println("-- No motion --");
    }

    bool_active_screen = false;
  }

  //enter low power with button
  if (DEVICE == 1)  //1_M5STACK_FIRE 2_M5STICK
  {

    //boutonB pour la gestion des LED NEOPIXEL
    if (flip_btnB_bool_info == true)
    {
      flip_btnB_bool_info = false;

      if (flip_btnB_bool == false)
      {
        //infos screen
        M5.Lcd.drawJpgFile(SPIFFS, "/led_on.jpg", 0, 0);
        delay(2000);
        M5.Lcd.fillScreen(0x0000);
        M5.Lcd.drawJpgFile(SPIFFS, "/light_on.jpg", 265, 200);
      }
      else
      {
        //infos screen
        M5.Lcd.drawJpgFile(SPIFFS, "/led_off.jpg", 0, 0);
        delay(2000);
        M5.Lcd.fillScreen(0x0000);
        M5.Lcd.drawJpgFile(SPIFFS, "/light_off.jpg", 265, 200);
      }

      //gestion icone file record or not
      //numero fichier
      M5.Lcd.setTextColor(WHITE);
      M5.Lcd.setTextSize(1);
      M5.Lcd.setCursor(295, 190);
      M5.Lcd.print(int_num_datalog);

      if (flip_btnC_bool == true )
      {
        M5.Lcd.drawJpgFile(SPIFFS, "/file_ok.jpg", 290, 200);
      }
      else
      {
        M5.Lcd.drawJpgFile(SPIFFS, "/file_no.jpg", 290, 200);
      }

    }

    //Gestion du Bouton A pour le mode low power
    if (btnState >= 1)
    {
      //infos screen
      M5.Lcd.drawJpgFile(SPIFFS, "/sleep_mode.jpg", 0, 0);
      delay(2000);
      M5.Lcd.fillScreen(0x0000);
      delay(10);

      //information
      Serial.println("M5STACK FIRE Enter in Low Power\n");

      //flash on start OFF
      for (int_i = 0; int_i < M5STICK_NUM_LEDS; int_i++)
      {
        //pixels.setPixelColor(int_i, pixels.Color(0, 0 , 0));
        leds[int_i] = CRGB::Black;
      }
      FastLED.show();
      delay(200);

      esp_deep_sleep_start();
    }
  }

  // If intPin goes high, all data registers have new data
  if (readByte(MPU9250_ADDRESS, INT_STATUS) & 0x01) {  // On interrupt, check if data ready interrupt
    readAccelData(accelCount);  // Read the x/y/z adc values
    getAres();

    // Now we'll calculate the accleration value into actual g's
    ax = (float)accelCount[0] * aRes; // - accelBias[0];  // get actual g value, this depends on scale being set
    ay = (float)accelCount[1] * aRes; // - accelBias[1];
    az = (float)accelCount[2] * aRes; // - accelBias[2];

    readGyroData(gyroCount);  // Read the x/y/z adc values
    getGres();

    // Calculate the gyro value into actual degrees per second
    gx = (float)gyroCount[0] * gRes; // get actual gyro value, this depends on scale being set
    gy = (float)gyroCount[1] * gRes;
    gz = (float)gyroCount[2] * gRes;

    readMagData(magCount);  // Read the x/y/z adc values
    getMres();
    //    magbias[0] = +470.;  // User environmental x-axis correction in milliGauss, should be automatically calculated
    //    magbias[1] = +120.;  // User environmental x-axis correction in milliGauss
    //    magbias[2] = +125.;  // User environmental x-axis correction in milliGauss
    magbias[0] = -8890.759555696817;  // User environmental x-axis correction in milliGauss, should be automatically calculated
    magbias[1] = 15168.213655967384;  // User environmental x-axis correction in milliGauss
    magbias[2] = -2725.084387493947;  // User environmental x-axis correction in milliGauss

    /*
       Mean MagX : -9483.550048517109  milliGauss
       Mean MagY : 15129.753912626955  milliGauss
       Mean MagZ : -2929.590018642322  milliGauss

      Mean MagX : -8890.759555696817
      Mean MagY : 15168.213655967384
      Mean MagZ : -2725.084387493947

    */

    // Calculate the magnetometer values in milliGauss
    // Include factory calibration per data sheet and user environmental corrections
    mx = (float)magCount[0] * mRes * magCalibration[0] - magbias[0]; // get actual magnetometer value, this depends on scale being set
    my = (float)magCount[1] * mRes * magCalibration[1] - magbias[1];
    mz = (float)magCount[2] * mRes * magCalibration[2] - magbias[2];
  }

  /*
    Now = micros();
    deltat = ((Now - lastUpdate) / 1000000.0f); // set integration time by time elapsed since last filter update
    lastUpdate = Now;

    sum += deltat; // sum for averaging filter update rate
    sumCount++;

    // Sensors x (y)-axis of the accelerometer is aligned with the y (x)-axis of the magnetometer;
    // the magnetometer z-axis (+ down) is opposite to z-axis (+ up) of accelerometer and gyro!
    // We have to make some allowance for this orientationmismatch in feeding the output to the quaternion filter.
    // For the MPU-9250, we have chosen a magnetic rotation that keeps the sensor forward along the x-axis just like
    // in the LSM9DS0 sensor. This rotation can be modified to allow any convenient orientation convention.
    // This is ok by aircraft orientation standards!
    // Pass gyro rate as rad/s
    // MadgwickQuaternionUpdate(ax, ay, az, gx*PI/180.0f, gy*PI/180.0f, gz*PI/180.0f,  my,  mx, mz);
    MahonyQuaternionUpdate(ax, ay, az, gx * PI / 180.0f, gy * PI / 180.0f, gz * PI / 180.0f, my, mx, mz);

    //SPEED UP MEASURE
    cppitch = pitch;
    cpyaw = yaw;
    cproll = roll;

    yaw   = atan2(2.0f * (quaternion_vect[1] * quaternion_vect[2] + quaternion_vect[0] * quaternion_vect[3]), quaternion_vect[0] * quaternion_vect[0] + quaternion_vect[1] * quaternion_vect[1] - quaternion_vect[2] * quaternion_vect[2] - quaternion_vect[3] * quaternion_vect[3]);
    pitch = -asin(2.0f * (quaternion_vect[1] * quaternion_vect[3] - quaternion_vect[0] * quaternion_vect[2]));
    roll  = atan2(2.0f * (quaternion_vect[0] * quaternion_vect[1] + quaternion_vect[2] * quaternion_vect[3]), quaternion_vect[0] * quaternion_vect[0] - quaternion_vect[1] * quaternion_vect[1] - quaternion_vect[2] * quaternion_vect[2] + quaternion_vect[3] * quaternion_vect[3]);
    pitch *= 180.0f / PI;
    yaw   *= 180.0f / PI;
    yaw   += 2.24; // Declination at ADTP location France ANNECY

  */
  /*
      ADTP location France ANNECY
      Model Used:   WMM-2020  Help
      Latitude:   45.895993° N
      Longitude:  6.099600° E
      Date        Declination
      2020-07-23  2.24° E  ± 0.36°  changing by  0.16° E per year
  */
  //roll  *= 180.0f / PI;


  /*
       _____   .________ _____________________________
      /     \  |   ____/ \______   \_   _____/\______ \
     /  \ /  \ |____  \   |       _/|    __)_  |    |  \
    /    Y    \/       \  |    |   \|        \ |    `   \
    \____|__  /______  /  |____|_  /_______  //_______  /
            \/       \/          \/        \/         \/
  */
  //M5STACK ROUGE

  if (SCREEN_M5STACK  && (DEVICE == 1))
  {

    //gestion boutton C 3 pour la sauvegarde SD
    if (flip_btnC_bool == true )
    {

      /*
            if (false)   //ancien code recalibration
            {
              M5.Lcd.drawJpgFile(SPIFFS, "/infos2.jpg", 0, 0);
              delay(2000);

              M5.Lcd.drawJpgFile(SPIFFS, "/status2.jpg", 0, 0);

              M5.Lcd.setTextColor(GREEN , BLACK);
              M5.Lcd.setTextSize(2);

              M5.Lcd.setCursor(27, 156); M5.Lcd.print("                       ");
              M5.Lcd.setCursor(27, 172); M5.Lcd.print("                       ");
              M5.Lcd.setCursor(27, 188); M5.Lcd.print("                       ");
              M5.Lcd.setCursor(27, 204); M5.Lcd.print("                       ");

              M5.Lcd.setCursor(27, 156); M5.Lcd.print("   Calibrate      ");
              M5.Lcd.setCursor(27, 172); M5.Lcd.print("   start          ");
              M5.Lcd.setCursor(27, 188); M5.Lcd.print("   not move       ");
              M5.Lcd.setCursor(27, 204); M5.Lcd.print("                  ");

              delay(1000);

              //calibration sensor
              readByte(AK8963_ADDRESS, AK8963_WHO_AM_I);  // Read WHO_AM_I register for AK8963
              delay(100);
              MPU9250SelfTest(SelfTest); // Start by performing self test and reporting values
              delay(100);
              //calibrateMPU9250(gyroBias, accelBias); // Calibrate gyro and accelerometers, load biases in bias registers
              delay(100);
              initMPU9250();
              delay(100);
              readByte(AK8963_ADDRESS, AK8963_WHO_AM_I);  // Read WHO_AM_I register for AK8963
              delay(100);
              // Get magnetometer calibration from AK8963 ROM
              initAK8963(magCalibration);
              Serial.println("AK8963 initialized for active data mode...."); // Initialize device for active mode read of magnetometer


              M5.Lcd.setCursor(27, 156); M5.Lcd.print("                       ");
              M5.Lcd.setCursor(27, 172); M5.Lcd.print("                       ");
              M5.Lcd.setCursor(27, 188); M5.Lcd.print("                       ");
              M5.Lcd.setCursor(27, 204); M5.Lcd.print("                       ");

              M5.Lcd.setCursor(27, 156); M5.Lcd.print("   Calibration    ");
              M5.Lcd.setCursor(27, 172); M5.Lcd.print("   end            ");
              M5.Lcd.setCursor(27, 188); M5.Lcd.print("   can move       ");
              M5.Lcd.setCursor(27, 204); M5.Lcd.print("                  ");
              delay(1000);
              M5.Lcd.setCursor(27, 156); M5.Lcd.print("                       ");
              M5.Lcd.setCursor(27, 172); M5.Lcd.print("                       ");
              M5.Lcd.setCursor(27, 188); M5.Lcd.print("                       ");
              M5.Lcd.setCursor(27, 204); M5.Lcd.print("                       ");
              delay(100);

              M5.Lcd.fillScreen(0x0000);

              flip_btnC_bool = false;
              btnC_count = 0;

            } //function calibration deactivate
      */
      if (true)   //nouvelle fonction save data SD card
      {
        //debut datalog
        if (bool_push_btnc_detect == true)
        {
          //START SAVE DATA ON SD CARD

          //test presense SD card
          // see if the card is present and can be initialized:
          if (!SD.begin(chipSelect))
          {
            Serial.println("Card failed, or not present");
            // don't do anything more:
            bool_sdcard_error = true;
            bool_file_create_ok = false;

            //force variable interrupt to not enable save
            flip_btnC_bool = false;
            btnC_count = 0;
            bool_block_screen_anim = false;  //activation affichage ecran M5STACK
          }
          else
          {
            Serial.println("card initialized.");
            bool_sdcard_error = false;
          }

          if (bool_sdcard_error == false)
          {
            //increment numero fichier
            int_num_datalog++;

            //load name file for SD card saving
            sprintf(name_file, "/data/datalog_%0.3d.txt", int_num_datalog);

            //test si fihier existe et increment automatique
            while (SD.exists(name_file))
            {
              if (SD.exists(name_file))
              {
                //increment numero fichier
                int_num_datalog++;
                sprintf(name_file, "/data/datalog_%0.3d.txt", int_num_datalog);
              }
            }//increment auto numero fichier si existe

            // open the file. note that only one file can be open at a time,
            // so you have to close this one before opening another.
            //File dataFile = SD.open("datalog.txt", FILE_WRITE);
            dataFile = SD.open(name_file, FILE_WRITE);

            //prepare data
            //sprintf(data_to_file, "datalog_%0.3d.txt", int_num_datalog);
            sprintf(data_to_file, "Tick_ms,delay_cycle,AccX,AccY,AccZ,GyrX,GyrY,GyrZ,MagX,MagY,MagZ");

            // if the file is available, write to it:
            if (dataFile)
            {
              //validation acces fichier
              bool_file_create_ok = true;

              dataFile.println(data_to_file);
              //dataFile.close();
              // debug uart
              Serial.print("Opening file ok : ");
              Serial.println(name_file);
            }
            // if the file isn't open, pop up an error:
            else
            {
              // debug uart
              Serial.print("error opening file : ");
              Serial.println(name_file);
            }

            //M5.Lcd.drawJpgFile(SPIFFS, "/infos2.jpg", 0, 0);
            //delay(2000);

            M5.Lcd.drawJpgFile(SPIFFS, "/save_ok.jpg", 0, 0);

            M5.Lcd.setTextColor(WHITE);
            M5.Lcd.setTextSize(1);

            sprintf(buffer_ws, "datalog_%0.3d.txt", int_num_datalog);
            M5.Lcd.setCursor(170, 10); M5.Lcd.print(buffer_ws);
            M5.Lcd.setTextSize(1);
            
            delay(2000);

            //init itteruption buttonC
            bool_push_btnc_detect = false;
            //btnC_count = 0;

            M5.Lcd.fillScreen(0x0000);

            //info enrefistrement en cours
            M5.Lcd.setTextSize(2);

            sprintf(buffer_ws, "Enregistremen en cours");
            M5.Lcd.setCursor(10, 20); M5.Lcd.print(buffer_ws);

            sprintf(buffer_ws, "datalog_%0.3d.txt", int_num_datalog);
            M5.Lcd.setCursor(10, 40); M5.Lcd.print(buffer_ws);
            M5.Lcd.setTextSize(1);

            //numero fichier
            M5.Lcd.setTextColor(WHITE);
            M5.Lcd.setTextSize(1);
            M5.Lcd.setCursor(295, 190);
            M5.Lcd.print(int_num_datalog);

            M5.Lcd.drawJpgFile(SPIFFS, "/file_ok.jpg", 290, 200);

            //gestion mutli touch boutton C
            ulong_time_now = millis();
            if ((ulong_time_now > ulong_time_btC))
            {
              if ((btnC_count >= 3))
              {
                bool_block_screen_anim = false;  //activation affichage ecran M5STACK
              }
              btnC_count = 0;
            }

          }//sd card ok
          else
          {
            //pb sd card or sd card not present
            //M5.Lcd.drawJpgFile(SPIFFS, "/infos2.jpg", 0, 0);
            //delay(2000);

            M5.Lcd.drawJpgFile(SPIFFS, "/sd_error.jpg", 0, 0);
            delay(2000);

            //init interrupt
            bool_push_btnc_detect = false;
            //btnC_count = 0;
            bool_block_screen_anim = false;  //activation affichage ecran M5STACK

            M5.Lcd.fillScreen(0x0000);

            //numero fichier
            M5.Lcd.setTextColor(WHITE);
            M5.Lcd.setTextSize(1);
            M5.Lcd.setCursor(295, 190);
            M5.Lcd.print(int_num_datalog);

            M5.Lcd.drawJpgFile(SPIFFS, "/file_no.jpg", 290, 200);

            //gestion mutli touch boutton C
            ulong_time_now = millis();
            if ((ulong_time_now > ulong_time_btC))
            {
              if ((btnC_count >= 3))
              {
                bool_block_screen_anim = false;  //activation affichage ecran M5STACK
              }
              btnC_count = 0;
            }

          }

          //gestion icone led
          if (flip_btnB_bool == false)
          {
            M5.Lcd.drawJpgFile(SPIFFS, "/light_on.jpg", 265, 200);
          }
          else
          {
            M5.Lcd.drawJpgFile(SPIFFS, "/light_off.jpg", 265, 200);
          }
        }

      }

    } //gestion flip du boutton C
    else
    {
      //gestion si boutton C pas appuyer
      if (true)
      {
        //gestion du flip du boutton C FALSE
        if (bool_push_btnc_detect == true)
        {
          //CLOSE FILE ON SD CARD

          //Gestion cloture fichier
          // if the file is available, write to it:

          bool_file_create_ok = false;

          if (bool_sdcard_error == false)
          {
            //carte sd ok on ferme le fichier
            if (dataFile)
            {
              //dataFile.println(data_to_file);
              dataFile.close();
              // debug uart
              Serial.print("Closing file ok : ");
              Serial.println(name_file);
            }
            // if the file isn't open, pop up an error:
            else
            {
              // debug uart
              Serial.print("error opening file : ");
              Serial.println(name_file);
            }

            //M5.Lcd.drawJpgFile(SPIFFS, "/infos2.jpg", 0, 0);
            //delay(2000);

            M5.Lcd.drawJpgFile(SPIFFS, "/save_stop.jpg", 0, 0);

            M5.Lcd.setTextColor(WHITE);
            M5.Lcd.setTextSize(1);
            sprintf(buffer_ws, "datalog_%0.3d.txt", int_num_datalog);
            M5.Lcd.setCursor(170, 10); M5.Lcd.print(buffer_ws);

            delay(2000);

            //gestion mutli touch boutton C
            ulong_time_now = millis();
            if ((ulong_time_now > ulong_time_btC))
            {
              if ((btnC_count >= 3))
              {
                bool_block_screen_anim = false;  //activation affichage ecran M5STACK
              }
              btnC_count = 0;
            }

          } //sd card valid
          else
          {
            //probleme SD card
            M5.Lcd.drawJpgFile(SPIFFS, "/save_stop.jpg", 0, 0);
            delay(2000);

            //gestion mutli touch boutton C
            ulong_time_now = millis();
            if ((ulong_time_now > ulong_time_btC))
            {
              if ((btnC_count >= 3))
              {
                bool_block_screen_anim = false;  //activation affichage ecran M5STACK
              }
              btnC_count = 0;
            }
          }

          //erase screen
          M5.Lcd.fillScreen(0x0000);

          //rearm interrupt boutonC
          bool_push_btnc_detect = false;
          //btnC_count = 0;
          bool_block_screen_anim = false;  //activation affichage ecran M5STACK

          //numero fichier
          M5.Lcd.setTextColor(WHITE);
          M5.Lcd.setTextSize(1);
          M5.Lcd.setCursor(295, 190);
          M5.Lcd.print(int_num_datalog);

          //gestion icone file save or not
          M5.Lcd.drawJpgFile(SPIFFS, "/file_no.jpg", 290, 200);

          //gestion icone led
          if (flip_btnB_bool == false)
          {
            M5.Lcd.drawJpgFile(SPIFFS, "/light_on.jpg", 265, 200);
          }
          else
          {
            M5.Lcd.drawJpgFile(SPIFFS, "/light_off.jpg", 265, 200);
          }

        }
      }
    } //else gestion boutton C

    // affichage sur l'ecran du M5STACK
    if (true)
    {
      if (bool_block_screen_anim == false)
      {
        //debug valeur brute
        //erase screen message
        M5.Lcd.setTextColor(GREEN , BLACK);
        M5.Lcd.setTextSize(2);

        sprintf(buffer_ws, "Time: %d            ", ulong_time_now);
        M5.Lcd.setCursor(10, 16); M5.Lcd.print(buffer_ws);

        sprintf(buffer_ws, "cycle %d             ", delay_cycle);
        M5.Lcd.setCursor(10, 32); M5.Lcd.print(buffer_ws);

        sprintf(buffer_ws, "%0.3f", ax);
        M5.Lcd.setCursor(10, 48); M5.Lcd.print(buffer_ws);

        sprintf(buffer_ws, "%0.3f", ay);
        M5.Lcd.setCursor(10, 64); M5.Lcd.print(buffer_ws);

        sprintf(buffer_ws, "%0.3f", az);
        M5.Lcd.setCursor(10, 80); M5.Lcd.print(buffer_ws);


        sprintf(buffer_ws, "%0.3f", gx);
        M5.Lcd.setCursor(10, 96); M5.Lcd.print(buffer_ws);

        sprintf(buffer_ws, "%0.3f", gy);
        M5.Lcd.setCursor(10, 112); M5.Lcd.print(buffer_ws);

        sprintf(buffer_ws, "%0.3f", gz);
        M5.Lcd.setCursor(10, 128); M5.Lcd.print(buffer_ws);


        sprintf(buffer_ws, "%0.3f", mx);
        M5.Lcd.setCursor(120, 48); M5.Lcd.print(buffer_ws);

        sprintf(buffer_ws, "%0.3f", my);
        M5.Lcd.setCursor(120, 64); M5.Lcd.print(buffer_ws);

        sprintf(buffer_ws, "%0.3f", mz);
        M5.Lcd.setCursor(120, 80); M5.Lcd.print(buffer_ws);
      }

    }
    else
    {
      //affichage graphique sur l'ecran du M5STACK

      //block animation if save SD
      if (bool_block_screen_anim == false)
      {
        //5 phare ronde de part et d'autre
        int nb_led = 5;
        int angle = 90 / nb_led;
        int i = 0;
        int ecart = 10;

        //erase ALL LEDS
        for (int_i = 0; int_i < M5STICK_NUM_LEDS; int_i++)
        {
          //pixels.setPixelColor(int_i, pixels.Color(0, 0 , 0));
          leds[int_i] = CRGB::Black;
        }

        //eclairage de 5 cercles de part et d'autre
        for ( i = 1; i < (nb_led + 1); i++)
        {
          if (pitch >= 0 )
          {
            //eclairage de 5 cercles  a droite ::  pitch positif

            if (pitch >= ((angle - 10)*i) )
            {
              //u8g2.drawCircle((32 - 23 * sin((0 + angle * i * PI) / 180.0f)), (32 - 23 * cos((0 + angle * i * PI) / 180.0f)), nb_led - i + 1 , U8G2_DRAW_ALL);
              //pixels.setPixelColor(i, pixels.Color(10, 0 , 0));
              leds[i - 1] = CRGB::Red;
              M5.Lcd.fillCircle((160 + 40 * sin(((ecart + angle) * i * PI) / 180.0f)), (75 - 40 * cos(((ecart + angle) * i * PI) / 180.0f)), nb_led - i + 5 , 0xff80);
              M5.Lcd.drawCircleHelper(160, 75, 53 + i, 2, 0xe8e4);
              M5.Lcd.drawCircleHelper(160, 75, 63 + i, 2, 0xe8e4);
              M5.Lcd.fillTriangle(320, 0 + 30 * i, 320, 30 + 30 * i, 270, 15 + 30 * i, 0xe8e4);
              //M5.Lcd.fillCircle(320, 75, 10+i*4, 0xe8e4);
            }
            else
            {
              M5.Lcd.drawCircle((160 + 40 * sin(((ecart + angle) * i * PI) / 180.0f)), (75 - 40 * cos(((ecart + angle) * i * PI) / 180.0f)), nb_led - i + 6 , 0xFFFFFF);
              M5.Lcd.fillCircle((160 + 40 * sin(((ecart + angle) * i * PI) / 180.0f)), (75 - 40 * cos(((ecart + angle) * i * PI) / 180.0f)), nb_led - i + 5 , 0x000000);
              M5.Lcd.drawCircleHelper(160, 75, 53 + i, 2, 0x000000);
              M5.Lcd.drawCircleHelper(160, 75, 63 + i, 2, 0x000000);
              M5.Lcd.fillTriangle(320, 0 + 30 * i, 320, 30 + 30 * i, 270, 15 + 30 * i, 0x000000);
              //M5.Lcd.fillCircle(320, 75, 10+i*4, 0x000000);
            }

          }
          else
          {
            //eclairage de 5 cercles  a gauche :: pitch negatif
            if (-pitch >= ((angle - 10)*i))
            {
              //u8g2.drawCircle((32 + 23 * sin((0 + angle * i * PI) / 180.0f)), (32 - 23 * cos((0 + angle * i * PI) / 180.0f)), nb_led - i + 1 , U8G2_DRAW_ALL);
              //pixels.setPixelColor(11 - i, pixels.Color(10, 0 , 0));
              leds[10 - i] = CRGB::Red;
              M5.Lcd.fillCircle((160 - 40 * sin(((ecart + angle) * i * PI) / 180.0f)), (75 - 40 * cos(((ecart + angle) * i * PI) / 180.0f)), nb_led - i + 5 , 0xff80);
              M5.Lcd.drawCircleHelper(160, 75, 53 + i, 1, 0xe8e4);
              M5.Lcd.drawCircleHelper(160, 75, 63 + i, 1, 0xe8e4);
              M5.Lcd.fillTriangle(0, 0 + 30 * i, 0, 30 + 30 * i, 50, 15 + 30 * i, 0xe8e4);
              // M5.Lcd.fillCircle(0, 75, 10+i*4, 0xe8e4);
            }
            else
            {
              M5.Lcd.drawCircle((160 - 40 * sin(((ecart + angle) * i * PI) / 180.0f)), (75 - 40 * cos(((ecart + angle) * i * PI) / 180.0f)), nb_led - i + 6 , 0xFFFFFF);
              M5.Lcd.fillCircle((160 - 40 * sin(((ecart + angle) * i * PI) / 180.0f)), (75 - 40 * cos(((ecart + angle) * i * PI) / 180.0f)), nb_led - i + 5 , 0x000000);
              M5.Lcd.drawCircleHelper(160, 75, 53 + i, 1, 0x000000);
              M5.Lcd.drawCircleHelper(160, 75, 63 + i, 1, 0x000000);
              M5.Lcd.fillTriangle(0, 0 + 30 * i, 0, 30 + 30 * i, 50, 15 + 30 * i, 0x000000);
              //  M5.Lcd.fillCircle(0, 75, 10+i*4, 0x000000);
            }

          }
        }

        //Affichage de l'angle vertical info angle
        li_x1 = ((-50 * sin(cppitch * PI / 180.0f)) + 160);
        li_y1 = (200 - (50 * cos(cppitch * PI / 180.0f)));
        li_x2 = 160;
        li_y2 = 200;

        //M5.Lcd.drawLine(((100 * sin(cppitch * PI / 180.0f)) + 160), (240 - (100 * cos(cppitch * PI / 180.0f))) , 160, 240, 0xEEEE);
        if (int_first == 0)
        {
          //M5.Lcd.drawLine(li_x1_old + 2, li_y1_old, li_x2_old + 2, li_y2_old, 0x000000);
          //M5.Lcd.drawLine(li_x1_old - 2, li_y1_old, li_x2_old - 2, li_y2_old, 0x000000);
          M5.Lcd.drawLine(li_x1_old, li_y1_old, li_x2_old, li_y2_old, 0x000000);
        }
        int_first = 0;

        //indicateur verticalité - ligne vertical
        M5.Lcd.drawLine(li_x1, li_y1, li_x2, li_y2, 0xFFFFFF);

        li_x1_old = li_x1;
        li_y1_old = li_y1;
        li_x2_old = li_x2;
        li_y2_old = li_y2;

        //Gestion des lignes min max de l'angle d'inclinaison et maintien 10s
        if (cppitch_min < cppitch)
        {
          //init timing si nouveau minimum
          cppitch_min = (int)cppitch;
          ulong_time_min_tilt = millis() + 10000;

          //Mémorisation des minimums
          li_x1_min = li_x1;
          li_y1_min = li_y1;
          li_x2_min = li_x2;
          li_y2_min = li_y2;
        }

        if (cppitch_max > cppitch)
        {
          //init timing si nouveau maximum
          cppitch_max = (int)cppitch;
          ulong_time_max_tilt = millis() + 10000;

          //Mémorisation des maximums
          li_x1_max = li_x1;
          li_y1_max = li_y1;
          li_x2_max = li_x2;
          li_y2_max = li_y2;
        }

        //gestion de l'affichage ou de l'effacement de la ligne min
        if (ulong_time_now >= ulong_time_min_tilt)
        {
          cppitch_min = cppitch;
          M5.Lcd.drawLine(li_x1_min, li_y1_min, li_x2_min, li_y2_min, 0x00000);
        }
        else
        {
          if (ulong_time_now >= (ulong_time_min_tilt - 9950))
          {
            if ((ulong_time_now % 100))
            {
              M5.Lcd.drawLine(li_x1_min, li_y1_min, li_x2_min, li_y2_min, 0xe8ea);
            }
          }
        }

        //gestion de l'affichage ou de l'effacement de la ligne max
        if (ulong_time_now >= ulong_time_max_tilt)
        {
          cppitch_max = cppitch;
          M5.Lcd.drawLine(li_x1_max, li_y1_max, li_x2_max, li_y2_max, 0x00000);
        }
        else
        {
          if ((ulong_time_now % 100))
          {
            M5.Lcd.drawLine(li_x1_max, li_y1_max, li_x2_max, li_y2_max, 0xe8ea);
          }
        }

        //repetition indicateur verticalité - ligne vertical evite scintillement
        M5.Lcd.drawLine(li_x1, li_y1, li_x2, li_y2, 0xFFFFFF);

        //draw petit cercle circle de l'indicateur de verticalité
        M5.Lcd.drawCircle(li_x2, li_y2, 5, 0xFFFFFF);
        M5.Lcd.drawCircle(li_x2, li_y2, 6, 0xFFFFFF);

        //draw cadran de l'indicateur de verticalité
        M5.Lcd.drawCircleHelper(li_x2, li_y2, 52, 1, 0xFFFFFF);
        M5.Lcd.drawCircleHelper(li_x2, li_y2, 52, 2, 0xFFFFFF);

        M5.Lcd.drawCircleHelper(li_x2, li_y2, 50, 1, 0xFFFFFF);
        M5.Lcd.drawCircleHelper(li_x2, li_y2, 50, 2, 0xFFFFFF);

        //angle
        M5.Lcd.setTextColor(WHITE , BLACK);
        M5.Lcd.setTextSize(2);
        M5.Lcd.setCursor(5, (li_y2 + 10));

        //name SV
        M5.Lcd.print("SV");

        M5.Lcd.setTextSize(2);

        sprintf(buffer_ws, "%0.0fdeg ", abs(pitch));
        M5.Lcd.setCursor(140, (li_y2 + 10));
        M5.Lcd.print(buffer_ws);

        //pitch min max
        //Indication trait min et max
        M5.Lcd.setTextColor(RED , BLACK);
        M5.Lcd.setTextSize(2);

        if (cppitch_min > 0)
        {
          sprintf(buffer_ws, "-%0.0f ", abs(cppitch_min));
        }
        else
        {
          sprintf(buffer_ws, "+%0.0f ", abs(cppitch_min));
        }
        M5.Lcd.setCursor(80, (li_y2 + 10));
        M5.Lcd.print(buffer_ws);

        if (cppitch_max > 0)
        {
          sprintf(buffer_ws, "-%0.0f ", abs(cppitch_max));
        }
        else
        {
          sprintf(buffer_ws, "+%0.0f ", abs(cppitch_max));
        }
        M5.Lcd.setCursor(220, (li_y2 + 10));
        M5.Lcd.print(buffer_ws);

        //grand phare
        M5.Lcd.drawCircle(li_x2, (li_y2 - 125), 20, 0xFFFFFF);
        M5.Lcd.drawCircle(li_x2, (li_y2 - 125), 22, 0xFFFFFF);

        M5.Lcd.fillCircle((li_x2 - 10), (li_y2 - 125 - 10), 3, 0xD6FE01);


        M5.Lcd.drawCircle(li_x2, (li_y2 - 125), 60, 0xFFFFFF);
        M5.Lcd.drawCircle(li_x2, (li_y2 - 125), 62, 0xFFFFFF);

        //ACC min et max
        M5.Lcd.setTextColor(WHITE , BLACK);
        M5.Lcd.setTextSize(2);



        if ( max_acc < abs((norm_acc - 1)))
        {
          max_acc = abs((norm_acc - 1));
          ulong_time_max_acc = millis() + 10000;

          if ((ay - ay_old) > 0)
          {
            // M5.Lcd.setCursor(25, 5); M5.Lcd.print("+");
            M5.Lcd.setTextColor(RED , BLACK);
          }
          else
          {
            // M5.Lcd.setCursor(25, 5); M5.Lcd.print("-");
            M5.Lcd.setTextColor(GREEN , BLACK);
          }
          if (max_acc > 2)
          {
            max_acc = 2;
            sprintf(buffer_ws, "%0.1f", 2);
            M5.Lcd.setCursor(50, 5); M5.Lcd.print(buffer_ws);
          }
          else
          {
            sprintf(buffer_ws, "%0.1f", max_acc);
            M5.Lcd.setCursor(50, 5); M5.Lcd.print(buffer_ws);
          }

        }

        if ((ulong_time_now >= ulong_time_max_acc) && (ulong_time_now % 100))
        {
          //efface la bande acc
          M5.Lcd.fillRect(60, 20, 10, 220, 0x000000);

          //remise a zero de l'acceleration
          max_acc = 0;

          if ((ay - ay_old) > 0)
          {
            // M5.Lcd.setCursor(25, 5); M5.Lcd.print("+");
            M5.Lcd.setTextColor(RED , BLACK);
          }
          else
          {
            // M5.Lcd.setCursor(25, 5); M5.Lcd.print("-");
            M5.Lcd.setTextColor(GREEN , BLACK);
          }
          //sprintf(buffer_ws, "%0.1f", max_acc);
          //M5.Lcd.setCursor(50, 5); M5.Lcd.print(buffer_ws);

        }
        else
        {
          if ((ay - ay_old) > 0)
          {
            M5.Lcd.setTextColor(RED , BLACK);
            M5.Lcd.fillRect(60, 120, 10, ((int)(40 * max_acc)), 0xe8e4);
          }
          else
          {
            M5.Lcd.setTextColor(GREEN , BLACK);
            M5.Lcd.fillRect(60, (120 - ((int)(40 * max_acc))), 10, ((int)(40 * max_acc)), 0xe8e4);
          }

        }

        //sprintf(buffer_ws, "%0.1f", ay);
        //M5.Lcd.setCursor(40, 20); M5.Lcd.print(buffer_ws);

        //save ay pour le signe de la norme de l'acceleration
        ay_old = ay;

        //ligne jaune zero acc
        M5.Lcd.fillRect(55, 120, 20, 1, 0xff80);
      }

      //update ALL LEDS
      //***************
      FastLED.show();

    } // IF bool_block_screen_anim==true BLOCK animation if save SD



    //SAVE DATA ON SD CARD
    //********************
    if ( bool_file_create_ok == true)
    {
      //SAVE Data on SD card
      //prepare data
      //sprintf(data_to_file, "datalog_%0.3d.txt", int_num_datalog);
      sprintf(data_to_file, "%d,%d,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f", ulong_time_now, delay_cycle, ax, ay, az, gx, gy, gz, mx, my, mz);
      //Serial.print("Data : ");
      //Serial.println(data_to_file);

      // if the file is available, write to it:
      if (dataFile)
      {
        dataFile.println(data_to_file);
        //dataFile.close();
        // debug uart
        //Serial.print("Opening file ok  ");
        //Serial.println(name_file);
      }
      // if the file isn't open, pop up an error:
      else
      {
        // debug uart
        Serial.print("error opening file during saving data : ");
        Serial.println(name_file);
      }
    } //secure data save in sd card

  }  //M5STACK SCREEN AND LEDs

}  //LOOP

//===================================================================================================================
//====== Set of useful function to access acceleration. gyroscope, magnetometer, and temperature data
//===================================================================================================================

void getMres() {
  switch (Mscale)
  {
    // Possible magnetometer scales (and their register bit settings) are:
    // 14 bit resolution (0) and 16 bit resolution (1)
    case MFS_14BITS:
      mRes = 10.*4912. / 8190.; // Proper scale to return milliGauss
      break;
    case MFS_16BITS:
      mRes = 10.*4912. / 32760.0; // Proper scale to return milliGauss
      break;
  }
}

void getGres() {
  switch (Gscale)
  {
    // Possible gyro scales (and their register bit settings) are:
    // 250 DPS (00), 500 DPS (01), 1000 DPS (10), and 2000 DPS  (11).
    // Here's a bit of an algorith to calculate DPS/(ADC tick) based on that 2-bit value:
    case GFS_250DPS:
      gRes = 250.0 / 32768.0;
      break;
    case GFS_500DPS:
      gRes = 500.0 / 32768.0;
      break;
    case GFS_1000DPS:
      gRes = 1000.0 / 32768.0;
      break;
    case GFS_2000DPS:
      gRes = 2000.0 / 32768.0;
      break;
  }
}

void getAres() {
  switch (Ascale)
  {
    // Possible accelerometer scales (and their register bit settings) are:
    // 2 Gs (00), 4 Gs (01), 8 Gs (10), and 16 Gs  (11).
    // Here's a bit of an algorith to calculate DPS/(ADC tick) based on that 2-bit value:
    case AFS_2G:
      aRes = 2.0 / 32768.0;
      break;
    case AFS_4G:
      aRes = 4.0 / 32768.0;
      break;
    case AFS_8G:
      aRes = 8.0 / 32768.0;
      break;
    case AFS_16G:
      aRes = 16.0 / 32768.0;
      break;
  }
}


void readAccelData(int16_t * destination)
{
  uint8_t rawData[6];  // x/y/z accel register data stored here
  readBytes(MPU9250_ADDRESS, ACCEL_XOUT_H, 6, &rawData[0]);  // Read the six raw data registers into data array
  destination[0] = ((int16_t)rawData[0] << 8) | rawData[1] ;  // Turn the MSB and LSB into a signed 16-bit value
  destination[1] = ((int16_t)rawData[2] << 8) | rawData[3] ;
  destination[2] = ((int16_t)rawData[4] << 8) | rawData[5] ;
}


void readGyroData(int16_t * destination)
{
  uint8_t rawData[6];  // x/y/z gyro register data stored here
  readBytes(MPU9250_ADDRESS, GYRO_XOUT_H, 6, &rawData[0]);  // Read the six raw data registers sequentially into data array
  destination[0] = ((int16_t)rawData[0] << 8) | rawData[1] ;  // Turn the MSB and LSB into a signed 16-bit value
  destination[1] = ((int16_t)rawData[2] << 8) | rawData[3] ;
  destination[2] = ((int16_t)rawData[4] << 8) | rawData[5] ;
}

void readMagData(int16_t * destination)
{
  uint8_t rawData[7];  // x/y/z gyro register data, ST2 register stored here, must read ST2 at end of data acquisition
  if (readByte(AK8963_ADDRESS, AK8963_ST1) & 0x01) { // wait for magnetometer data ready bit to be set
    readBytes(AK8963_ADDRESS, AK8963_XOUT_L, 7, &rawData[0]);  // Read the six raw data and ST2 registers sequentially into data array
    uint8_t c = rawData[6]; // End data read by reading ST2 register
    if (!(c & 0x08)) { // Check if magnetic sensor overflow set, if not then report data
      destination[0] = ((int16_t)rawData[1] << 8) | rawData[0] ;  // Turn the MSB and LSB into a signed 16-bit value
      destination[1] = ((int16_t)rawData[3] << 8) | rawData[2] ;  // Data stored as little Endian
      destination[2] = ((int16_t)rawData[5] << 8) | rawData[4] ;
    }
  }
}

int16_t readTempData()
{
  uint8_t rawData[2];  // x/y/z gyro register data stored here
  readBytes(MPU9250_ADDRESS, TEMP_OUT_H, 2, &rawData[0]);  // Read the two raw data registers sequentially into data array
  return ((int16_t)rawData[0] << 8) | rawData[1] ;  // Turn the MSB and LSB into a 16-bit value
}

void initAK8963(float * destination)
{
  // First extract the factory calibration for each magnetometer axis
  uint8_t rawData[3];  // x/y/z gyro calibration data stored here
  writeByte(AK8963_ADDRESS, AK8963_CNTL, 0x00); // Power down magnetometer
  delay(10);
  writeByte(AK8963_ADDRESS, AK8963_CNTL, 0x0F); // Enter Fuse ROM access mode
  delay(10);
  readBytes(AK8963_ADDRESS, AK8963_ASAX, 3, &rawData[0]);  // Read the x-, y-, and z-axis calibration values
  destination[0] =  (float)(rawData[0] - 128) / 256. + 1.; // Return x-axis sensitivity adjustment values, etc.
  destination[1] =  (float)(rawData[1] - 128) / 256. + 1.;
  destination[2] =  (float)(rawData[2] - 128) / 256. + 1.;
  writeByte(AK8963_ADDRESS, AK8963_CNTL, 0x00); // Power down magnetometer
  delay(10);
  // Configure the magnetometer for continuous read and highest resolution
  // set Mscale bit 4 to 1 (0) to enable 16 (14) bit resolution in CNTL register,
  // and enable continuous mode data acquisition Mmode (bits [3:0]), 0010 for 8 Hz and 0110 for 100 Hz sample rates
  writeByte(AK8963_ADDRESS, AK8963_CNTL, Mscale << 4 | Mmode); // Set magnetometer data resolution and sample ODR
  delay(10);
}


void initMPU9250()
{
  // wake up device
  writeByte(MPU9250_ADDRESS, PWR_MGMT_1, 0x00); // Clear sleep mode bit (6), enable all sensors
  delay(100); // Wait for all registers to reset

  // get stable time source
  writeByte(MPU9250_ADDRESS, PWR_MGMT_1, 0x01);  // Auto select clock source to be PLL gyroscope reference if ready else
  delay(200);

  // Configure Gyro and Thermometer
  // Disable FSYNC and set thermometer and gyro bandwidth to 41 and 42 Hz, respectively;
  // minimum delay time for this setting is 5.9 ms, which means sensor fusion update rates cannot
  // be higher than 1 / 0.0059 = 170 Hz
  // DLPF_CFG = bits 2:0 = 011; this limits the sample rate to 1000 Hz for both
  // With the MPU9250, it is possible to get gyro sample rates of 32 kHz (!), 8 kHz, or 1 kHz
  writeByte(MPU9250_ADDRESS, CONFIG, 0x03);

  // Set sample rate = gyroscope output rate/(1 + SMPLRT_DIV)
  writeByte(MPU9250_ADDRESS, SMPLRT_DIV, 0x04);  // Use a 200 Hz rate; a rate consistent with the filter update rate
  // determined inset in CONFIG above

  // Set gyroscope full scale range
  // Range selects FS_SEL and GFS_SEL are 0 - 3, so 2-bit values are left-shifted into positions 4:3
  uint8_t c = readByte(MPU9250_ADDRESS, GYRO_CONFIG); // get current GYRO_CONFIG register value
  // c = c & ~0xE0; // Clear self-test bits [7:5]
  c = c & ~0x03; // Clear Fchoice bits [1:0]
  c = c & ~0x18; // Clear GFS bits [4:3]
  c = c | Gscale << 3; // Set full scale range for the gyro
  // c =| 0x00; // Set Fchoice for the gyro to 11 by writing its inverse to bits 1:0 of GYRO_CONFIG
  writeByte(MPU9250_ADDRESS, GYRO_CONFIG, c ); // Write new GYRO_CONFIG value to register

  // Set accelerometer full-scale range configuration
  c = readByte(MPU9250_ADDRESS, ACCEL_CONFIG); // get current ACCEL_CONFIG register value
  // c = c & ~0xE0; // Clear self-test bits [7:5]
  c = c & ~0x18;  // Clear AFS bits [4:3]
  c = c | Ascale << 3; // Set full scale range for the accelerometer
  writeByte(MPU9250_ADDRESS, ACCEL_CONFIG, c); // Write new ACCEL_CONFIG register value

  // Set accelerometer sample rate configuration
  // It is possible to get a 4 kHz sample rate from the accelerometer by choosing 1 for
  // accel_fchoice_b bit [3]; in this case the bandwidth is 1.13 kHz
  c = readByte(MPU9250_ADDRESS, ACCEL_CONFIG2); // get current ACCEL_CONFIG2 register value
  c = c & ~0x0F; // Clear accel_fchoice_b (bit 3) and A_DLPFG (bits [2:0])
  c = c | 0x03;  // Set accelerometer rate to 1 kHz and bandwidth to 41 Hz
  writeByte(MPU9250_ADDRESS, ACCEL_CONFIG2, c); // Write new ACCEL_CONFIG2 register value
  // The accelerometer, gyro, and thermometer are set to 1 kHz sample rates,
  // but all these rates are further reduced by a factor of 5 to 200 Hz because of the SMPLRT_DIV setting

  // Configure Interrupts and Bypass Enable
  // Set interrupt pin active high, push-pull, hold interrupt pin level HIGH until interrupt cleared,
  // clear on read of INT_STATUS, and enable I2C_BYPASS_EN so additional chips
  // can join the I2C bus and all can be controlled by the Arduino as master
  writeByte(MPU9250_ADDRESS, INT_PIN_CFG, 0x22);
  writeByte(MPU9250_ADDRESS, INT_ENABLE, 0x01);  // Enable data ready (bit 0) interrupt
  delay(100);
}


// Function which accumulates gyro and accelerometer data after device initialization. It calculates the average
// of the at-rest readings and then loads the resulting offsets into accelerometer and gyro bias registers.
void calibrateMPU9250(float * dest1, float * dest2)
{
  uint8_t data[12]; // data array to hold accelerometer and gyro x, y, z, data
  uint16_t ii, packet_count, fifo_count;
  int32_t gyro_bias[3]  = {0, 0, 0}, accel_bias[3] = {0, 0, 0};

  // reset device
  writeByte(MPU9250_ADDRESS, PWR_MGMT_1, 0x80); // Write a one to bit 7 reset bit; toggle reset device
  delay(100);

  // get stable time source; Auto select clock source to be PLL gyroscope reference if ready
  // else use the internal oscillator, bits 2:0 = 001
  writeByte(MPU9250_ADDRESS, PWR_MGMT_1, 0x01);
  writeByte(MPU9250_ADDRESS, PWR_MGMT_2, 0x00);
  delay(200);

  // Configure device for bias calculation
  writeByte(MPU9250_ADDRESS, INT_ENABLE, 0x00);   // Disable all interrupts
  writeByte(MPU9250_ADDRESS, FIFO_EN, 0x00);      // Disable FIFO
  writeByte(MPU9250_ADDRESS, PWR_MGMT_1, 0x00);   // Turn on internal clock source
  writeByte(MPU9250_ADDRESS, I2C_MST_CTRL, 0x00); // Disable I2C master
  writeByte(MPU9250_ADDRESS, USER_CTRL, 0x00);    // Disable FIFO and I2C master modes
  writeByte(MPU9250_ADDRESS, USER_CTRL, 0x0C);    // Reset FIFO and DMP
  delay(15);

  // Configure MPU6050 gyro and accelerometer for bias calculation
  writeByte(MPU9250_ADDRESS, CONFIG, 0x01);      // Set low-pass filter to 188 Hz
  writeByte(MPU9250_ADDRESS, SMPLRT_DIV, 0x00);  // Set sample rate to 1 kHz
  writeByte(MPU9250_ADDRESS, GYRO_CONFIG, 0x00);  // Set gyro full-scale to 250 degrees per second, maximum sensitivity
  writeByte(MPU9250_ADDRESS, ACCEL_CONFIG, 0x00); // Set accelerometer full-scale to 2 g, maximum sensitivity

  uint16_t  gyrosensitivity  = 131;   // = 131 LSB/degrees/sec
  uint16_t  accelsensitivity = 16384;  // = 16384 LSB/g

  // Configure FIFO to capture accelerometer and gyro data for bias calculation
  writeByte(MPU9250_ADDRESS, USER_CTRL, 0x40);   // Enable FIFO
  writeByte(MPU9250_ADDRESS, FIFO_EN, 0x78);     // Enable gyro and accelerometer sensors for FIFO  (max size 512 bytes in MPU-9150)
  delay(40); // accumulate 40 samples in 40 milliseconds = 480 bytes

  // At end of sample accumulation, turn off FIFO sensor read
  writeByte(MPU9250_ADDRESS, FIFO_EN, 0x00);        // Disable gyro and accelerometer sensors for FIFO
  readBytes(MPU9250_ADDRESS, FIFO_COUNTH, 2, &data[0]); // read FIFO sample count
  fifo_count = ((uint16_t)data[0] << 8) | data[1];
  packet_count = fifo_count / 12; // How many sets of full gyro and accelerometer data for averaging

  for (ii = 0; ii < packet_count; ii++) {
    int16_t accel_temp[3] = {0, 0, 0}, gyro_temp[3] = {0, 0, 0};
    readBytes(MPU9250_ADDRESS, FIFO_R_W, 12, &data[0]); // read data for averaging
    accel_temp[0] = (int16_t) (((int16_t)data[0] << 8) | data[1]  ) ;  // Form signed 16-bit integer for each sample in FIFO
    accel_temp[1] = (int16_t) (((int16_t)data[2] << 8) | data[3]  ) ;
    accel_temp[2] = (int16_t) (((int16_t)data[4] << 8) | data[5]  ) ;
    gyro_temp[0]  = (int16_t) (((int16_t)data[6] << 8) | data[7]  ) ;
    gyro_temp[1]  = (int16_t) (((int16_t)data[8] << 8) | data[9]  ) ;
    gyro_temp[2]  = (int16_t) (((int16_t)data[10] << 8) | data[11]) ;

    accel_bias[0] += (int32_t) accel_temp[0]; // Sum individual signed 16-bit biases to get accumulated signed 32-bit biases
    accel_bias[1] += (int32_t) accel_temp[1];
    accel_bias[2] += (int32_t) accel_temp[2];
    gyro_bias[0]  += (int32_t) gyro_temp[0];
    gyro_bias[1]  += (int32_t) gyro_temp[1];
    gyro_bias[2]  += (int32_t) gyro_temp[2];

  }
  accel_bias[0] /= (int32_t) packet_count; // Normalize sums to get average count biases
  accel_bias[1] /= (int32_t) packet_count;
  accel_bias[2] /= (int32_t) packet_count;
  gyro_bias[0]  /= (int32_t) packet_count;
  gyro_bias[1]  /= (int32_t) packet_count;
  gyro_bias[2]  /= (int32_t) packet_count;

  if (accel_bias[2] > 0L) {
    accel_bias[2] -= (int32_t) accelsensitivity; // Remove gravity from the z-axis accelerometer bias calculation
  }
  else {
    accel_bias[2] += (int32_t) accelsensitivity;
  }

  // Construct the gyro biases for push to the hardware gyro bias registers, which are reset to zero upon device startup
  data[0] = (-gyro_bias[0] / 4  >> 8) & 0xFF; // Divide by 4 to get 32.9 LSB per deg/s to conform to expected bias input format
  data[1] = (-gyro_bias[0] / 4)       & 0xFF; // Biases are additive, so change sign on calculated average gyro biases
  data[2] = (-gyro_bias[1] / 4  >> 8) & 0xFF;
  data[3] = (-gyro_bias[1] / 4)       & 0xFF;
  data[4] = (-gyro_bias[2] / 4  >> 8) & 0xFF;
  data[5] = (-gyro_bias[2] / 4)       & 0xFF;

  // Push gyro biases to hardware registers
  writeByte(MPU9250_ADDRESS, XG_OFFSET_H, data[0]);
  writeByte(MPU9250_ADDRESS, XG_OFFSET_L, data[1]);
  writeByte(MPU9250_ADDRESS, YG_OFFSET_H, data[2]);
  writeByte(MPU9250_ADDRESS, YG_OFFSET_L, data[3]);
  writeByte(MPU9250_ADDRESS, ZG_OFFSET_H, data[4]);
  writeByte(MPU9250_ADDRESS, ZG_OFFSET_L, data[5]);

  // Output scaled gyro biases for display in the main program
  dest1[0] = (float) gyro_bias[0] / (float) gyrosensitivity;
  dest1[1] = (float) gyro_bias[1] / (float) gyrosensitivity;
  dest1[2] = (float) gyro_bias[2] / (float) gyrosensitivity;

  // Construct the accelerometer biases for push to the hardware accelerometer bias registers. These registers contain
  // factory trim values which must be added to the calculated accelerometer biases; on boot up these registers will hold
  // non-zero values. In addition, bit 0 of the lower byte must be preserved since it is used for temperature
  // compensation calculations. Accelerometer bias registers expect bias input as 2048 LSB per g, so that
  // the accelerometer biases calculated above must be divided by 8.

  int32_t accel_bias_reg[3] = {0, 0, 0}; // A place to hold the factory accelerometer trim biases
  readBytes(MPU9250_ADDRESS, XA_OFFSET_H, 2, &data[0]); // Read factory accelerometer trim values
  accel_bias_reg[0] = (int32_t) (((int16_t)data[0] << 8) | data[1]);
  readBytes(MPU9250_ADDRESS, YA_OFFSET_H, 2, &data[0]);
  accel_bias_reg[1] = (int32_t) (((int16_t)data[0] << 8) | data[1]);
  readBytes(MPU9250_ADDRESS, ZA_OFFSET_H, 2, &data[0]);
  accel_bias_reg[2] = (int32_t) (((int16_t)data[0] << 8) | data[1]);

  uint32_t mask = 1uL; // Define mask for temperature compensation bit 0 of lower byte of accelerometer bias registers
  uint8_t mask_bit[3] = {0, 0, 0}; // Define array to hold mask bit for each accelerometer bias axis

  for (ii = 0; ii < 3; ii++) {
    if ((accel_bias_reg[ii] & mask)) mask_bit[ii] = 0x01; // If temperature compensation bit is set, record that fact in mask_bit
  }

  // Construct total accelerometer bias, including calculated average accelerometer bias from above
  accel_bias_reg[0] -= (accel_bias[0] / 8); // Subtract calculated averaged accelerometer bias scaled to 2048 LSB/g (16 g full scale)
  accel_bias_reg[1] -= (accel_bias[1] / 8);
  accel_bias_reg[2] -= (accel_bias[2] / 8);

  data[0] = (accel_bias_reg[0] >> 8) & 0xFF;
  data[1] = (accel_bias_reg[0])      & 0xFF;
  data[1] = data[1] | mask_bit[0]; // preserve temperature compensation bit when writing back to accelerometer bias registers
  data[2] = (accel_bias_reg[1] >> 8) & 0xFF;
  data[3] = (accel_bias_reg[1])      & 0xFF;
  data[3] = data[3] | mask_bit[1]; // preserve temperature compensation bit when writing back to accelerometer bias registers
  data[4] = (accel_bias_reg[2] >> 8) & 0xFF;
  data[5] = (accel_bias_reg[2])      & 0xFF;
  data[5] = data[5] | mask_bit[2]; // preserve temperature compensation bit when writing back to accelerometer bias registers

  // Apparently this is not working for the acceleration biases in the MPU-9250
  // Are we handling the temperature correction bit properly?
  // Push accelerometer biases to hardware registers
  writeByte(MPU9250_ADDRESS, XA_OFFSET_H, data[0]);
  writeByte(MPU9250_ADDRESS, XA_OFFSET_L, data[1]);
  writeByte(MPU9250_ADDRESS, YA_OFFSET_H, data[2]);
  writeByte(MPU9250_ADDRESS, YA_OFFSET_L, data[3]);
  writeByte(MPU9250_ADDRESS, ZA_OFFSET_H, data[4]);
  writeByte(MPU9250_ADDRESS, ZA_OFFSET_L, data[5]);

  // Output scaled accelerometer biases for display in the main program
  dest2[0] = (float)accel_bias[0] / (float)accelsensitivity;
  dest2[1] = (float)accel_bias[1] / (float)accelsensitivity;
  dest2[2] = (float)accel_bias[2] / (float)accelsensitivity;
}


/*
  // Calibration capteur magnetique 23/07/2020
  void magcalMPU9250(float * dest1, float * dest2)
  {
  uint16_t ii = 0, sample_count = 0;
  int32_t mag_bias[3] = {0, 0, 0}, mag_scale[3] = {0, 0, 0};
  int16_t mag_max[3] = { -32767, -32767, -32767}, mag_min[3] = {32767, 32767, 32767}, mag_temp[3] = {0, 0, 0};

  Serial.println("Mag Calibration: Wave device in a figure eight until done!");
  delay(4000);

  // shoot for ~fifteen seconds of mag data
  if (Mmode == 0x02) sample_count = 128; // at 8 Hz ODR, new mag data is available every 125 ms
  if (Mmode == 0x06) sample_count = 1500; // at 100 Hz ODR, new mag data is available every 10 ms
  for (ii = 0; ii < sample_count; ii++) {
    readMagData(mag_temp);  // Read the mag data
    for (int jj = 0; jj < 3; jj++) {
      if (mag_temp[jj] > mag_max[jj]) mag_max[jj] = mag_temp[jj];
      if (mag_temp[jj] < mag_min[jj]) mag_min[jj] = mag_temp[jj];
    }
    if (Mmode == 0x02) delay(135); // at 8 Hz ODR, new mag data is available every 125 ms
    if (Mmode == 0x06) delay(12); // at 100 Hz ODR, new mag data is available every 10 ms
  }


  // Get hard iron correction
  mag_bias[0]  = (mag_max[0] + mag_min[0]) / 2; // get average x mag bias in counts
  mag_bias[1]  = (mag_max[1] + mag_min[1]) / 2; // get average y mag bias in counts
  mag_bias[2]  = (mag_max[2] + mag_min[2]) / 2; // get average z mag bias in counts

  dest1[0] = (float) mag_bias[0] * MPU9250mRes * MPU9250magCalibration[0]; // save mag biases in G for main program
  dest1[1] = (float) mag_bias[1] * MPU9250mRes * MPU9250magCalibration[1];
  dest1[2] = (float) mag_bias[2] * MPU9250mRes * MPU9250magCalibration[2];

  // Get soft iron correction estimate
  mag_scale[0]  = (mag_max[0] - mag_min[0]) / 2; // get average x axis max chord length in counts
  mag_scale[1]  = (mag_max[1] - mag_min[1]) / 2; // get average y axis max chord length in counts
  mag_scale[2]  = (mag_max[2] - mag_min[2]) / 2; // get average z axis max chord length in counts

  float avg_rad = mag_scale[0] + mag_scale[1] + mag_scale[2];
  avg_rad /= 3.0;

  dest2[0] = avg_rad / ((float)mag_scale[0]);
  dest2[1] = avg_rad / ((float)mag_scale[1]);
  dest2[2] = avg_rad / ((float)mag_scale[2]);

  Serial.println("Mag Calibration done!");
  }
*/


// Accelerometer and gyroscope self test; check calibration wrt factory settings
void MPU9250SelfTest(float * destination) // Should return percent deviation from factory trim values, +/- 14 or less deviation is a pass
{
  uint8_t rawData[6] = {0, 0, 0, 0, 0, 0};
  uint8_t selfTest[6];
  int32_t gAvg[3] = {0}, aAvg[3] = {0}, aSTAvg[3] = {0}, gSTAvg[3] = {0};
  float factoryTrim[6];
  uint8_t FS = 0;

  writeByte(MPU9250_ADDRESS, SMPLRT_DIV, 0x00);    // Set gyro sample rate to 1 kHz
  writeByte(MPU9250_ADDRESS, CONFIG, 0x02);        // Set gyro sample rate to 1 kHz and DLPF to 92 Hz
  writeByte(MPU9250_ADDRESS, GYRO_CONFIG, FS << 3); // Set full scale range for the gyro to 250 dps
  writeByte(MPU9250_ADDRESS, ACCEL_CONFIG2, 0x02); // Set accelerometer rate to 1 kHz and bandwidth to 92 Hz
  writeByte(MPU9250_ADDRESS, ACCEL_CONFIG, FS << 3); // Set full scale range for the accelerometer to 2 g

  for ( int ii = 0; ii < 200; ii++) { // get average current values of gyro and acclerometer

    readBytes(MPU9250_ADDRESS, ACCEL_XOUT_H, 6, &rawData[0]);        // Read the six raw data registers into data array
    aAvg[0] += (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]) ;  // Turn the MSB and LSB into a signed 16-bit value
    aAvg[1] += (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]) ;
    aAvg[2] += (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]) ;

    readBytes(MPU9250_ADDRESS, GYRO_XOUT_H, 6, &rawData[0]);       // Read the six raw data registers sequentially into data array
    gAvg[0] += (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]) ;  // Turn the MSB and LSB into a signed 16-bit value
    gAvg[1] += (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]) ;
    gAvg[2] += (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]) ;
  }

  for (int ii = 0; ii < 3; ii++) { // Get average of 200 values and store as average current readings
    aAvg[ii] /= 200;
    gAvg[ii] /= 200;
  }

  // Configure the accelerometer for self-test
  writeByte(MPU9250_ADDRESS, ACCEL_CONFIG, 0xE0); // Enable self test on all three axes and set accelerometer range to +/- 2 g
  writeByte(MPU9250_ADDRESS, GYRO_CONFIG,  0xE0); // Enable self test on all three axes and set gyro range to +/- 250 degrees/s
  delay(25);  // Delay a while to let the device stabilize

  for ( int ii = 0; ii < 200; ii++) { // get average self-test values of gyro and acclerometer

    readBytes(MPU9250_ADDRESS, ACCEL_XOUT_H, 6, &rawData[0]);  // Read the six raw data registers into data array
    aSTAvg[0] += (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]) ;  // Turn the MSB and LSB into a signed 16-bit value
    aSTAvg[1] += (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]) ;
    aSTAvg[2] += (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]) ;

    readBytes(MPU9250_ADDRESS, GYRO_XOUT_H, 6, &rawData[0]);  // Read the six raw data registers sequentially into data array
    gSTAvg[0] += (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]) ;  // Turn the MSB and LSB into a signed 16-bit value
    gSTAvg[1] += (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]) ;
    gSTAvg[2] += (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]) ;
  }

  for (int ii = 0; ii < 3; ii++) { // Get average of 200 values and store as average self-test readings
    aSTAvg[ii] /= 200;
    gSTAvg[ii] /= 200;
  }

  // Configure the gyro and accelerometer for normal operation
  writeByte(MPU9250_ADDRESS, ACCEL_CONFIG, 0x00);
  writeByte(MPU9250_ADDRESS, GYRO_CONFIG,  0x00);
  delay(25);  // Delay a while to let the device stabilize

  // Retrieve accelerometer and gyro factory Self-Test Code from USR_Reg
  selfTest[0] = readByte(MPU9250_ADDRESS, SELF_TEST_X_ACCEL); // X-axis accel self-test results
  selfTest[1] = readByte(MPU9250_ADDRESS, SELF_TEST_Y_ACCEL); // Y-axis accel self-test results
  selfTest[2] = readByte(MPU9250_ADDRESS, SELF_TEST_Z_ACCEL); // Z-axis accel self-test results
  selfTest[3] = readByte(MPU9250_ADDRESS, SELF_TEST_X_GYRO);  // X-axis gyro self-test results
  selfTest[4] = readByte(MPU9250_ADDRESS, SELF_TEST_Y_GYRO);  // Y-axis gyro self-test results
  selfTest[5] = readByte(MPU9250_ADDRESS, SELF_TEST_Z_GYRO);  // Z-axis gyro self-test results

  // Retrieve factory self-test value from self-test code reads
  factoryTrim[0] = (float)(2620 / 1 << FS) * (pow( 1.01 , ((float)selfTest[0] - 1.0) )); // FT[Xa] factory trim calculation
  factoryTrim[1] = (float)(2620 / 1 << FS) * (pow( 1.01 , ((float)selfTest[1] - 1.0) )); // FT[Ya] factory trim calculation
  factoryTrim[2] = (float)(2620 / 1 << FS) * (pow( 1.01 , ((float)selfTest[2] - 1.0) )); // FT[Za] factory trim calculation
  factoryTrim[3] = (float)(2620 / 1 << FS) * (pow( 1.01 , ((float)selfTest[3] - 1.0) )); // FT[Xg] factory trim calculation
  factoryTrim[4] = (float)(2620 / 1 << FS) * (pow( 1.01 , ((float)selfTest[4] - 1.0) )); // FT[Yg] factory trim calculation
  factoryTrim[5] = (float)(2620 / 1 << FS) * (pow( 1.01 , ((float)selfTest[5] - 1.0) )); // FT[Zg] factory trim calculation

  // Report results as a ratio of (STR - FT)/FT; the change from Factory Trim of the Self-Test Response
  // To get percent, must multiply by 100
  for (int i = 0; i < 3; i++) {
    destination[i]   = 100.0 * ((float)(aSTAvg[i] - aAvg[i])) / factoryTrim[i] - 100.; // Report percent differences
    destination[i + 3] = 100.0 * ((float)(gSTAvg[i] - gAvg[i])) / factoryTrim[i + 3] - 100.; // Report percent differences
  }

}



// Wire.h read and write protocols
void writeByte(uint8_t address, uint8_t subAddress, uint8_t data)
{
  Wire.beginTransmission(address);  // Initialize the Tx buffer
  Wire.write(subAddress);           // Put slave register address in Tx buffer
  Wire.write(data);                 // Put data in Tx buffer
  Wire.endTransmission();           // Send the Tx buffer
}



uint8_t readByte(uint8_t address, uint8_t subAddress)
{
  uint8_t data; // `data` will store the register data
  Wire.beginTransmission(address);         // Initialize the Tx buffer
  Wire.write(subAddress);                  // Put slave register address in Tx buffer
  Wire.endTransmission(false);             // Send the Tx buffer, but send a restart to keep connection alive
  Wire.requestFrom(address, (uint8_t) 1);  // Read one byte from slave register address
  data = Wire.read();                      // Fill Rx buffer with result
  return data;                             // Return data read from slave register
}

void readBytes(uint8_t address, uint8_t subAddress, uint8_t count, uint8_t * dest)
{
  Wire.beginTransmission(address);   // Initialize the Tx buffer
  Wire.write(subAddress);            // Put slave register address in Tx buffer
  Wire.endTransmission(false);       // Send the Tx buffer, but send a restart to keep connection alive
  uint8_t i = 0;
  Wire.requestFrom(address, count);  // Read bytes from slave register address
  while (Wire.available()) {
    dest[i++] = Wire.read();
  }         // Put read results in the Rx buffer
}
