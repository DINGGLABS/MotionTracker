/** ===========================================================
 *  test MotionTracker V02
 *
 * \file       test_MotionTracker_V02
 * \author     jh
 * \date       xx.06.2015  
 * \subversion -00
 *
 * \controller ATmega32U2
 * \f_clk      ext. 8MHz
 *
 * \brief      SW to control the DINGGLABS MotionTracker
 *
 * \history   -00 creating the file
 * @{
 ============================================================== */

/* Configurable defines --------------------------------------- */
#define BAUDRATE                    115200    // e.g. 9600 or 19200 or 57600 or 115200

/* Includes --------------------------------------------------- */
#include <avr/sleep.h>
#include <avr/interrupt.h>
#include <SPI.h>
// #include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM9DS0.h>
// #include "SFE_LSM9DS0.h"

/* PIN definitions -------------------------------------------- */
// ATMEL ATMEGA32U2 / Minimus 32 => MotionTracker_V01
//
// Pins numbered counter-clockwise starting from USB connector
//
// D0		PC2		EN_G
// D1#		PD0		INT1_XM
// D2		PD1		INT2_XM
// D3		PD2		INT_G
// D4		PD3		DRDY_G
// D5		PD4		CS_XM
// D6		PD5		CS_G
// D7		PD6		CS_F
// D8		PD7		-
// D9		PB0		-
// D10		PB1		SCK
// [D11]			
// D12		PB2		MOSI
// D13		PB3		MISO
// D14		PB4		RED
// D15		PB5		BLUE
// D16		PB6		GREEN
// D17#		PB7		-
// D18		PC7		BUTTON
// D19#		PC6		-
// [D20]			
// D21#		PC5		HOLD
// D22		PC4		WP

/* MotionTracker_V01 -> Minimus 32 */
#define BUTTON    18
#define RED       14
#define GREEN     16
#define BLUE      15
//#define MOSI      12 
//#define SCK       10
//#define MISO      13
#define CS_XM      5
#define CS_G       6
#define INT2_XM    2
#define INT1_XM    1
#define HOLD      21
#define WP        22
#define CS_F       7
#define DRDY_G     4
#define INT_G      3
#define EN_G       0

/* unused PINS */
#define UU_0       8
#define UU_1       9
#define UU_2      17
#define UU_3      19

/* port manipulation makros */
#ifndef setBit
#define setBit(reg, b) (_SFR_BYTE(reg) |= _BV(b))
#endif 
#ifndef clearBit
#define clearBit(reg, b) (_SFR_BYTE(reg) &= ~_BV(b))
#endif
#ifndef getBit
#define getBit(reg, b) (_SFR_BYTE(reg) & _BV(b))
#endif

/* General defines -------------------------------------------- */
#define FOREVER      true
#define UP           true
#define DOWN         false

/* Global variables ------------------------------------------- */
/* using HW SPI to communicate with the sensor */
Adafruit_LSM9DS0 lsm = Adafruit_LSM9DS0(CS_XM, CS_G, 1000);

/** ===========================================================
 * \fn      setup
 * \brief   standard Arduino setup-function with all
 *          initializations
 *
 * \param   -
 * \return  -
 ============================================================== */
void setup()
{
//  while(!Serial);
  Serial.begin(BAUDRATE);
  delay(1000);
  Serial.println("-------------------------");
  Serial.println("sensor test MotionTracker_V02");
  
  /* initialize pins */
  pinMode(INT1_XM, INPUT);
  pinMode(INT2_XM, INPUT);
  pinMode(INT_G, INPUT);
  pinMode(DRDY_G, INPUT);
  pinMode(WP, INPUT);
  pinMode(HOLD, INPUT);
  pinMode(BUTTON, INPUT);
  digitalWrite(BUTTON, HIGH);  // enable internal pullup
  
  pinMode(CS_F, OUTPUT);
  pinMode(CS_XM, OUTPUT);
  pinMode(CS_G, OUTPUT);
  pinMode(EN_G, OUTPUT);
  pinMode(RED, OUTPUT);
  pinMode(BLUE, OUTPUT);
  pinMode(GREEN, OUTPUT);
  digitalWrite(RED, HIGH);	// turn off LED
  digitalWrite(BLUE, HIGH); 	// turn off LED
  digitalWrite(GREEN, HIGH);	// turn off LED
  
  /* initialize unused pins to reduce current consumption */
  pinMode(UU_0, INPUT);
  pinMode(UU_1, INPUT);
  pinMode(UU_2, INPUT);
  pinMode(UU_3, INPUT);
  digitalWrite(UU_0, HIGH);    // enable internal pullup
  digitalWrite(UU_1, HIGH);    // enable internal pullup
  digitalWrite(UU_2, HIGH);    // enable internal pullup
  digitalWrite(UU_3, HIGH);    // enable internal pullup
  
  /* Initialise the sensor */
  if(!lsm.begin())
  {
    /* There was a problem detecting the LSM9DS0 ... check your connections */
    Serial.print(F("Ooops, no LSM9DS0 detected ... Check your wiring!"));
    digitalWrite(RED, LOW);
    while (FOREVER);
  }
  Serial.println(F("Found LSM9DS0 9DOF"));
  digitalWrite(GREEN,LOW);
  
  /* Display some basic information on this sensor */
  displaySensorDetails();
  
  /* Setup the sensor gain and integration time */
  configureSensor();
  
  /* We're ready to go! */
  Serial.println("");
}

/** ===========================================================
 * \fn      loop
 * \brief   standard Arduino forever loop-function with the
 *          state machine
 *
 * \param   -
 * \return  -
 ============================================================== */
void loop()
{ 
  /* Get a new sensor event */ 
  sensors_event_t accel, mag, gyro, temp;

  lsm.getEvent(&accel, &mag, &gyro, &temp); 

  // print out accelleration data
  Serial.print("Accel X: "); Serial.print(accel.acceleration.x); Serial.print(" ");
  Serial.print("  \tY: "); Serial.print(accel.acceleration.y); Serial.print(" ");
  Serial.print("  \tZ: "); Serial.print(accel.acceleration.z); Serial.println("  \tm/s^2");

  // print out magnetometer data
  Serial.print("Magn. X: "); Serial.print(mag.magnetic.x); Serial.print(" ");
  Serial.print("  \tY: "); Serial.print(mag.magnetic.y); Serial.print(" ");
  Serial.print("  \tZ: "); Serial.print(mag.magnetic.z); Serial.println("  \tgauss");
  
  // print out gyroscopic data
  Serial.print("Gyro  X: "); Serial.print(gyro.gyro.x); Serial.print(" ");
  Serial.print("  \tY: "); Serial.print(gyro.gyro.y); Serial.print(" ");
  Serial.print("  \tZ: "); Serial.print(gyro.gyro.z); Serial.println("  \tdps");

  // print out temperature data
  Serial.print("Temp: "); Serial.print(temp.temperature); Serial.println(" *C");

  Serial.println("**********************\n");

  delay(250);
}

/** ===========================================================
 * \fn      displaySensorDetails
 * \brief   displays some basic information on this sensor from
 *          the unified sensor API sensor_t type (see
 *          Adafruit_Sensor for more information)
 *
 * \param   -
 * \return  -
 ============================================================== */
void displaySensorDetails(void)
{
  sensor_t accel, mag, gyro, temp;
  
  lsm.getSensor(&accel, &mag, &gyro, &temp);
  
  Serial.println(F("------------------------------------"));
  Serial.print  (F("Sensor:       ")); Serial.println(accel.name);
  Serial.print  (F("Driver Ver:   ")); Serial.println(accel.version);
  Serial.print  (F("Unique ID:    ")); Serial.println(accel.sensor_id);
  Serial.print  (F("Max Value:    ")); Serial.print(accel.max_value); Serial.println(F(" m/s^2"));
  Serial.print  (F("Min Value:    ")); Serial.print(accel.min_value); Serial.println(F(" m/s^2"));
  Serial.print  (F("Resolution:   ")); Serial.print(accel.resolution); Serial.println(F(" m/s^2"));  
  Serial.println(F("------------------------------------"));
  Serial.println(F(""));

  Serial.println(F("------------------------------------"));
  Serial.print  (F("Sensor:       ")); Serial.println(mag.name);
  Serial.print  (F("Driver Ver:   ")); Serial.println(mag.version);
  Serial.print  (F("Unique ID:    ")); Serial.println(mag.sensor_id);
  Serial.print  (F("Max Value:    ")); Serial.print(mag.max_value); Serial.println(F(" uT"));
  Serial.print  (F("Min Value:    ")); Serial.print(mag.min_value); Serial.println(F(" uT"));
  Serial.print  (F("Resolution:   ")); Serial.print(mag.resolution); Serial.println(F(" uT"));  
  Serial.println(F("------------------------------------"));
  Serial.println(F(""));

  Serial.println(F("------------------------------------"));
  Serial.print  (F("Sensor:       ")); Serial.println(gyro.name);
  Serial.print  (F("Driver Ver:   ")); Serial.println(gyro.version);
  Serial.print  (F("Unique ID:    ")); Serial.println(gyro.sensor_id);
  Serial.print  (F("Max Value:    ")); Serial.print(gyro.max_value); Serial.println(F(" rad/s"));
  Serial.print  (F("Min Value:    ")); Serial.print(gyro.min_value); Serial.println(F(" rad/s"));
  Serial.print  (F("Resolution:   ")); Serial.print(gyro.resolution); Serial.println(F(" rad/s"));  
  Serial.println(F("------------------------------------"));
  Serial.println(F(""));

  Serial.println(F("------------------------------------"));
  Serial.print  (F("Sensor:       ")); Serial.println(temp.name);
  Serial.print  (F("Driver Ver:   ")); Serial.println(temp.version);
  Serial.print  (F("Unique ID:    ")); Serial.println(temp.sensor_id);
  Serial.print  (F("Max Value:    ")); Serial.print(temp.max_value); Serial.println(F(" C"));
  Serial.print  (F("Min Value:    ")); Serial.print(temp.min_value); Serial.println(F(" C"));
  Serial.print  (F("Resolution:   ")); Serial.print(temp.resolution); Serial.println(F(" C"));  
  Serial.println(F("------------------------------------"));
  Serial.println(F(""));
  
  delay(500);
}

/** ===========================================================
 * \fn      configureSensor
 * \brief   configures the gain and integration time for the
 *          TSL2561
 *
 * \param   -
 * \return  -
 ============================================================== */
void configureSensor(void)
{
  // 1.) Set the accelerometer range
  lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_2G);
  //lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_4G);
  //lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_6G);
  //lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_8G);
  //lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_16G);
  
  // 2.) Set the magnetometer sensitivity
  lsm.setupMag(lsm.LSM9DS0_MAGGAIN_2GAUSS);
  //lsm.setupMag(lsm.LSM9DS0_MAGGAIN_4GAUSS);
  //lsm.setupMag(lsm.LSM9DS0_MAGGAIN_8GAUSS);
  //lsm.setupMag(lsm.LSM9DS0_MAGGAIN_12GAUSS);

  // 3.) Setup the gyroscope
  lsm.setupGyro(lsm.LSM9DS0_GYROSCALE_245DPS);
  //lsm.setupGyro(lsm.LSM9DS0_GYROSCALE_500DPS);
  //lsm.setupGyro(lsm.LSM9DS0_GYROSCALE_2000DPS);
}
