/** ===========================================================
 *  MotionTracker V01
 *
 * \file       MotionTracker_V01_0
 * \author     jh
 * \date       xx.04.2015  
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
/* buttons */
#define MAX_DURATION_SHORTCLICK       1000    // max. duration in ms a SHORTCLICK will be recognized
#define MIN_DURATION_LONGCLICK        2000    // min. duration in ms a LONGCLICK will be recognized

/* communication */
#define BAUDRATE                    115200    // e.g. 9600 or 19200 or 57600 or 115200
#define KNOCK_DELAY		       200    // delay between knocks
#define HANDSHAKE_MESSAGE        ("knock")    // handshacke key

/* sensor defines */
#define NUMBER_OF_DIFFERENT_DATA        11    // t, ax, ay, az, Bx, By, Bz, wx, wy, wz, T (in this order!)
//#define MEAN                            10    // number of data to summarize before saving
//#define MEDIAN                        true    // comment out to use MEAN calculation -> uses a lot mor RAM!

/* timer interrupt */               // 78 -> 10ms, 117 -> 15ms, 156 -> 20ms
#define INTERRUPT_FREQUENCY_DIVISOR_1  156//117//78    // t_isr = 1/(8000000/(prescaler*VALUE))

/* debugging */
#define DEBUGGING                     true    // (must be commented!) 

/* Includes --------------------------------------------------- */
#include <avr/sleep.h>
#include <avr/interrupt.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM9DS0.h>

/* PIN definitions -------------------------------------------- */
// ATMEL ATMEGA32U2 / Minimus 32 => MotionTracker_V01
//
// Pins numbered counter-clockwise starting from USB connector
//
// D0		PC2		EN_G
// D1#		PD0		INT1_XM
// D2		PD1		INT2_XM
// D3		PD2		RXD
// D4		PD3		TXD
// D5		PD4		INT_G
// D6		PD5		DRDY_G
// D7		PD6		CS_F
// D8		PD7		CS_XM (HWB)
// D9		PB0		CS_G
// D10		PB1		SCK
// [D11]			
// D12		PB2		MOSI
// D13		PB3		MISO
// D14		PB4		STATE
// D15		PB5		RED
// D16		PB6		BLUE
// D17#		PB7		GREEN
// D18		PC7		BUTTON
// D19#		PC6		-
// [D20]			
// D21#		PC5		HOLD
// D22		PC4		WP

/* MotionTracker_V01 -> Minimus 32 */
#define BUTTON    18

#define RED       15
#define GREEN     17
#define BLUE      16

#define STATE     14

#define MOSI      12 
#define SCK       10
#define MISO      13
#define CS_XM      8
#define CS_G       9
#define INT2_XM    2
#define INT1_XM    1

#define HOLD      21
#define WP        22
#define CS_F       7
#define DRDY_G     6
#define INT_G      5
#define EN_G       0

#define RXD        3
#define TXD        4

/* MotionTracker_V01 -> MattairTech */
// #define BUTTON     8

// #define RED        5
// #define GREEN      7
// #define BLUE       6

// #define STATE      4

// #define MOSI       2 
// #define SCK        1
// #define MISO       3
// #define CS_XM     20
// #define CS_G       0
// #define INT2_XM   14
// #define INT1_XM   13

// #define HOLD      10
// #define WP        11
// #define CS_F      19
// #define DRDY_G    18
// #define INT_G     17
// #define EN_G      12

// #define RXD       15
// #define TXD       16

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

///* State machine ---------------------------------------------- */
///* states */
//#define OFF              0
//#define RDY              1
//#define READ             2
//#define FULL             3
//#define WRITE            4
//
///* events */
//#define NO_EVENT         0
//#define SHORTCLICK       1
//#define LONGCLICK        2
//#define DOUBLECLICK      3
//#define MEMORY_FULL      4

/* General defines -------------------------------------------- */
#define FOREVER      true
#define UP           true
#define DOWN         false

///* Global variables ------------------------------------------- */
///* default state machine */
//byte state = OFF;  // first state
//byte event = NO_EVENT;

/* timing reference variables */
//unsigned long startMillis = millis();
//unsigned long startMicros = micros();
unsigned long us, us_ref;
//unsigned long us_ref = micros();

/* using HW SPI to communicate with the sensor */
Adafruit_LSM9DS0 lsm = Adafruit_LSM9DS0(CS_XM, CS_G, 1000);

/* sensor event variables */
sensors_event_t accel, mag, gyro, temp;

/* timer interrupt counter (have to be defined as volatile!) */
volatile unsigned int timerInterruptCounter = 0;

/* sensor data array */
#ifdef MEDIAN
float dataArray2D[NUMBER_OF_DIFFERENT_DATA][MEAN];  // 440 bytes
#endif
#ifndef MEDIAN
float dataArray[NUMBER_OF_DIFFERENT_DATA];  // 44 bytes
#endif
byte meanCntr = 0;

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
  Serial.begin(BAUDRATE);
  #ifdef DEBUGGING
  while(!Serial);
  #endif
  #ifndef DEBUGGING
  delay(2000);
  #endif
  Serial.println("-------------------------");
  Serial.println("start setup");
    
  /* enable all interrupts */
  sei();

  /* initialize pins */
  pinMode(INT1_XM, INPUT);
  pinMode(INT2_XM, INPUT);
  pinMode(INT_G, INPUT);
  pinMode(DRDY_G, INPUT);
  pinMode(STATE, INPUT);
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
  
  /* send something (serial data) to establish contact until receiver responds */
  establishContact();
  delay(1000);
  
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
  
//  /* initialize timer interrupt for the sensor */
//  initTimerInterrupt_CTC_1();
  
  /* Display some basic information on this sensor */
  displaySensorDetails();
  
  /* Setup the sensor gain and integration time */
  configureSensor();
  
  
  /* disable unused cpu-functions to reduce power consumption: */
  /* disable ACD and ADC */
  //  ACSR |= (1 << ACD);       // set bit 7    -> ~20uA less
  //  ADCSRA &= ~(1 << ADEN);   // clear bit 7  -> ~240uA less

  //  /* disable WDT */
  //  WDTCR |= 0x18;         // set bit 4 and 3
  //  WDTCR &= ~(1 << 3);    // clear bit 3

  //  /* set Power Reduction Register (page 42&43) */
  //  PRR0 = ;
  //  PRR1 = ;
  
  
  Serial.println("end setup");
  Serial.println("start loop...");
  
  /* set first timing ref */
  us_ref = micros();
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
  /* get sensor data */
  lsm.getEvent(&accel, &mag, &gyro, &temp);
  
  /* wait remaining 10ms */
  while ((micros() - us_ref) < 10000);   // wait remaining 10ms
  unsigned int dt = micros() - us_ref;   // calculate dt
  us_ref = micros();                     // set new timing reference
  
  /* collect data */
  #ifdef MEDIAN  // use median
  {
    dataArray2D[0][meanCntr]  = dt;
    dataArray2D[1][meanCntr]  = accel.acceleration.x;
    dataArray2D[2][meanCntr]  = accel.acceleration.y;
    dataArray2D[3][meanCntr]  = accel.acceleration.z;
    dataArray2D[4][meanCntr]  = mag.magnetic.x;
    dataArray2D[5][meanCntr]  = mag.magnetic.y;
    dataArray2D[6][meanCntr]  = mag.magnetic.z;
    dataArray2D[7][meanCntr]  = gyro.gyro.x;
    dataArray2D[8][meanCntr]  = gyro.gyro.y;
    dataArray2D[9][meanCntr]  = gyro.gyro.z;
    dataArray2D[10][meanCntr] = temp.temperature;
    
    meanCntr++;
    
    if (meanCntr >= MEAN)
    {
      meanCntr = 0;
      
      /* sort arrays */
      for (byte i = 1; i < NUMBER_OF_DIFFERENT_DATA; i++) qsort(dataArray2D[i], MEAN, sizeof(float), cmpfunc);
  
      /* print timestamp in us */
      Serial.print(micros()); Serial.print(", ");                      // t
  
      /* print delta time in us */
      Serial.print(dataArray2D[0][MEAN-1]); Serial.print(", ");        // dt
          
      /* print acceleration data in m/s^2 */
      Serial.print(dataArray2D[1][MEAN/2]); Serial.print(", ");        // ax
      Serial.print(dataArray2D[2][MEAN/2]); Serial.print(", ");        // ay
      Serial.print(dataArray2D[3][MEAN/2]); Serial.print(", ");        // az
      
      /* print magnetometer data in Gs */
      Serial.print(dataArray2D[4][MEAN/2]); Serial.print(", ");        // Bx
      Serial.print(dataArray2D[5][MEAN/2]); Serial.print(", ");        // By
      Serial.print(dataArray2D[6][MEAN/2]); Serial.print(", ");        // Bz
      
      /* print gyroscop data in °/s */
      Serial.print(dataArray2D[7][MEAN/2]); Serial.print(", ");        // wx
      Serial.print(dataArray2D[8][MEAN/2]); Serial.print(", ");        // wy
      Serial.print(dataArray2D[9][MEAN/2]); Serial.print(", ");        // wz
      
      /* print temp data in °C */
      Serial.println(dataArray2D[10][MEAN/2]);                         // T
    }
  }
  #endif
  #ifndef MEDIAN
  #ifdef MEAN  // use mean
  {
    dataArray[0]  += dt;
    dataArray[1]  += accel.acceleration.x;
    dataArray[2]  += accel.acceleration.y;
    dataArray[3]  += accel.acceleration.z;
    dataArray[4]  += mag.magnetic.x;
    dataArray[5]  += mag.magnetic.y;
    dataArray[6]  += mag.magnetic.z;
    dataArray[7]  += gyro.gyro.x;
    dataArray[8]  += gyro.gyro.y;
    dataArray[9]  += gyro.gyro.z;
    dataArray[10] += temp.temperature;
    
    meanCntr++;
    
    if (meanCntr >= MEAN)
    {
      meanCntr = 0;
      
      /* divide data by MEAN */
      for (byte i = 0; i < NUMBER_OF_DIFFERENT_DATA; i++) dataArray[i] /= MEAN;
  
//      /* print timestamp in us */
//      Serial.print(micros()); Serial.print(", ");            // t
  
      /* print delta time in us */
      Serial.print(dataArray[0] * MEAN); Serial.print(", "); // dt
          
      /* print acceleration data in m/s^2 */
      Serial.print(dataArray[1]); Serial.print(", ");        // ax
      Serial.print(dataArray[2]); Serial.print(", ");        // ay
      Serial.print(dataArray[3]); Serial.print(", ");        // az
      
      /* print magnetometer data in Gs */
      Serial.print(dataArray[4]); Serial.print(", ");        // Bx
      Serial.print(dataArray[5]); Serial.print(", ");        // By
      Serial.print(dataArray[6]); Serial.print(", ");        // Bz
      
      /* print gyroscop data in °/s */
      Serial.print(dataArray[7]); Serial.print(", ");        // wx
      Serial.print(dataArray[8]); Serial.print(", ");        // wy
      Serial.print(dataArray[9]); Serial.print(", ");        // wz
      
      /* print temp data in °C */
      Serial.println(dataArray[10]);                         // T
      
      /* reset data array */
      for (byte i = 0; i < NUMBER_OF_DIFFERENT_DATA; i++) dataArray[i] = 0;
    }
  }
  #endif
  #endif
  #ifndef MEDIAN  // use neither median nor mean
  {
    dataArray[0]  = dt;
    dataArray[1]  = accel.acceleration.x;
    dataArray[2]  = accel.acceleration.y;
    dataArray[3]  = accel.acceleration.z;
    dataArray[4]  = mag.magnetic.x;
    dataArray[5]  = mag.magnetic.y;
    dataArray[6]  = mag.magnetic.z;
    dataArray[7]  = gyro.gyro.x;
    dataArray[8]  = gyro.gyro.y;
    dataArray[9]  = gyro.gyro.z;
    dataArray[10] = temp.temperature;
    
//    /* print timestamp in us */
//    Serial.print(micros()); Serial.print(", ");            // t
  
    /* print delta time in us */
    Serial.print(dataArray[0]); Serial.print(", ");        // dt
        
    /* print acceleration data in m/s^2 */
    Serial.print(dataArray[1]); Serial.print(", ");        // ax
    Serial.print(dataArray[2]); Serial.print(", ");        // ay
    Serial.print(dataArray[3]); Serial.print(", ");        // az
    
    /* print magnetometer data in Gs */
    Serial.print(dataArray[4]); Serial.print(", ");        // Bx
    Serial.print(dataArray[5]); Serial.print(", ");        // By
    Serial.print(dataArray[6]); Serial.print(", ");        // Bz
    
    /* print gyroscop data in °/s */
    Serial.print(dataArray[7]); Serial.print(", ");        // wx
    Serial.print(dataArray[8]); Serial.print(", ");        // wy
    Serial.print(dataArray[9]); Serial.print(", ");        // wz
    
    /* print temp data in °C */
    Serial.println(dataArray[10]);                         // T
  }
  #endif  

  
  
  
  
  
//  Serial.print(millis()); Serial.print(", ");                // millis
//  Serial.print(dt); Serial.print(", ");                      // dt
//  
//  cli();  // disable intterupts to prevent wrong transmissions
//  
//  /* print acceleration data in m/s^2 */
//  Serial.print(accel.acceleration.x); Serial.print(", ");    // ax
//  Serial.print(accel.acceleration.y); Serial.print(", ");    // ay
//  Serial.print(accel.acceleration.z); Serial.print(", ");    // az
//  
////  /* print heading in ° */
////  float heading = 180 * atan2(mag.magnetic.z, mag.magnetic.y)/PI;
////  if (heading < 0) heading += 360;
////  Serial.print(heading); Serial.print(", ");                 // heading
////  
////  /* print gyroscop data in ° */
////  float Gdt = (float)dt / 1000000;
////  Serial.print(gyro.gyro.x * Gdt); Serial.print(", ");       // Gx
////  Serial.print(gyro.gyro.y * Gdt); Serial.print(", ");       // Gy
////  Serial.println(gyro.gyro.z * Gdt);                         // Gz
//  
//  /* print magnetometer data in Gs */
//  Serial.print(mag.magnetic.x); Serial.print(", ");          // Bx
//  Serial.print(mag.magnetic.y); Serial.print(", ");          // By
//  Serial.print(mag.magnetic.z); Serial.print(", ");          // Bz
//  
//  /* print gyroscop data in °/s */
//  Serial.print(gyro.gyro.x); Serial.print(", ");             // wx
//  Serial.print(gyro.gyro.y); Serial.print(", ");             // wy
//  Serial.println(gyro.gyro.z);                               // wz
//  
//  sei();
}

int cmpfunc (const void * a, const void * b)
{
  return ( *(int*)a - *(int*)b );
}

///** ===========================================================
// * \fn      ISR (timer1 OCIE1A interrupt TIMER1_COMPA)
// * \brief   interrupt service routine (handler) which will be 
// *          called every few milli seconds to control the
// *          sensor reading
// *          (f_isr = fcpu / (prescaler * cnt))
// *
// * \param   'TIMER1_COMPA vector'
// * \return  -
// ============================================================== */
//ISR(TIMER1_COMPA_vect)
//{
//  /* test interrupt */
//  PORTD ^= (1 << 2);      // toggle PD2 pin
//  
//  /* increase timer interrupt counter */
//  timerInterruptCounter++;
//  
//  /* get a new sensor event */
//  //lsm.getEvent(&accel, &mag, &gyro, &temp);
//}

/** ===========================================================
 * \fn      establishContact
 * \brief   sends given handshake String continously until
 *          receiver responds with something (it doesn't matter
 *          what). Flushs the receive buffer afterwards
 *
 * \param   -
 * \return  -
 ============================================================== */
void establishContact()
{
  while (Serial.available() <= 0)
  {
    Serial.println(HANDSHAKE_MESSAGE);
    delay(KNOCK_DELAY);
  }
  
  /* flush the receive buffer  */
  while (Serial.available()) Serial.read();
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
  
  delay(2000);
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



// /* get current state machine event */
// event = getButtonEvent(BUTTON, MAX_DURATION_SHORTCLICK, MIN_DURATION_LONGCLICK);
//// if (/*speicher voll*/) event = MEMORY_FULL;
//
// /* state machine -------------------------------------------- */
// switch (state)
// {
//   /* STATE 1 .................................................. */
// case OFF:
//   switch (event)
//   {
//   case NO_EVENT:
//     /* */
//     break;
//
//   case SHORTCLICK:
//     /* */
//     break;
//
//   case LONGCLICK:
//     /* */
//     break;
//
//   default: 
//     /* */
//     ;
//   }
//   break;
//
//   /* STATE 2 .................................................. */
// case RDY:
//   switch (event)
//   {
//   case NO_EVENT:
//     /* */
//     break;
//
//   case SHORTCLICK:
//     /* */
//     break;
//
//   case LONGCLICK:
//     /* */
//     break;
//
//   default: 
//     /* */
//     ;
//   }
//   break;
//
//   /* STATE 3 .................................................. */
// case READ:
//   switch (event)
//   {
//   case NO_EVENT:
//     /* */
//     break;
//
//   case SHORTCLICK:
//     /* */
//     break;
//
//   case LONGCLICK:
//     /* */
//     break;
//
//   default: 
//     /* */
//     ;
//   }
//   break;
// }
// /* end of state machine ------------------------------------- */
//}



///** ===========================================================
// * \fn      getButtonEvent
// * \brief   returns given values on base of current button state
// *          (contains a while-loop as long as button is pushed)
// *
// * \param   (byte) defined button pin
// *          (uint) max. duration in ms a short click will be
// *                 recognized
// *          (uint) min. duration in ms a long click will be
// *                 recognized
// * \return  (byte) 0 = no event
// *                 1 = short button click
// *                 2 = long button click
// ============================================================== */
//byte getButtonEvent(byte buttonPin, unsigned int max_duration_shortclick, unsigned int min_duration_longclick)
//{
//  byte buttonEvent = 0;
//
//  /* check if pressed */
//  if (!digitalRead(buttonPin))
//  {
//    /* get number of ms since the program runs as reference */
//    unsigned long ref = millis();
//
//    /* wait as long as the button is pressed */
//    while (!digitalRead(buttonPin));
//
//    /* calculate past time (since the start variable has been written) and check if it was a short click */
//    if ((millis() - ref) < max_duration_shortclick)
//    {
//      /* short click: */
//      buttonEvent = 1;  // set current event
//    }
//    else 
//    {
//      /* long click: */
//      buttonEvent = 2;  // set current event
//    }
//  }
//  
//  /* no button event if not pressed */
//
//  return buttonEvent;
//}

///** ===========================================================
// * \fn      turnOff
// * \brief   turns off the device by putting it in a power down
// *          sleep-mode, it will wake up again if INTN occures
// *
// * \requ    <avr/interrupt.h> and <avr/sleep.h>
// *
// * \param   -
// * \return  -
// ============================================================== */
//void turnOff()
//{
//  /* disable peirphery */
//  displaySleep(true);
//
//  /* all interrupts have to be disabled during turn off configurations */
//  cli();
//
//  /* define that a low-level of INT2 and 3 generates an interrupt request */
//  EICRA &= ~(1 << ISC21) & ~(1 << ISC20);  // clear bit 5 and 4 to choose low-level-trigger
//  EICRA &= ~(1 << ISC31) & ~(1 << ISC30);  // clear bit 7 and 6 to choose low-level-trigger
////  EICRB &= ~(1 << ISC61) & ~(1 << ISC60);  // clear bit 5 and 4 to choose low-level-trigger
//
//  /* clear interrupt flag */
//  EIFR |= (1 << INTF2) | (1 << INTF3);       //set bit 2 and 3
//
//  /* enable external interrupt INT2 and 3 (to wake up later) */
//  EIMSK |= (1 << INT2) | (1 << INT3);
//
//  /* choose "Power-down" sleep mode */
//  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
//  //  SMCR |= (1 << SM1);    // set bit 2
//
//  /* enable sleep-mode */
//  sleep_enable();         
//  //  SMCR |= (1 << SE);     // set bit 0
//
//  /* reenable interrupts */
//  sei();
//
//  /* system actually sleeps here */
//  sleep_cpu();
//
//  /* zzzZZZzzzZZZ... */
//
//  /* INT2 or 3 (BUTTON 1 & 2) occured -> wakes up! system continues here... */
//  
//  /* enable peirphery */
//  displaySleep(false);
//}

///** ===========================================================
// * \fn      ISR (external interrupt INT2)
// * \brief   interrupt service routine (handler) for the wake-up-
// *          interrupt (INT2)
// *
// * \requ    <avr/sleep.h>
// *
// * \param   'INT2 vector'
// * \return  -
// ============================================================== */
//ISR(INT2_vect)
//{
//  /* disable sleep-mode */
//  sleep_disable();
//  //  SMCR &= ~(1 << SM1)      // reset bit 0
//
//  /* disable external interrupts */
//  EIMSK &= ~(1 << INT2) & ~(1 << INT3) & ~(1 << INT6);
//}

///** ===========================================================
// * \fn      ISR (external interrupt INT3)
// * \brief   interrupt service routine (handler) for the wake-up-
// *          interrupt (INT3)
// *
// * \requ    <avr/sleep.h>
// *
// * \param   'INT3 vector'
// * \return  -
// ============================================================== */
//ISR(INT3_vect)
//{
//  /* disable sleep-mode */
//  sleep_disable();
//  //  SMCR &= ~(1 << SM1)      // reset bit 0
//
//  /* disable external interrupts */
//  EIMSK &= ~(1 << INT2) & ~(1 << INT3) & ~(1 << INT6);
//}

///** ===========================================================
// * \fn      ISR (external interrupt INT6)
// * \brief   interrupt service routine (handler) for the wake-up-
// *          interrupt (INT6)
// *
// * \requ    <avr/sleep.h>
// *
// * \param   'INT6 vector'
// * \return  -
// ============================================================== */
//ISR(INT6_vect)
//{  
//  /* disable sleep-mode */
//  sleep_disable();
//  //  SMCR &= ~(1 << SM1)      // reset bit 0
//
//  /* disable external interrupts */
//  EIMSK &= ~(1 << INT2) & ~(1 << INT3) & ~(1 << INT6);
//}

///** ===========================================================
// * \fn      writeByte
// * \brief   writes a non-specific byte via SPI to the OLED
// *          controller
// *
// * \param   (byte) byte value
// * \return  -
// ============================================================== */
//void writeByte(byte byteValue)
//{
//  /* HW SPI (~250ns) */
//  clearBit(PORTB, 0);      // reset chip select pin PB0 (OLED display enabled)
//  SPI.transfer(byteValue);
//  setBit(PORTB, 0);        // set chip select pin PB0 (OLED display disabled)
//}

///** ===========================================================
// * \fn      waitRemainingMillisConditionally
// * \brief   wait given remaining duration but leaves immediately
// *          when given digital input pin becomes 0 (false) 
// *          returns 1 (true) if that was the case
// *
// * \requ    global variable: unsigned long startMillis = millis();
// *
// * \param   (ulong) duration to wait in ms
// *          (byte)  digital input pin to leave the while-loop
// * \return  (bool)  true if while-loop has been left
// ============================================================== */
//boolean waitRemainingMillisConditionally(unsigned long ms, byte inputPin)
//{
//  while ((millis() - startMillis) < ms && digitalRead(inputPin)); // low active!
//
//  startMillis = millis();
//  if (!digitalRead(inputPin)) return true;
//  else return false;
//}
//
///** ===========================================================
// * \fn      waitRemainingMillis
// * \brief   wait given remaining duration in milli seconds
// *
// * \requ    global variable: unsigned long startMillis = millis();
// *
// * \param   (ulong) time to wait in milli seconds
// * \return  -
// ============================================================== */
//void waitRemainingMillis(unsigned long ms)
//{
//  while ((millis() - startMillis) < ms);
//
//  startMillis = millis();
//}
//
///** ===========================================================
// * \fn      waitRemainingMicros
// * \brief   wait given remaining duration in micro seconds
// *
// * \requ    global variable: unsigned long startMicros = micros();
// *
// * \param   (ulong) time to wait in micro seconds
// * \return  -
// ============================================================== */
//void waitRemainingMicros(unsigned long us)
//{
//  while ((micros() - startMicros) < us);
//
//  startMicros = micros();
//}
//
///** ===========================================================
// * \fn      waitConditionally
// * \brief   wait given duration but leaves immediately when
// *          given digital input pin becomes 0 (false) 
// *          returns 1 (true) if that was the case
// *
// * \param   (ulong) duration to wait in ms
// *          (byte)  digital input pin to leave the while-loop
// * \return  (bool)  true if while-loop was left
// ============================================================== */
//boolean waitConditionally(unsigned long duration, byte inputPin)
//{
//  unsigned long ref = millis();
//  while ((millis() - ref) < duration && digitalRead(inputPin)); // low active!
//
//  if (!digitalRead(inputPin)) return true;
//  else return false;
//}
//
///** ===========================================================
// * \fn      ISR (timer1 OCIE1A interrupt TIMER1_COMPA)
// * \brief   interrupt service routine (handler) which will be 
// *          called every few milli seconds to control the
// *          sensor reading
// *          (f_isr = fcpu / (prescaler * cnt))
// *
// * \param   'TIMER1_COMPA vector'
// * \return  -
// ============================================================== */
//ISR(TIMER1_COMPA_vect)
//{
////  /* test interrupt */
////  PORTD ^= (1 << 2);      // toggle PD2 pin
//  
////  cli();
//  /* increase global timer interrupt counter */
//  timerInterruptCounterPiezo++; 
//      
//  /* check if it is time to handle the piezo */
//  if (timerInterruptCounterPiezo >=  piezoFrequencyDivisor)
//  {
//    /* reset global piezo timer interrupt counter */
//    timerInterruptCounterPiezo = 0;
//  
//    /* toggle the piezo on base of current desired state */
//    if (piezoOn) PORTC ^= (1 << 7);  // PIEZO = PC7
//  }
////  sei();
//}
//
///** ===========================================================
// * \fn      initTimerInterrupt_CTC_0
// * \brief   initializes the timer 0 and releated Clear Timer on
// *          Compare match (CTC) interrupt
// *          (!Attention: Arduino uses timer 0 for delay-
// *          functions by the ATmegas)
// *
// * \param   -
// * \return  -
// ============================================================== */
//void initTimerInterrupt_CTC_0()
//{
//  /* init timer0 and releated interrupt (t_isr = 1/(8MHz/(prescaler*cnt))) */
//  TCCR0A = (1 << WGM01);   // enable CTC (Clear Timer on Compare match) mode
//  TCCR0B = (1 << CS02) | (1 << CS00);  // prescaler (CS00 = 1, CS01 = 8, CS00&CS01 = 64, CS02 = 256, CS00&CS02 = 1024)
//  
//  TIMSK0 = (1 << OCIE0A);  // set bit 1 to enable timer0 output compare match A interrupt (TIMER0_COMPA)
//  OCR0A = INTERRUPT_FREQUENCY_DIVISOR_0;  // define output compare register A value (0... 255)
//
//  /* reset timer 0 counter register */
//  TCNT0 = 0x00;
//}

/** ===========================================================
 * \fn      initTimerInterrupt_CTC_1
 * \brief   initializes the timer 1 and releated Clear Timer on
 *          Compare match (CTC) interrupt
 *          (ATmega32U2)
 *
 * \param   -
 * \return  -
 ============================================================== */
void initTimerInterrupt_CTC_1()
{
  /* init timer1 and releated interrupt (t_isr = 1/(8MHz/(prescaler*cnt))) */
  TCCR1A = 0x00;         // enable CTC (Clear Timer on Compare match) mode
  TCCR1B = ((1 << WGM12) | (1 << CS10) | (1 << CS12));  // prescaler = 1024
  
  TIMSK1 = (1 << OCIE1A);     // set bit 1 to enable timer1 output compare match A interrupt (TIMER1_COMPA)
  OCR1A = INTERRUPT_FREQUENCY_DIVISOR_1;  // define output compare register A value (0... 65535)

  /* reset timer 1 counter register */
  TCNT1 = 0x00;
}

/**
 * @}
 */
