#include <virtuabotixRTC.h>
#include "MPU6050_6Axis_MotionApps20.h"
#include <I2Cdev.h>
#include <Wire.h>
#include <MPU6050.h>
#include <LiquidCrystal.h>

//RTC code provided by virtuabotix 
//MPU6050 code provided by Jeff Rowberg
//https://www.virtuabotix.com/virtuabotix-ds1302-real-time-clock-module-pin-out-coding-guide/
//https://github.com/jrowberg/i2cdevlib/tree/master/Arduino/MPU6050


//RTC variables ---------------------------------------------------------
//RTC set (CLK, DAT, RST)
virtuabotixRTC my_RTC(A0, A1, A2); 
double month, year, day, minutes, hours, seconds;
double A, B;
double longitude = -117.313261;
double latitude = 33.975447;
double J2000;
double dec_time;
double LST; 
double LST_degrees;
double LST_hours;

//-----------------------------------------------------------------------

//LCD variables 
LiquidCrystal lcd(7, 8, 9, 10, 11, 12);
uint8_t data;
char* names[] = {"Orion", "Cassiopia", "Big Dipper", "Cygnus", "Perseus", "Sagittarius"};
double star_yaw[] = {84.0542, 15, 165, 315, 45, 285};
double star_roll[] = {-1.2019, 60, 50, 40, 45, -25};
uint8_t pos = 0;

//mpu variables ---------------------------------------------------------
MPU6050 mpu;
#define LED_PIN 13
bool blinkState = false;

//MPU control_status variables 
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorFloat gravity;    // [x, y, z]            gravity vector
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

//interrupt detection routine
volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}
//-----------------------------------------------------------------------

//general variables ---------------------------------------------------------
//test user yaw and roll point to orion
double user_yaw = 0; //yaw wanted by user 
double user_roll = 90; //roll wanted by user
double real_yaw; //real yaw needed to point at sky
float yaw_fix = 0; // subtract from yaw_current to get 0 degrees 
bool yaw_flag = false; // used to set yaw fix
float yaw_current = 0; //yaw used to hold ypr[0]
double yaw_calibrate = 0;
bool gryo_calibrate = false; 
double LST_floor;
double value = 0;

uint8_t yaw_send;
uint8_t roll_send = 90;
uint8_t prev_yaw_send;
uint8_t prev_roll_send = 0;
double roll_fix;
double yaw_off_degrees; //used to offset yaw by a couple degrees 
double yaw_current_off_degrees;
bool once = false;
float temp_yaw; //used to temporarily hold yaw to change float to double

//-----------------------------------------------------------------------

void setup() {
  //set RTC (seconds, minutes, hours, day of week, day of month, month, year) 
  my_RTC.setDS1302Time(0, 00, 20, 4, 30, 11, 2016);
  my_RTC.updateTime();
  get_LST();
  Serial.begin(9600);
  real_yaw = user_yaw + floor(LST_degrees); 
  if (real_yaw > 360) {
    real_yaw = real_yaw - 360;
  }
  mpu_init();
  lcd.begin(16, 2);
  lcd.print("Press Start to");
  lcd.setCursor(0, 1);
  lcd.print("move manually");
  delay(15000);
}

void loop() {
  my_RTC.updateTime();
  get_LST();
  //this will update the local sidereal time 1 degree at a time 
  real_yaw = user_yaw - round(LST_degrees); 
  if (real_yaw < 0) {
    real_yaw = real_yaw + 360;
  }
//
  yaw_off_degrees = round(real_yaw);
  temp_yaw = round(yaw_current);
  yaw_current_off_degrees = (double)temp_yaw;

  if (yaw_current_off_degrees == 360) {
    yaw_current_off_degrees = 0;
  }

//  Serial.print("yaw_off yaw current LST\t");
//  Serial.print( yaw_off_degrees);
//  Serial.print("\t");
//  Serial.print(yaw_current_off_degrees);
//  Serial.print("\t");
//  Serial.println(LST_degrees);


  LCD_loop();
  fix_roll();
  fix_yaw();
  mpu_loop();
}

void LCD_loop() {
  if (Serial.available() > 0) 
  {
    data = Serial.read();
  }
  lcd.setCursor(0, 0);
  // print the number of seconds since reset:
  if (data == 0x01) {
    if (pos == 0) { pos = 6; }
    pos--;
    lcd.clear();
    lcd.print(names[pos]);
    data = 0;
  }
  else if (data == 0x02) {
    pos++;
    if (pos == 6) { pos = 0; }
    lcd.clear();
    lcd.print(names[pos]);
    data = 0;
  }
  else if (data == 0x08) {
    lcd.clear();
    lcd.print("searching");
    user_yaw = star_yaw[pos];
    user_roll = star_roll[pos];
  }
}

void fix_yaw() {
  //+/- 1 degree 
  yaw_off_degrees = round(real_yaw);
  temp_yaw = round(yaw_current);
  yaw_current_off_degrees = (double)temp_yaw;

  if (yaw_current_off_degrees == 360) {
    yaw_current_off_degrees = 0;
  }
  
  if (yaw_off_degrees > 1 && yaw_current_off_degrees < yaw_off_degrees - 1) {
    //turn counterclockwise
    yaw_send = 0xF1;
    if (yaw_send != prev_yaw_send) {
      Serial.write(yaw_send);
      Serial.flush();
      prev_yaw_send = yaw_send;
    }
  }
  else if (yaw_off_degrees < 359 && yaw_current_off_degrees > yaw_off_degrees + 1) {
    //turn clockwise 
    yaw_send = 0xF2;
    if (yaw_send != prev_yaw_send) {
      Serial.write(yaw_send);
      Serial.flush();
      prev_yaw_send = yaw_send;
    }
  }
  else {
    yaw_send = 0xF3;
    if (yaw_send != prev_yaw_send) {
      Serial.write(yaw_send);
      Serial.flush();
      prev_yaw_send = yaw_send;
    }
  }
}

void fix_roll() {
  roll_fix = round(user_roll);
  roll_fix += 90;
  roll_send = (unsigned char)roll_fix;
  //make roll send from 0 - 180
  if (roll_send != prev_roll_send) {
      Serial.write(0xFF);
      Serial.flush();
      Serial.write(roll_send);
      Serial.flush();
      prev_roll_send = roll_send;
    }
}


void mpu_loop(){
  // if programming failed, don't try to do anything
  if (!dmpReady) return;

  // wait for MPU interrupt or extra packet(s) available
  while (!mpuInterrupt && fifoCount < packetSize);

  // reset interrupt flag and get INT_STATUS byte
  mpuInterrupt = false;
  mpuIntStatus = mpu.getIntStatus();

  // get current FIFO count
  fifoCount = mpu.getFIFOCount();

  // check for overflow (this should never happen unless our code is too inefficient)
  if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
      // reset so we can continue cleanly
      mpu.resetFIFO();
      //Serial.println(F("FIFO overflow!"));

  // otherwise, check for DMP data ready interrupt (this should happen frequently)
  } else if (mpuIntStatus & 0x02) {
      // wait for correct available data length, should be a VERY short wait
      while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

      // read a packet from FIFO
      mpu.getFIFOBytes(fifoBuffer, packetSize);
      
      // track FIFO count here in case there is > 1 packet available
      // (this lets us immediately read more without waiting for an interrupt)
      fifoCount -= packetSize;

      // display Euler angles in degrees
      mpu.dmpGetQuaternion(&q, fifoBuffer);
      mpu.dmpGetGravity(&gravity, &q);
      mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
      yaw_current = ypr[0] * 180/M_PI;
      if (!yaw_flag) {
        yaw_fix = yaw_current;
        yaw_flag = true;
      }
      if (yaw_current < 0) {
       yaw_current = 360 + yaw_current;
      }
      //after the gyroscope calibrates set yaw to 0
      yaw_current = yaw_current - yaw_fix;
      if (yaw_current < 0) {
        //fix negative numbers 
        yaw_current = 360 + yaw_current;
      }
      //RA increases the other direction so we need to flip direction
      yaw_current = 360 - yaw_current;

      //something is making gyroscope slowly increase yaw 
      //to counter that I will subtract from the yaw current for every loop
      value += 0.00008;
      yaw_current = yaw_current - value;
      if (yaw_current < 0) {
        //fix negative numbers 
        yaw_current = 360 + yaw_current;
      }
      
//      Serial.print("YAWWWWW\t");
//      Serial.println(yaw_current);

      // blink LED to indicate activity
      blinkState = !blinkState;
      digitalWrite(LED_PIN, blinkState);
  }
}


void mpu_init(){
  // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    // initialize serial communication
    // (115200 chosen because it is required for Teapot Demo output, but it's
    // really up to you depending on your project)
    while (!Serial); // wait for Leonardo enumeration, others continue immediately

    // NOTE: 8MHz or slower host processors, like the Teensy @ 3.3v or Ardunio
    // Pro Mini running at 3.3v, cannot handle this baud rate reliably due to
    // the baud timing being too misaligned with processor ticks. You must use
    // 38400 or slower in these cases, or use some kind of external separate
    // crystal solution for the UART timer.

    // initialize device
//    Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();

    // verify connection
//    Serial.println(F("Testing device connections..."));
//    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

    // load and configure the DMP
//    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();

    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // turn on the DMP, now that it's ready
//        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
//        Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
        attachInterrupt(0, dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
//        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
//        Serial.print(F("DMP Initialization failed (code "));
//        Serial.print(devStatus);
//        Serial.println(F(")"));
    }

    // configure LED for output
    pinMode(LED_PIN, OUTPUT);
    
}

//Calculates local sidereal time 
//equations can be found here
//http://www.stargazing.net/kepler/altaz.html 
void get_LST(){
  month = (double) my_RTC.month;
  year = (double) my_RTC.year;
  day = (double) my_RTC.dayofmonth;
  minutes = (double) my_RTC.minutes;
  hours = (double) my_RTC.hours;
  seconds = (double) my_RTC.seconds;
  A = (double)(year-2000)*365.242199;
  B = (double)(month-1)*30.4368499;
  J2000 = A + B + (day-1) + hours/24;
  dec_time = hours + (minutes/60) + (seconds/3600);
  LST = 100.46 + 0.985647 * J2000 + longitude + 15*dec_time;
  LST_degrees = (LST - (floor(LST/360) * 360));
  LST_hours = LST_degrees/15;
}

