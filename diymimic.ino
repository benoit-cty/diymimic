// From MPU6050 to PPM Signal
// by Benoit Courty july 2015 - benoit.courty {at] neo-robotix.com
// Original source for MPU6050 use by Jeff Rowberg : https://github.com/jrowberg/i2cdevlib
// You have to copy the folder MPU6050 and I2Cdev to your librairies directory.
// Original source for PPM by Hasi on http://www.rcgroups.com/forums/showthread.php?t=1808432
#define BAUD_RATE 115200
#define OUTPUT_SERIAL
// For MPU6050
#include "Wire.h"
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif

//#define OUTPUT_READABLE_YAWPITCHROLL

MPU6050 mpu;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================
volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
  mpuInterrupt = true;
}



////////////////////// PPM CONFIGURATION///////////////////////////////
#define sigPin 3                  // Pin of the PPM signalconnected to the trainer port
#define chanel_number 8           // Set the number of chanels
#define PWM_CENTER 1500           // Set the default servo value
#define PWM_DEAD_ZONE 20          // Set a dead zone value
#define PPM_FrLen 22500           //set the PPM frame length in microseconds (1ms = 1000µs)
#define PPM_PulseLen 300          //set the pulse length
#define onState 1                 //set polarity of the pulses: 1 is positive, 0 is negative
#define LED_PIN 13
// Define the channel number to send the data (starting from 1 and depending of your needs and configuration)
#define OUT_CH_YAW 4
#define OUT_CH_PITCH 2
#define OUT_CH_ROLL 1

bool blinkState = false;
// Array containing the value of all channels
int ppm[chanel_number];
int iAngleYawInit = PWM_CENTER;
int iChannelYaw = PWM_CENTER;
int iChannelPitch = PWM_CENTER;
int iChannelRoll = PWM_CENTER;
uint16_t AvgAngle;
int iAngleYaw, iAnglePitch, iAngleRoll;
//////////////////////////////////////////////////////////////////


void setup() {
  // initialize serial communication
  Serial.begin(BAUD_RATE);
  // NOTE: 8MHz or slower host processors, like the Teensy @ 3.3v or Ardunio
  // Pro Mini running at 3.3v, cannot handle high baud rate reliably due to
  // the baud timing being too misaligned with processor ticks. You must use
  // 38400 or slower in these cases, or use some kind of external separate
  // crystal solution for the UART timer.
  
  while (!Serial); // wait for Leonardo enumeration, others continue immediately
  // LED
  pinMode(LED_PIN, OUTPUT);
  // MPU6050 //////////////
  // join I2C bus (I2Cdev library doesn't do this automatically)
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
  TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif
  // initialize device
  Serial.println(F("Initializing I2C devices..."));
  mpu.initialize();
  // verify connection
  Serial.println(F("Testing device connections..."));
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));
  // load and configure the DMP
  Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();
  // supply your own gyro offsets here, scaled for min sensitivity
  mpu.setXGyroOffset(220);
  mpu.setYGyroOffset(76);
  mpu.setZGyroOffset(-85);
  mpu.setZAccelOffset(1788); // 1688 factory default for my test chip
  // make sure it worked (returns 0 if so)
  if (devStatus == 0) {
    // turn on the DMP, now that it's ready
    Serial.println(F("Enabling DMP..."));
    mpu.setDMPEnabled(true);
    // enable Arduino interrupt detection
    Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
    attachInterrupt(0, dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();
    // set our DMP Ready flag so the main loop() function knows it's okay to use it
    Serial.println(F("DMP ready! Waiting for first interrupt..."));
    dmpReady = true;
    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();
  } else {
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)
    Serial.print(F("DMP Initialization failed (code "));
    Serial.print(devStatus);
    Serial.println(F(")"));
  }
  
  /////////////////////////////////
  //// PPM Setup
  // Set values of PPM channel
  for (int i = 0; i < chanel_number; i++) {
    ppm[i] = PWM_CENTER;
  }
  // Set the output PIN for PPM
  pinMode(sigPin, OUTPUT);
  digitalWrite(sigPin, !onState);  //set the PPM signal pin to the default state (off)
  // Configuration of timer for PPM
  cli();
  TCCR1A = 0; // set entire TCCR1 register to 0
  TCCR1B = 0;
  OCR1A = 100;  // compare match register, change this
  TCCR1B |= (1 << WGM12);  // turn on CTC mode
  TCCR1B |= (1 << CS11);  // 8 prescaler: 0,5 microseconds at 16mhz
  TIMSK1 |= (1 << OCIE1A); // enable timer compare interrupt
  sei();
}

void loop() {
  // if programming failed, don't try to do anything
  if (!dmpReady) return;

  // wait for MPU interrupt or extra packet(s) available
  while (!mpuInterrupt && fifoCount < packetSize) {
    // other program behavior stuff here
  }
  // reset interrupt flag and get INT_STATUS byte
  mpuInterrupt = false;
  mpuIntStatus = mpu.getIntStatus();
  // get current FIFO count
  fifoCount = mpu.getFIFOCount();
  // check for overflow (this should never happen unless our code is too inefficient)
  if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
    // reset so we can continue cleanly
    mpu.resetFIFO();
    Serial.println(F("FIFO overflow!"));
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
#ifdef OUTPUT_READABLE_YAWPITCHROLL
    Serial.print("ypr\t");
    Serial.print(ypr[0] * 180 / M_PI);
    Serial.print("\t");
    Serial.print(ypr[1] * 180 / M_PI);
    Serial.print("\t");
    Serial.println(ypr[2] * 180 / M_PI);
#endif

    // YAW Value ////////////////////////
    iAngleYaw = (ypr[0] * 180 / M_PI);
    if(iAngleYawInit == PWM_CENTER){
      iAngleYawInit=iAngleYaw;
    }
    iAngleYaw=iAngleYaw+iAngleYawInit;
    iChannelYaw = map(iAngleYaw, -90, 90, 1000, 2000);
#ifdef OUTPUT_SERIAL
    Serial.print("Yaw =\t");
    Serial.print(iAngleYaw);
    Serial.print(" PWM=\t");
    Serial.print(iChannelYaw);
#endif
    // Dead zone
    if ((iChannelYaw < PWM_CENTER+PWM_DEAD_ZONE) && (iChannelYaw > PWM_CENTER-PWM_DEAD_ZONE) ) iChannelYaw = PWM_CENTER;
    if (iChannelYaw < 1000) iChannelYaw = 1000;
    if (iChannelYaw > 2000) iChannelYaw = 2000;

    ppm[OUT_CH_YAW - 1] = iChannelYaw;

    // PITCH Value ////////////////////////
    iAnglePitch = (ypr[2] * 180 / M_PI);
    iChannelPitch = map(iAnglePitch, -90, 90, 1000, 2000);
#ifdef OUTPUT_SERIAL
    Serial.print(" Pitch=\t");
    Serial.print(iAnglePitch);
    Serial.print(" PWM=\t");
    Serial.print(iChannelPitch);
#endif
    // Dead zone
    if ((iChannelPitch < PWM_CENTER+PWM_DEAD_ZONE) && (iChannelPitch > PWM_CENTER-PWM_DEAD_ZONE) ) iChannelPitch = PWM_CENTER;
    if (iChannelPitch < 1000) iChannelPitch = 1000;
    if (iChannelPitch > 2000) iChannelPitch = 2000;
    ppm[OUT_CH_PITCH - 1] = iChannelPitch;

    // ROLL Value ////////////////////////
    iAngleRoll = (ypr[1] * 180 / M_PI);
    iChannelRoll = map(iAngleRoll, -90, 90, 2000, 1000); // 2000,1000 instead of 1000,2000 to reverse channel. It depends of your needs
#ifdef OUTPUT_SERIAL
    Serial.print(" Roll=\t");
    Serial.print(iAngleRoll);
    Serial.print(" PWM=\t");
    Serial.println(iChannelRoll);
#endif
    // Dead zone
    if ((iChannelRoll < PWM_CENTER+PWM_DEAD_ZONE) && (iChannelRoll > PWM_CENTER-PWM_DEAD_ZONE) ) iChannelRoll = PWM_CENTER;
    if (iChannelRoll < 1000) iChannelRoll = 1000;
    if (iChannelRoll > 2000) iChannelRoll = 2000;
    ppm[OUT_CH_ROLL - 1] = iChannelRoll;

    // blink LED to indicate activity
    blinkState = !blinkState;
    digitalWrite(LED_PIN, blinkState);
  }

}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// PPM generator interuption
ISR(TIMER1_COMPA_vect) {
  static boolean state = true;
  TCNT1 = 0;
  if (state) { //start pulse
    digitalWrite(sigPin, onState);
    OCR1A = PPM_PulseLen * 2;
    state = false;
  }
  else { //end pulse and calculate when to start the next pulse
    static byte cur_chan_numb;
    static unsigned int calc_rest;
    digitalWrite(sigPin, !onState);
    state = true;
    if (cur_chan_numb >= chanel_number) {
      cur_chan_numb = 0;
      calc_rest = calc_rest + PPM_PulseLen;//
      OCR1A = (PPM_FrLen - calc_rest) * 2;
      calc_rest = 0;
    }
    else {
      OCR1A = (ppm[cur_chan_numb] - PPM_PulseLen) * 2;
      calc_rest = calc_rest + ppm[cur_chan_numb];
      cur_chan_numb++;
    }
  }
}
