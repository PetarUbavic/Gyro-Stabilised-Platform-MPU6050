#include "I2Cdev.h"
#include <PID_v1.h>


#include "MPU6050_6Axis_MotionApps20.h"

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif
#include <Servo.h>

MPU6050 mpu;

// Define the 3 servo motors
Servo servo1;
Servo servo2;

float correct;
int j = 0;

double servo1Value = 0;
double servo2Value = 0;

double ulaz1 = 0;
double ulaz2 = 0;

#define OUTPUT_READABLE_YAWPITCHROLL

#define INTERRUPT_PIN 2  // use pin 2 on Arduino Uno & most boards

#define LED 10

bool blinkState = false;

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

// packet structure for InvenSense teapot demo
uint8_t teapotPacket[14] = { '$', 0x02, 0, 0, 0, 0, 0, 0, 0, 0, 0x00, 0x00, '\r', '\n' };

double Kp1 = 300;
double Ki1 = 1.7;
double Kd1 = 5;

double Kp2 = 300;
double Ki2 = 1.7;
double Kd2 = 5;


double ypr1 = 0;
double ypr2 = 0;
double srv1vl = 0;
double srv2vl = 0;

double zeljena1 = 90;
double zeljena2 = 95;

double test1 = 45;
double test2 = 45;

PID motor1(&ypr1, &srv1vl, &zeljena1, Kp1, Ki1, Kd1, DIRECT); //ulaz1 srv1vl ypr1
PID motor2(&ypr2, &srv2vl, &zeljena2, Kp2, Ki2, Kd2, DIRECT); //ulaz2 srv2vl ypr2


// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
  mpuInterrupt = true;
}

// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

void setup() {
 
  // join I2C bus (I2Cdev library doesn't do this automatically)
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
  Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif

  motor1.SetMode(AUTOMATIC);
  motor2.SetMode(AUTOMATIC);
  // initialize serial communication
  // (115200 chosen because it is required for Teapot Demo output, but it's
  // really up to you depending on your project)
  Serial.begin(38400);
  while (!Serial); // wait for Leonardo enumeration, others continue immediately

  // initialize device
  //Serial.println(F("Initializing I2C devices..."));
  mpu.initialize();
  pinMode(INTERRUPT_PIN, INPUT);
  devStatus = mpu.dmpInitialize();
  
  // supply your own gyro offsets here, scaled for min sensitivity
  mpu.setXGyroOffset(17);
  mpu.setYGyroOffset(-69);
  mpu.setZGyroOffset(27);
  mpu.setZAccelOffset(1551); // 1688 factory default for my test chip

  // make sure it worked (returns 0 if so)
  if (devStatus == 0) {
    // turn on the DMP, now that it's ready
     Serial.println(F("Enabling DMP..."));
    mpu.setDMPEnabled(true);

    attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
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

  // Define the pins to which the 3 servo motors are connected
  servo1.attach(5);
  servo2.attach(6);
  //servo2.attach(8);
}

void blinkLED(int LED1)
{
  digitalWrite(LED1, HIGH);
  delay(100);
  digitalWrite(LED1, LOW);
  delay(100);
}
// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

void loop() {

  
  // if programming failed, don't try to do anything
  if (!dmpReady) return;

  //Serial.println("12");
  
  // wait for MPU interrupt or extra packet(s) available
  while (!mpuInterrupt && fifoCount < packetSize) {
    if (mpuInterrupt && fifoCount < packetSize) {
      // try to get out of the infinite loop
      fifoCount = mpu.getFIFOCount();
    }
  }

  //Serial.println("15");
  
  // reset interrupt flag and get INT_STATUS byte
  mpuInterrupt = false;
  mpuIntStatus = mpu.getIntStatus();

  // get current FIFO count
  fifoCount = mpu.getFIFOCount();

  // check for overflow (this should never happen unless our code is too inefficient)
  if ((mpuIntStatus & _BV(MPU6050_INTERRUPT_FIFO_OFLOW_BIT)) || fifoCount >= 1024) {
    // reset so we can continue cleanly
    mpu.resetFIFO();
    fifoCount = mpu.getFIFOCount();
    Serial.println(F("FIFO overflow!"));

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
  } else if (mpuIntStatus & _BV(MPU6050_INTERRUPT_DMP_INT_BIT)) {
    // wait for correct available data length, should be a VERY short wait
    while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

    // read a packet from FIFO
    mpu.getFIFOBytes(fifoBuffer, packetSize);

    // track FIFO count here in case there is > 1 packet available
    // (this lets us immediately read more without waiting for an interrupt)
    fifoCount -= packetSize;

    // Get Yaw, Pitch and Roll values
#ifdef OUTPUT_READABLE_YAWPITCHROLL
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    //mpu.dmpGetEuler(ypr, &q);


    // Yaw, Pitch, Roll values - Radians to degrees
    ypr[0] = ypr[0] * 180 / M_PI;
    ypr[1] = ypr[1] * 180 / M_PI;
    ypr[2] = ypr[2] * 180 / M_PI;
    
    // Skip 300 readings (self-calibration process)
    if (j <= 300) {
      correct = ypr[0]; // Yaw starts at random value, so we capture last value after 300 readings
      j++;
    }
    // After 300 readings
    else {
      ypr[0] = ypr[0] - correct; // Set the Yaw to 0 deg - subtract  the last random Yaw value from the currrent value to make the Yaw 0 degrees
      // Map the values of the MPU6050 sensor from -90 to 90 to values suatable for the servo control from 0 to 180
      
     // int servo0Value = map(ypr[0], -90, 90, 0, 180); //motor za ovu osu ne postoji, imam samo 2 motora
      
      motor1.Compute();
      motor2.Compute();
       
      servo1Value = map(ypr[1], -90, 90, 0, 180); //glavni motor
      servo2Value = map(ypr[2], -90, 90, 180, 0); //ne ide od 180 do 0 zbog, kako bi se postavio na pravi inicijalni poloyaj motora 185 5


      /*ulaz1 = servo1.read();
      ulaz2 = servo2.read();*/


      ypr1 = ypr[1];
      ypr2 = ypr[2];
      srv1vl = servo1Value;
      srv2vl = servo2Value;
      
      // Control the servos according to the MPU6050 orientation

      
      Serial.print(ypr[1]); // naredne 3 linije za debugovanje preko serijske
      Serial.print("    ");
      Serial.println(ypr[2]); 

      //Serial.println(ulaz1);
      


      if(ypr[1] > 72 || ypr[1] < -85)
        {
          digitalWrite(LED, HIGH); //LOW JER PNP
          servo2Value = servo2.readMicroseconds();
          servo2.writeMicroseconds(servo2Value);
          
          //Serial.print(servo1Value);
          Serial.print(" //  ");
          Serial.println("KEC KEC");

          servo2.write(servo2Value);
          /*if(ypr[1] > 72)
            ypr1 = 72;
          if(ypr[1] < -85)
            ypr1 = -85*/
        }

      else
      {
        digitalWrite(LED, LOW);
        servo2.write(servo2Value);
      }
      
      if(ypr[2] > 87 || ypr[2] < -87)
        {
          digitalWrite(LED, HIGH); //LOW JER PNP
          servo1Value = servo1.readMicroseconds();
          servo1.writeMicroseconds(servo1Value);
          
          Serial.println("DVICA DVICA");

          
          servo1.write(servo1Value);
        }

      else
      {
        digitalWrite(LED, LOW);
        servo1.write(servo1Value);
       }


       
    }
#endif
  }
}