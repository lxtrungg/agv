#define USE_USBCON //If you use a Leonardo board
#include "I2Cdev.h"
#include "MPU9250_6Axis_MotionApps20.h"
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

MPU9250 mpu;

#define INTERRUPT_PIN 2
#define LED_PIN 13

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
VectorFloat gravity;    // [x, y, z]            gravity vector
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

#include <ros.h>
#include <ros/time.h>
#include <geometry_msgs/Vector3.h>
#include <std_msgs/Bool.h>

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
  mpuInterrupt = true;
}

bool isStart = false;

ros::NodeHandle nh;
geometry_msgs::Vector3  rpy_msg;
ros::Publisher rpy_pub("/imu/rpy", &rpy_msg);

void sub_callback(const std_msgs::Bool& sub_msg) {
  isStart = sub_msg.data;
}
ros::Subscriber<std_msgs::Bool> str_sub("/agv/start_imu", &sub_callback);

void setup() {
    pinMode(LED_BUILTIN, OUTPUT);
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    Serial.begin(115200);
    mpu.initialize();
    pinMode(INTERRUPT_PIN, INPUT);
    devStatus = mpu.dmpInitialize();
    mpu.setXGyroOffset(228);
    mpu.setYGyroOffset(93);
    mpu.setZGyroOffset(-64);
    mpu.setZAccelOffset(1421); // 1688 factory default for my test chip
    
    if (devStatus == 0) {
        digitalWrite(LED_BUILTIN, HIGH);
        mpu.CalibrateAccel(6);
        mpu.CalibrateGyro(6);
        mpu.PrintActiveOffsets();
        mpu.setDMPEnabled(true); 
        digitalPinToInterrupt(INTERRUPT_PIN);
        attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();
        dmpReady = true;
        packetSize = mpu.dmpGetFIFOPacketSize();
    } 
    else {
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }
    nh.initNode();
    nh.advertise(rpy_pub);
    nh.subscribe(str_sub);
}

void loop() {
  if (!dmpReady) {
    digitalWrite(LED_BUILTIN, LOW);
    delay(500);
    digitalWrite(LED_BUILTIN, HIGH);
    delay(500);
  }
  if (!isStart) {
    mpu.dmpInitialize();
    mpu.CalibrateAccel(6);
    mpu.CalibrateGyro(6);
    mpu.setDMPEnabled(true);
  }
  else {
  // read a packet from FIFO
    if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
        // display Euler angles in degrees
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        q.y = -q.y;
        q.z = -q.z;
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    
        rpy_msg.x = ypr[2];
        rpy_msg.y = ypr[1];
        rpy_msg.z = ypr[0];
        rpy_pub.publish(&rpy_msg);
    //          String log_msg = String(rpy_msg.z);
    //          nh.loginfo(log_msg.c_str());
    }
  }
  nh.spinOnce();
  delay(1);
}
