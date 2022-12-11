#define USE_USBCON //If you use a Leonardo board
#include <MPU9250_asukiaaa.h>
#include <ros.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Bool.h>
MPU9250_asukiaaa mpu;
bool dmpReady = false;
int sample = 2000;
int i = 0;
float bias_accel[3];
float bias_gyro[3] ;
float acc[3];
float gyro[3];
float mag[3];   
bool dataReady =false;
bool Ready=false;
ros::NodeHandle_<ArduinoHardware, 2, 2, 80, 512> nh;
sensor_msgs::Imu imu_msg;
ros::Publisher imu("imu/data_raw", &imu_msg);

void sub_callback(const std_msgs::Bool& sub_msg) {
 dataReady = sub_msg.data;
}

ros::Subscriber<std_msgs::Bool> str_sub("/agv/start_imu", &sub_callback);

void setup() {
  nh.initNode();
  nh.advertise(imu);
  nh.subscribe(str_sub);
  imu_msg.header.frame_id = "imu_link";
  Serial.begin(115200);
  while(!Serial);
  mpu.beginAccel();
  mpu.beginGyro();
  mpu.beginMag();
  uint8_t sensorId;
  if (mpu.readId(&sensorId) == 0) {
    Serial.println("sensorId: " + String(sensorId));
  } else {
    Serial.println(F("Cannot read sensorId"));
  }
  
}    
void loop() {
    if (dataReady == false){
      acc[0]=0;
      acc[1]=0;
      acc[2]=0;
      gyro[0]=0;
      gyro[1]=0;
      gyro[2]=0;
      bias_accel[0]=0;
      bias_accel[1]=0;
      bias_accel[2]=0;
      bias_gyro[0]=0;
      bias_gyro[1]=0;
      bias_gyro[2]=0;
      Ready = false;
    }
      if (dataReady == true){
        if (Ready == false){
          i=0;
         while (i < sample) {
          i+=1;
          if (mpu.accelUpdate() == 0) {
            bias_accel[0] += mpu.accelX();
            bias_accel[1] += mpu.accelY();
            bias_accel[2] += mpu.accelZ();
          }
          if (mpu.gyroUpdate() == 0) { 
            bias_gyro[0] += mpu.gyroX()*3.141592653589793/180;
            bias_gyro[1] += mpu.gyroY()*3.141592653589793/180;
            bias_gyro[2] += mpu.gyroZ()*3.141592653589793/180;
          }
        
        }
          bias_accel[0]/=sample; 
          bias_accel[1]/=sample; 
          bias_accel[2]/=sample; 
    
          bias_accel[2]= bias_accel[2] - 1.0;
    
          bias_gyro[0]/=sample; 
          bias_gyro[1]/=sample;    
          bias_gyro[2]/=sample;
          Ready = true;  
        }
      if (mpu.accelUpdate() == 0) {
      acc[0]=mpu.accelX();
      acc[1]=mpu.accelY();
      acc[2]=mpu.accelZ();
      }
      if (mpu.gyroUpdate() == 0) {
      gyro[0]=mpu.gyroX()*3.141592653589793/180;
      gyro[1]=mpu.gyroY()*3.141592653589793/180;
      gyro[2]=mpu.gyroZ()*3.141592653589793/180;
      }
      
      acc[0]-=bias_accel[0];
      acc[1]-=bias_accel[1];
      acc[2]-=bias_accel[2];
      gyro[0]-=bias_gyro[0];
      gyro[1]-=bias_gyro[1];
      gyro[2]-=bias_gyro[2];
//      Serial.println("Acc Value is " + String(acc[0]) +" " + String(acc[1]) +" "+ String(acc[2]));
//     Serial.println("Gyro Value is" + String(gyro[0]) +" " + String(gyro[1])+ " " + String(gyro[2]));
//      Serial.println(F("Hello"));
//      Serial.println();
     imu_msg.header.stamp =nh.now();
     imu_msg.orientation.x=0;
     imu_msg.orientation.y=0;
     imu_msg.orientation.z=0;
     imu_msg.orientation.w=1;
     imu_msg.orientation_covariance[0]=1e6;
     imu_msg.orientation_covariance[4]=1e6;
     imu_msg.orientation_covariance[8]=0.1;

     imu_msg.angular_velocity.x =gyro[0];
     imu_msg.angular_velocity.y =gyro[1];
     imu_msg.angular_velocity.z =gyro[2];
     imu_msg.angular_velocity_covariance[0]=1e6;
     imu_msg.angular_velocity_covariance[4]=1e6;
     imu_msg.angular_velocity_covariance[8]=0.1;

     imu_msg.linear_acceleration.x =acc[0];
     imu_msg.linear_acceleration.y =acc[1];
     imu_msg.linear_acceleration.z =acc[2];
     imu_msg.linear_acceleration_covariance[0]=1e6;
     imu_msg.linear_acceleration_covariance[4]=1e6;
     imu_msg.linear_acceleration_covariance[8]=0.1;
    
     imu.publish( &imu_msg);
     }
     
    nh.spinOnce();
    delay(100);
}
