// IMU Library
#include "ICM_20948.h"
#define AD0_VAL 0
ICM_20948_I2C myICM;
#define WIRE_PORT Wire
#define DEG_TO_RADS 0.0174533
#define aG 9.8067
// ROS
#include <ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include <puma_msgs/ImuData.h>

ros::NodeHandle nh;
sensor_msgs::Imu imu_msg;
ros::Publisher imuRawPub("imu", &imu_msg);
sensor_msgs::MagneticField compass_msg;
ros::Publisher compassRawPub("mag", &compass_msg);

puma_msgs::ImuData icm20948_msg;
ros::Publisher icm20948Pub("puma/sensors/icm20948/raw", &icm20948_msg);

void setup() {
  // put your setup code here, to run once:
  WIRE_PORT.begin();
  WIRE_PORT.setClock(400000);
  nh.getHardware()->setBaud(500000);
  nh.initNode();
  nh.advertise(icm20948Pub);

  nh.loginfo("Iniciando IMU");
  bool initialized = false;
  while (!initialized)
  {
    myICM.begin(WIRE_PORT, AD0_VAL);

    if (myICM.status != ICM_20948_Stat_Ok){
      delay(500);
    }
    else {
      initialized = true;
    }
  }
  nh.loginfo("Configurando Imu");
  bool success = true;
  success &= (myICM.initializeDMP() == ICM_20948_Stat_Ok);
  // Enable the DMP orientation sensor
  myICM.enableDMPSensor(INV_ICM20948_SENSOR_ACCELEROMETER);
  myICM.enableDMPSensor(INV_ICM20948_SENSOR_GYROSCOPE);
  myICM.enableDMPSensor(INV_ICM20948_SENSOR_MAGNETIC_FIELD_UNCALIBRATED);
  
  // Configurar frecuencia DMP (ej. 100Hz)
  myICM.setDMPODRrate(DMP_ODR_Reg_Accel, 40);
  myICM.setDMPODRrate(DMP_ODR_Reg_Gyro, 40);
  myICM.setDMPODRrate(DMP_ODR_Reg_Cpass, 40);

  myICM.enableDMP();
  myICM.resetDMP();

  // imu_msg.header.frame_id = compass_msg.header.frame_id = "imu_link";
 
}

void loop() {
  // put your main code here, to run repeatedly:
  static unsigned long prevTime = 0;
  unsigned long currTime = millis();

  if (currTime - prevTime >= 20) {
    prevTime = currTime;
    if (myICM.dataReady()) {

      myICM.getAGMT();
      // imu_msg.linear_acceleration.x = myICM.accX() / 1000.0 * aG;
      // imu_msg.linear_acceleration.y = myICM.accY() / 1000.0 * aG;
      // imu_msg.linear_acceleration.z = myICM.accZ() / 1000.0 * aG;

      // imu_msg.angular_velocity.x = myICM.gyrX() * DEG_TO_RADS;
      // imu_msg.angular_velocity.y = myICM.gyrY() * DEG_TO_RADS;
      // imu_msg.angular_velocity.z = myICM.gyrZ() * DEG_TO_RADS;

      // compass_msg.magnetic_field.x = myICM.magX();
      // compass_msg.magnetic_field.y = myICM.magY();
      // compass_msg.magnetic_field.z = myICM.magZ();
      
      // imu_msg.header.stamp = compass_msg.header.stamp = nh.now();
      // imuRawPub.publish(&imu_msg);
      // compassRawPub.publish(&compass_msg);

      icm20948_msg.acc_x_g = myICM.accX() / 1000.0;
      icm20948_msg.acc_y_g = myICM.accY() / 1000.0;
      icm20948_msg.acc_z_g = myICM.accZ() / 1000.0;
      
      icm20948_msg.gyro_x_deg = myICM.gyrX();
      icm20948_msg.gyro_y_deg = myICM.gyrY();
      icm20948_msg.gyro_z_deg = myICM.gyrZ();

      icm20948_msg.mag_x_uT = myICM.magX();
      icm20948_msg.mag_y_uT = myICM.magY();
      icm20948_msg.mag_z_uT = myICM.magZ();

      icm20948_msg.temp_C = myICM.temp();

      icm20948Pub.publish(&icm20948_msg);

    }
  }

  nh.spinOnce();
  // delay(20);
}
