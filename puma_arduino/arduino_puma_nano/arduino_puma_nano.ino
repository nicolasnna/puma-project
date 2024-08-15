#include <ros.h>
#include <std_msgs/Float32.h>

int sensorVoltage = A7;
float voltageValue = 0.0;

// Variables ROS
ros::NodeHandle nh;

std_msgs::Float32 battery_voltage;
ros::Publisher batteryVoltagePub("puma/sensors/battery/raw_12v", &battery_voltage);

void setup() {
  nh.initNode();
  nh.advertise(batteryVoltagePub);
}

void loop() {

  if (nh.connected()) {
    float sumVoltage = 0.0;
    for (int i = 0; i<25; i++) {
      float volt = (float)25*analogRead(sensorVoltage)/1024.0;
      sumVoltage += volt;
    }
    battery_voltage.data = sumVoltage/25;
    batteryVoltagePub.publish( &battery_voltage );
  }
  delay(100);
  nh.spinOnce();
}
