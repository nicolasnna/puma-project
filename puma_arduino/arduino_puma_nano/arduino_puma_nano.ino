#include <ros.h>
#include <std_msgs/Float32.h>

const int sensorVoltage = A7;
const float voltageFactor = 25.0 / 1024.0;

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
    uint32_t sumVoltage = 0;  // Usar enteros en lugar de floats para sumar las lecturas

    for (uint8_t i = 0; i < 25; i++) {
      sumVoltage += analogRead(sensorVoltage);
    }

    // Convertir el promedio a voltaje una vez
    battery_voltage.data = (float)(sumVoltage * voltageFactor / 25.0);
    batteryVoltagePub.publish(&battery_voltage);
  }

  delay(100);
  nh.spinOnce();
}
