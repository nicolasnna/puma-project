#include <ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>

const int sensorVoltage = A0;
const float voltageFactor = 5.0 / 1024.0 * 20; // factor 20 por sensor de 100v, 100 / 5 = 20
const int releInput = 4;

// Variables ROS
ros::NodeHandle nh;
std_msgs::Float32 battery_voltage;
ros::Publisher batteryVoltagePub("puma/sensors/battery/raw_72v", &battery_voltage);

void lightsCallback( const std_msgs::Bool& data_received);
ros::Subscriber<std_msgs::Bool> light_sub("puma/lights/command", lightsCallback);

void setup() {
  nh.initNode();
  nh.advertise(batteryVoltagePub);
  nh.subscribe(light_sub);
  pinMode(releInput, OUTPUT);
  digitalWrite(releInput, HIGH);
}

void loop() {
  if (nh.connected()) {
    uint32_t sumVoltage = 0;  // Usar enteros en lugar de floats para sumar las lecturas

    for (uint8_t i = 0; i < 5; i++) {
      sumVoltage += analogRead(sensorVoltage);
    }

    // Convertir el promedio a voltaje una vez
    battery_voltage.data = (float)(sumVoltage * voltageFactor / 6.0);
    batteryVoltagePub.publish(&battery_voltage);
  } else {
    digitalWrite(releInput, HIGH);
  }

  delay(400);
  nh.spinOnce();
}

void lightsCallback( const std_msgs::Bool& data_received) {
  if (data_received.data == true) {
    digitalWrite(releInput, LOW);
  } else {
    digitalWrite(releInput, HIGH);
  }
}
