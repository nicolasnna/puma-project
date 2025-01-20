#include <ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>

const int sensorVoltage = A0;
const float conversionFactor = 100.0 / 1024.0; 
const uint8_t numSamples = 15;
const int releInput1 = 4; // SIN USO
const int releInput2 = 5; // SIN USO
const int releInput3 = 6; // SIN USO
const int releInput4 = 7; // Luz

// Variables ROS
ros::NodeHandle nh;
std_msgs::Float32 battery_voltage;
ros::Publisher batteryVoltagePub("puma/sensors/battery/raw_72v", &battery_voltage);

void lightsCallback( const std_msgs::Bool& data_received);
ros::Subscriber<std_msgs::Bool> light_sub("puma/control/lights", lightsCallback);

void setup() {
  nh.initNode();
  nh.advertise(batteryVoltagePub);
  nh.subscribe(light_sub);
  pinMode(releInput1, OUTPUT);
  pinMode(releInput2, OUTPUT);
  pinMode(releInput3, OUTPUT);
  pinMode(releInput4, OUTPUT);
  digitalWrite(releInput1, HIGH);
  digitalWrite(releInput2, HIGH);
  digitalWrite(releInput3, HIGH);
  digitalWrite(releInput4, HIGH);
}

void loop() {
  if (nh.connected()) {
    uint32_t sumVoltage = 0;  // Usar enteros en lugar de floats para sumar las lecturas

    for (uint8_t i = 0; i < numSamples; i++) {
      sumVoltage += analogRead(sensorVoltage);
    }

    // Convertir el promedio a voltaje 
    float averageVoltage = (float)sumVoltage * conversionFactor;
    battery_voltage.data =  averageVoltage / numSamples;
    batteryVoltagePub.publish(&battery_voltage);
  } else {
    digitalWrite(releInput4, HIGH);
  }

  delay(400);
  nh.spinOnce();
}

void lightsCallback( const std_msgs::Bool& data_received) {
  digitalWrite(releInput4, data_received.data ? LOW : HIGH);
}
