#include <ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
#include <puma_msgs/StatusArduinoRelay.h>

const int sensorVoltage = A0;
const float conversionFactor = 100.0 / 1024.0; 
const uint8_t numSamples = 15;
const int releInput1 = 4; // Accionador de luz seguridad - NO
const int releInput2 = 5; // Conector de carga - NO
const int releInput3 = 6; // Alimentación Jetson - NC
const int releInput4 = 7; // Luz - NO

#define debounceLimitLights 50
#define debounceLimitJetson 1000
#define debounceLimitChargeConnector 100
#define debounceLimitSecurityLight 100

// Status relay 
bool releLightFront = false;
bool releJetsonSupply = true;
bool releChargeConnection = false;
bool releLightSecurity = false;

#define timeBetweenBattery 2000
unsigned long timeBetweenMeasurement = 1000;
#define timeBetweenStatusRelay 2000

// Variables ROS
ros::NodeHandle nh;
std_msgs::Float32 battery_voltage;
ros::Publisher batteryVoltagePub("puma/sensors/battery/raw_72v", &battery_voltage);

puma_msgs::StatusArduinoRelay status_msgs;
ros::Publisher statusRelayPub("puma/arduino/status_relay", &status_msgs);

void lightsCallback( const std_msgs::Bool& data_received);
ros::Subscriber<std_msgs::Bool> light_sub("puma/control/lights_front", lightsCallback);

void jetsonCallback( const std_msgs::Bool& signal);
ros::Subscriber<std_msgs::Bool> jetson_sub("puma/control/jetson_supply", jetsonCallback);

void chargeConnectorCb( const std_msgs::Bool& signal);
ros::Subscriber<std_msgs::Bool> charge_sub("puma/control/charge_connection", chargeConnectorCb);

void securityLightCb( const std_msgs::Bool& signal);
ros::Subscriber<std_msgs::Bool> security_light_sub("puma/control/security_lights", securityLightCb);

void setup() {
  nh.initNode();
  nh.advertise(batteryVoltagePub);
  nh.advertise(statusRelayPub);
  nh.subscribe(light_sub);
  nh.subscribe(jetson_sub);
  nh.subscribe(charge_sub);
  nh.subscribe(security_light_sub);
  pinMode(releInput1, OUTPUT);
  pinMode(releInput2, OUTPUT);
  pinMode(releInput3, OUTPUT);
  pinMode(releInput4, OUTPUT);
  shutdownReles();
  timeBetweenMeasurement = timeBetweenBattery / numSamples;
}

void loop() {
  if (nh.connected()) {
    measurementAndPublishVoltage();
    publishStatusRelay();
  } else {
    shutdownReles();
  }
  nh.spinOnce();
}

void shutdownReles() {
  digitalWrite(releInput1, HIGH);
  digitalWrite(releInput2, HIGH);
  digitalWrite(releInput3, HIGH);
  digitalWrite(releInput4, HIGH);
}

void measurementAndPublishVoltage(){
  static uint32_t sumVoltage = 0;
  static unsigned long lastTimeMeasurement = 0;
  static uint8_t count = 0;
  unsigned long actualTime = millis();
  if (actualTime - lastTimeMeasurement >= timeBetweenMeasurement ) {
    sumVoltage += analogRead(sensorVoltage);
    count++;
    lastTimeMeasurement = actualTime;
  }
  if (count >= numSamples) {
    float averageVoltage = (float)sumVoltage * conversionFactor;
    battery_voltage.data =  averageVoltage / numSamples;
    batteryVoltagePub.publish(&battery_voltage);
    count = 0;
    sumVoltage = 0;
  }
}

void publishStatusRelay() {
  static unsigned long lastTimeStatusRelay = 0;
  unsigned long actualTime = millis();
  if (actualTime - lastTimeStatusRelay >= timeBetweenStatusRelay) {
    status_msgs.lights_front = releLightFront;
    status_msgs.jetson_supply = releJetsonSupply;
    status_msgs.charge_connection = releChargeConnection;
    status_msgs.security_lights = releLightSecurity;
    statusRelayPub.publish(&status_msgs);
    lastTimeStatusRelay = actualTime;
  }
}

void lightsCallback( const std_msgs::Bool& data_received) {
  static unsigned long prevCmdLights = 0;
  unsigned long currentTime = millis();
  if (currentTime - prevCmdLights > debounceLimitLights) {
    digitalWrite(releInput4, data_received.data ? LOW : HIGH);
    releLightFront = data_received.data;
    prevCmdLights = currentTime;
  }
}

void jetsonCallback( const std_msgs::Bool& signal) {
  static unsigned long prevCmdJetson = 0;
  unsigned long currentTime = millis();
  if (currentTime - prevCmdJetson > debounceLimitJetson) {
    digitalWrite(releInput3, signal.data ? HIGH : LOW);
    releJetsonSupply = signal.data;
    prevCmdJetson = currentTime;
  }
}

void chargeConnectorCb( const std_msgs::Bool& signal) {
  static unsigned long prevCmdCharge = 0;
  unsigned long currentTime = millis();
  if (currentTime - prevCmdCharge > debounceLimitChargeConnector) {
    digitalWrite(releInput2, signal.data ? LOW : HIGH);
    releChargeConnection = signal.data;
    prevCmdCharge = currentTime;
  }
}

void securityLightCb( const std_msgs::Bool& signal) {
  static unsigned long prevCmdSecuritySignal = 0;
  unsigned long currentTime = millis();
  if (currentTime - prevCmdSecuritySignal > debounceLimitSecurityLight) {
    digitalWrite(releInput1, signal.data ? LOW : HIGH);
    releLightSecurity = signal.data;
    prevCmdSecuritySignal = currentTime;
  }
}