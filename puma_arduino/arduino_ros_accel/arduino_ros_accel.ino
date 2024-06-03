#include <ros.h>
#include <std_msgs/Int16.h>
#include <accel_puma_msg/accel_status.h>

ros::NodeHandle nh;

void accelCallback( const std_msgs::Int16& data_received );
ros::Subscriber<std_msgs::Int16> accel_sub("accel_puma/value", accelCallback);

accel_puma_msg::accel_status accelStatus;
ros::Publisher accel_pub("accel_puma/status", &accelStatus);

const int pwmPin = 2;
const int minPwmValue = 43; // Valor minimo
int pwmValue = minPwmValue; 

void setup() {
  pinMode(pwmPin,OUTPUT);
  nh.initNode();
  nh.advertise(accel_pub);
  nh.subscribe(accel_sub);

  // Value init 
  accelStatus.valuePwm = 0;
  accelStatus.outVoltage = 0.0;
}

void loop() {
  analogWrite(pwmPin, pwmValue); 
  accel_pub.publish( &accelStatus );
  nh.spinOnce();
  delay(20);
}

void accelCallback( const std_msgs::Int16& data_received ) {
  if (data_received.data >= 43 || data_received.data < 169){
    pwmValue = data_received.data + minPwmValue;
    accelStatus.valuePwm = pwmValue;
    accelStatus.outVoltage = accelStatus.valuePwm/255.0 * 5.0;
  }
}
