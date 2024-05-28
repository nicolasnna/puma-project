#include <ros.h>
#include <brake_controller_msgs/brake_control.h>
#include <control_dir_msgs/dir_data.h>
#include <std_msgs/Int16.h>
#include <init_puma/status_arduino.h>
const int pin5v = 12;

// Variables freno
const int MS[3] = {51,49,47};
const int DIR_MPP1 = 8;
const int STEP_MPP1 = 9;
const int DIR_MPP2 = 10;
const int STEP_MPP2 = 11;

int positionCurrent = 0;
int positionToReach = 0;
int positionMissing = 0;
bool positiveRunMotor = true;
bool moveBrake = false;

// Variables direccion
const int sensorPositionPin = A1;
const int enableDirPin = 14;
const int right_dir = 2;
const int left_dir = 3;
const int limit_min = 263;
const int limit_max = 521;
const int zero_position = 392;

int valueDirection = 0;
int sensorPositionValue = 0;
bool stop_dir_right = true;
bool stop_dir_left = true;
bool enablePinDirection = false;

// Variables acelerador
const int acceleratorPin = 5;
const int minAcceleratorValue = 43;
const int maxAcceleratorValue = 169;
int acceleratorValue = minAcceleratorValue;
int newAcceleratorValue = 0;
bool enableAccelerator = false;

// Variables ROS
ros::NodeHandle nh;

void brakeCallback( const brake_controller_msgs::brake_control& data_received );
ros::Subscriber<brake_controller_msgs::brake_control> brake_sub("brake_controller/data_control", brakeCallback);

void dirCallback( const control_dir_msgs::dir_data& data_received);
ros::Subscriber<control_dir_msgs::dir_data> dir_sub("control_dir/dir_data", dirCallback);

void accelCallback( const std_msgs::Int16& data_received );
ros::Subscriber<std_msgs::Int16> accel_sub("accel_puma/value", accelCallback);

init_puma::status_arduino status_msg;
ros::Publisher arduinoStatusPub("arduino_puma/status", &status_msg);


void setup() {
  // Config ros
  nh.initNode();
  nh.subscribe(brake_sub);
  nh.subscribe(dir_sub);
  nh.subscribe(accel_sub);
  nh.advertise(arduinoStatusPub);
  nh.loginfo("Inicializando Arduino");
  // Config pinMode brake
  pinMode(DIR_MPP1, OUTPUT);
  pinMode(DIR_MPP2, OUTPUT);
  pinMode(STEP_MPP1, OUTPUT);
  pinMode(STEP_MPP2, OUTPUT);
  // Resolution brake
  pinMode(MS[0], OUTPUT);
  pinMode(MS[1], OUTPUT);
  pinMode(MS[2], OUTPUT);
  digitalWrite(MS[0],HIGH);
  digitalWrite(MS[1],LOW);
  digitalWrite(MS[2],LOW);
  digitalWrite(DIR_MPP1, HIGH);
  // Config pinMode dir
  pinMode(enableDirPin, OUTPUT);
  pinMode(right_dir, OUTPUT);
  pinMode(left_dir, OUTPUT);
  digitalWrite(enableDirPin, LOW);
  // Config pinMode accel
  pinMode(acceleratorPin, OUTPUT);

  // Write topics to msg status
  status_msg.topic_brake = "brake_controller/data_control";
  status_msg.topic_dir = "control_dir/dir_data";
  status_msg.topic_accel = "accel_puma/value";

  pinMode(pin5v, OUTPUT);
  digitalWrite(pin5v, HIGH);
}

void loop() {
  accelController();
  dirController();
  brakeController();
  // Publish msg
  publishMsgStatus();
  nh.spinOnce();
  delay(1);
}

void publishMsgStatus() {
  // Brake info
  status_msg.position_brake = positionCurrent;
  status_msg.is_move_brake = moveBrake;
  // Dir info
  status_msg.current_position_dir = sensorPositionValue;
  status_msg.enable_dir = enablePinDirection;
  status_msg.is_limit_right_dir = stop_dir_right;
  status_msg.is_limit_left_dir = stop_dir_left;
  // Accel info
  status_msg.enable_accel = enableAccelerator;
  status_msg.pwm_accel = acceleratorValue;
  status_msg.voltage_accel = (acceleratorValue/255.0) * 5.0;
  arduinoStatusPub.publish(&status_msg);
}

void accelController(){
  if (enableAccelerator) {
    analogWrite(acceleratorPin, acceleratorValue);
  } else {
    analogWrite(acceleratorPin, minAcceleratorValue);
    acceleratorValue = minAcceleratorValue;
  }
}

void dirController(){
  sensorPositionValue = analogRead(sensorPositionPin);
  // Checking if can turn on direction
  if(sensorPositionValue > limit_min){
    stop_dir_right = false;
  } else {
    nh.logwarn("Direccion a la derecha maxima alcanzada");
    stop_dir_right = true;
  }
  if(sensorPositionValue < limit_max){
    stop_dir_left = false;
  } else {
    nh.logwarn("Direccion a la derecha maxima alcanzada");
    stop_dir_left = true;
  }

  if (stop_dir_right == false && valueDirection > 0 && enablePinDirection){
    analogWrite(right_dir,255);
  } else {
    analogWrite(right_dir,0);
  }
  if (stop_dir_left == false && valueDirection < 0 && enablePinDirection){
    analogWrite(left_dir,255);
  } else {
    analogWrite(left_dir,0);
  }
}

void brakeController(){
  if (moveBrake){
    if (positiveRunMotor){
      for (int x = 0; x < positionMissing; x++) {
        digitalWrite(STEP_MPP1, HIGH);
        digitalWrite(STEP_MPP2, HIGH);
        delayMicroseconds(500);
        digitalWrite(STEP_MPP1, LOW);
        digitalWrite(STEP_MPP2, LOW);
        delayMicroseconds(300);
        positionCurrent++;
      }
      moveBrake = false;
    } else {
      for (int x = 0; x > positionMissing; x--) {
        digitalWrite(STEP_MPP1, HIGH);
        digitalWrite(STEP_MPP2, HIGH);
        delayMicroseconds(500);
        digitalWrite(STEP_MPP1, LOW);
        digitalWrite(STEP_MPP2, LOW);
        delayMicroseconds(300);
        positionCurrent--;
      }
      moveBrake = false;
    }
  }
}

void accelCallback( const std_msgs::Int16& data_received ) {
  newAcceleratorValue = data_received.data + minAcceleratorValue;
  if (newAcceleratorValue>= minAcceleratorValue && newAcceleratorValue < maxAcceleratorValue){
    acceleratorValue = newAcceleratorValue;
    enableAccelerator = true;
  } else {
    enableAccelerator = false;
    nh.logwarn("Valor de acceleracion recibida invalida");
  }
}

void dirCallback( const control_dir_msgs::dir_data& data_received) {
  valueDirection = data_received.range;
  if (data_received.activate) {
    digitalWrite(enableDirPin, HIGH);
    enablePinDirection = true;
  } else {
    valueDirection = 0;
    digitalWrite(enableDirPin, LOW);
    enablePinDirection = false;
    analogWrite(right_dir,0);
    analogWrite(left_dir,0);
  }
}

void brakeCallback( const brake_controller_msgs::brake_control& data_received ) {
  positionToReach = (data_received.position/100)*100;
  if (positionToReach >= positionCurrent) {
    digitalWrite(DIR_MPP1, HIGH);
    digitalWrite(DIR_MPP2, HIGH);
    positiveRunMotor = true;   
  } else {
    digitalWrite(DIR_MPP1, LOW);
    digitalWrite(DIR_MPP2, LOW);
    positiveRunMotor = false;
  }
  positionMissing = (positionToReach - positionCurrent);
  moveBrake = true;
}