#include <ros.h>
#include <puma_brake_msgs/BrakeCmd.h>
#include <puma_direction_msgs/DirectionCmd.h>
#include <puma_arduino_msgs/StatusArduino.h>
#include <puma_arduino_msgs/StatusTacometer.h>
#include <std_msgs/Int16.h>

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
const int right_dir = 4;
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

// Variable tacometro
const int tacometroPin = 2;
int pulsoContador = 0;
unsigned long last_time = 0;
int limit_time = 500;

// Variables ROS
ros::NodeHandle nh;

void brakeCallback( const puma_brake_msgs::BrakeCmd& data_received );
ros::Subscriber<puma_brake_msgs::BrakeCmd> brake_sub("puma/brake/command", brakeCallback);

void dirCallback( const puma_direction_msgs::DirectionCmd& data_received);
ros::Subscriber<puma_direction_msgs::DirectionCmd> dir_sub("puma/direction/command", dirCallback);

void accelCallback( const std_msgs::Int16& data_received );
ros::Subscriber<std_msgs::Int16> accel_sub("puma/accelerator/commmand", accelCallback);

puma_arduino_msgs::StatusArduino status_msg;
ros::Publisher arduinoStatusPub("puma/arduino/status", &status_msg);

puma_arduino_msgs::StatusTacometer tacometer_pub;
ros::Publisher tacometerStatusPub("puma/sensors/tacometer", &tacometer_pub);

void setup() {
  // Config ros
  nh.initNode();
  nh.subscribe(brake_sub);
  nh.subscribe(dir_sub);
  nh.subscribe(accel_sub);
  nh.advertise(arduinoStatusPub);
  nh.advertise(tacometerStatusPub);
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
  // Config tacometro
  pinMode(tacometroPin, INPUT_PULLUP); 
  attachInterrupt(digitalPinToInterrupt(tacometroPin), countPulse, RISING); 

  // Write topics to msg status
  status_msg.topic_brake = "puma/brake/command";
  status_msg.topic_dir = "puma/direction/command";
  status_msg.topic_accel = "puma/accelerator/command";
}

void loop() {
  // Controladores
  accelController();
  dirController();
  brakeController();
  // Publicar pulsos tacometro
  unsigned long current_time = millis();
  if (current_time - last_time >= limit_time) {
    noInterrupts();
    tacometer_pub.pulsos = pulsoContador;
    tacometer_pub.time_millis = limit_time;
    tacometerStatusPub.publish(&tacometer_pub);
    pulsoContador = 0;
    interrupts();

    last_time = current_time;
  }
  // Publish msg
  publishMsgStatus();
  nh.spinOnce();
  delay(1);
}

void countPulse() {
  pulsoContador++;
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

void dirCallback( const puma_direction_msgs::DirectionCmd& data_received) {
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

void brakeCallback( const puma_brake_msgs::BrakeCmd& data_received ) {
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