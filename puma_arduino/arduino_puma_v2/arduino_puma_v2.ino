#include <ros.h>
#include <puma_brake_msgs/BrakeCmd.h>
#include <puma_direction_msgs/DirectionCmd.h>
#include <puma_arduino_msgs/StatusArduino.h>
#include <puma_arduino_msgs/StatusTachometer.h>
#include <std_msgs/Int16.h>

#define Analog2Rad 0.006135937
#define Rad2Degree 57.2958279

// Variables freno
const int DIR_MPP1 = 8;
const int STEP_MPP1 = 9;
const int DIR_MPP2 = 10;
const int STEP_MPP2 = 11;
bool enableBrake = false;
const int pinReaderSwitchA = 29;
const int pinReaderSwitchB = 28;
const int posFinishBrake = 400;
int currentPosBrake = 0;
bool initialCalibration = true;

// Variables direccion
const int sensorPositionPin = A2;
const int enableDirPin = 14;
const int right_dir = 3;
const int left_dir = 4;
// 45 grados limite
//const int limit_min = 263;
//const int limit_max = 521;
// 30 grados limite
const int limit_min = 310;
const int limit_max = 480;
const int zero_position = 392;

float angleGoal = 0; // Is save angle goal in rads
int positionGoal = 0;
int sensorPositionValue = 0;
float angleCurrent = 0;
bool stop_dir_right = true;
bool stop_dir_left = true;
bool enablePinDirection = false;

// Variables acelerador
const int acceleratorPin = 12;
const int minAcceleratorValue = 43;
const int maxAcceleratorValue = 169;
int acceleratorValue = minAcceleratorValue;
int newAcceleratorValue = 0;
bool enableAccelerator = false;

// Variable tacometro
const int tacometroPin = 2;
volatile unsigned long pulsoContador = 0;
unsigned long last_time = 0;
int limit_time = 500;

// Variables ROS
ros::NodeHandle nh;

void brakeCallback( const puma_brake_msgs::BrakeCmd& data_received );
ros::Subscriber<puma_brake_msgs::BrakeCmd> brake_sub("puma/brake/command", brakeCallback);

void dirCallback( const puma_direction_msgs::DirectionCmd& data_received);
ros::Subscriber<puma_direction_msgs::DirectionCmd> dir_sub("puma/direction/command", dirCallback);

void accelCallback( const std_msgs::Int16& data_received );
ros::Subscriber<std_msgs::Int16> accel_sub("puma/accelerator/command", accelCallback);

puma_arduino_msgs::StatusArduino status_msg;
ros::Publisher arduinoStatusPub("puma/arduino/status", &status_msg);

puma_arduino_msgs::StatusTachometer tacometer_pub;
ros::Publisher tacometerStatusPub("puma/sensors/tachometer", &tacometer_pub);

void setup() {
  // Config ros
  //Serial.begin(115200);
  nh.getHardware()->setBaud(115200);
  nh.initNode();
  nh.subscribe(brake_sub);
  nh.subscribe(dir_sub);
  nh.subscribe(accel_sub);
  nh.advertise(arduinoStatusPub);
  nh.advertise(tacometerStatusPub);
  nh.loginfo("Inicializando Arduino");
  // Config pinMode brake
  pinMode(pinReaderSwitchA, INPUT);
  pinMode(pinReaderSwitchB, INPUT);
  pinMode(DIR_MPP1, OUTPUT);
  pinMode(DIR_MPP2, OUTPUT);
  pinMode(STEP_MPP1, OUTPUT);
  pinMode(STEP_MPP2, OUTPUT);
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
  nh.spinOnce();
  // int readSwitchA = digitalRead(pinReaderSwitchA);
  // int readSwitchB = digitalRead(pinReaderSwitchB);
  // if (readSwitchA==1) {
  //   Serial.println("switch A high");
  // } else {
  //   Serial.println("switch A low");
  //   useBrake(false,true);
  // }
  // if (readSwitchB==1) {
  //   Serial.println("switch B high");
  // } else {
  //   Serial.println("switch B low");
  //   useBrake(true,false);
  // }
  // Si ros esta activado
  if (nh.connected()) {
    // Cuando ejecute por primera vez
    if (initialCalibration) {calibrateBrakes();}
    // Controladores
    accelController();
    dirController();
    brakeController();

    // Publicar pulsos tacometro
    unsigned long current_time = millis();
    if (current_time - last_time >= limit_time) {
      noInterrupts();
      tacometer_pub.pulsos = pulsoContador;
      pulsoContador = 0;
      interrupts();
      tacometer_pub.time_millis = limit_time;
      tacometerStatusPub.publish(&tacometer_pub);

      last_time = current_time;
    }
    // Publish msg
    publishMsgStatus();
  } else {  // Si ros esta desactivado o fue apagado
    analogWrite(acceleratorPin, minAcceleratorValue); // Colocar accelerador al minimo
    digitalWrite(enableDirPin, LOW); // Desactivar direccion
  }
}

void calibrateBrakes() {
  // Calibrar frenos
  digitalWrite(DIR_MPP1, LOW);
  digitalWrite(DIR_MPP2, LOW);
  int readSwitchA = digitalRead(pinReaderSwitchA);
  int readSwitchB = digitalRead(pinReaderSwitchB);
  while (readSwitchA == 1) {
    useBrake(false,true);
    status_msg.is_moving_brake = true;
    readSwitchA = digitalRead(pinReaderSwitchA);
  }
  delay(100);
  while (readSwitchB == 1) {
    useBrake(true,false);
    status_msg.is_moving_brake = true;
    readSwitchB = digitalRead(pinReaderSwitchB);
  }
  nh.loginfo("Calibracion inicial de frenos completada");
  initialCalibration = false;
}

void countPulse() {
  pulsoContador++;
}

void publishMsgStatus() {
  // Brake info
  status_msg.activate_brake = enableBrake;
  int readSwitchA = digitalRead(pinReaderSwitchA);
  int readSwitchB = digitalRead(pinReaderSwitchB);
  if (readSwitchA == 1) {
    status_msg.switch_a = true;
  } else {
    status_msg.switch_a = false;
  }
  if (readSwitchB == 1) {
    status_msg.switch_b = true;
  } else {
    status_msg.switch_b = false;
  }
  // Dir info
  status_msg.current_position_dir = sensorPositionValue;
  status_msg.current_angle_rad_dir = sensorPositionValue*Analog2Rad;
  status_msg.current_angle_deg_dir = sensorPositionValue*Analog2Rad*Rad2Degree;
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
  positionGoal = int(angleGoal / Analog2Rad) + zero_position;
  angleCurrent = (sensorPositionValue - zero_position) * Analog2Rad;

  // Checking if can turn on direction
  if(sensorPositionValue > limit_min){
    stop_dir_right = false;
  } else {
    if (!stop_dir_right) {
      nh.logwarn("Direccion a la derecha maxima alcanzada");
    }
    stop_dir_right = true;
  }
  if(sensorPositionValue < limit_max){
    stop_dir_left = false;
  } else {  
    if (!stop_dir_left) {
      nh.logwarn("Direccion a la izquierda maxima alcanzada");
    }
    stop_dir_left = true;
  }

  bool is_missing_goal = (positionGoal - 10 > sensorPositionValue ) || (positionGoal + 10 < sensorPositionValue );
  bool is_diff_angle_positiv = (angleCurrent < angleGoal) && (sensorPositionValue <= limit_max) && (sensorPositionValue >= limit_min-20);
  bool is_diff_angle_negativ = (angleCurrent > angleGoal) && (sensorPositionValue <= limit_max+20) && (sensorPositionValue >= limit_min);

  if (is_diff_angle_negativ && enablePinDirection && is_missing_goal){
    analogWrite(left_dir,0);
    analogWrite(right_dir,150);
  } else if (is_diff_angle_positiv && enablePinDirection && is_missing_goal){
    analogWrite(right_dir,0);
    analogWrite(left_dir,150);
  } else {
    analogWrite(right_dir,0);
    analogWrite(left_dir,0);
  }
}

void brakeController(){
  int readSwitchA = digitalRead(pinReaderSwitchA);
  int readSwitchB = digitalRead(pinReaderSwitchB);
  if (enableBrake && currentPosBrake != posFinishBrake) {
    for (int x = 0; x<posFinishBrake; x++){
      useBrake(true,true);
      status_msg.is_moving_brake = true;
    }
    currentPosBrake = posFinishBrake;
  } else if (!enableBrake) {
    if(readSwitchA == 0 && readSwitchB == 0) {
      for(int i = 0; i <5; i++) {
        useBrake(true,true);
        status_msg.is_moving_brake = true;
        readSwitchA = digitalRead(pinReaderSwitchA);
        readSwitchB = digitalRead(pinReaderSwitchB);
      }
    }
    else if(readSwitchA == 0) {
      for(int i = 0; i<5; i++) {
        useBrake(false, true);
        status_msg.is_moving_brake = true;
        readSwitchA = digitalRead(pinReaderSwitchA);
      }
    } else if(readSwitchB == 0) {
      for(int i=0; i<5; i++) {
      useBrake(true, false);
      status_msg.is_moving_brake = true;
      readSwitchB = digitalRead(pinReaderSwitchB);
      }
    }
    currentPosBrake = 0;
  }
  status_msg.is_moving_brake = false;
}

void useBrake(bool step1, bool step2) {
  if (step1 && step2) {
    digitalWrite(STEP_MPP1, HIGH);
    digitalWrite(STEP_MPP2, HIGH);
    delayMicroseconds(500);
    digitalWrite(STEP_MPP1, LOW);
    digitalWrite(STEP_MPP2, LOW);
    delayMicroseconds(300);
  } else if (step1 && !step2) {
    digitalWrite(STEP_MPP1, HIGH);
    delayMicroseconds(500);
    digitalWrite(STEP_MPP1, LOW);
    delayMicroseconds(300);
  } else if (!step1 && step2) {
    digitalWrite(STEP_MPP2, HIGH);
    delayMicroseconds(500);
    digitalWrite(STEP_MPP2, LOW);
    delayMicroseconds(300);
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
  angleGoal = data_received.angle;
  if (data_received.activate) {
    digitalWrite(enableDirPin, HIGH);
    enablePinDirection = true;
  } else {
    angleGoal = 0;
    digitalWrite(enableDirPin, LOW);
    enablePinDirection = false;
    analogWrite(right_dir,0);
    analogWrite(left_dir,0);
  }
}

void brakeCallback( const puma_brake_msgs::BrakeCmd& data_received ) {
  enableBrake = data_received.activate_brake;
  if (enableBrake) {
    digitalWrite(DIR_MPP1, HIGH);
    digitalWrite(DIR_MPP2, HIGH);   
  } else {
    digitalWrite(DIR_MPP1, LOW);
    digitalWrite(DIR_MPP2, LOW);
  }
}