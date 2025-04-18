#include <ros.h>
#include <puma_msgs/DirectionCmd.h>
#include <puma_msgs/StatusArduino.h>
#include <puma_msgs/StatusTachometer.h>
#include <puma_msgs/UpdateParamArduino.h>
#include <puma_msgs/Log.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int16.h>
#include <std_msgs/String.h>

#define ANALOG_TO_RAD 0.006135937
#define RAD_TO_DEG 57.2958279
#define TIME_PUBLISH_IMU 30 // freq 33
#define TIME_PUBLISH_STATUS 1500 // freq 0.67

/* Señal seguridad */
const int SECURITY_PIN = 45;
bool enableSecurity = false;
/* Mode selector */
unsigned long lastTimeModeReceived = 0;
String posibleModes[] = {"joystick", "idle", "web", "navegacion"};
String latestMode = "idle";
bool isRunMode = false;
#define MILLIS_LIMIT_MODE 500
/* Variables freno */ 
const int STEP_FRONT_PIN = 9; //11
const int DIR_FRONT_PIN = 8; //10
const int STEP_REAR_PIN = 11; //9
const int DIR_REAR_PIN = 10; //8
const int SWITCH_BRAKE_FRONT_OFF = 26;
const int SWITCH_BRAKE_FRONT_ON = 28;
const int SWITCH_BRAKE_REAR_OFF = 36;
const int SWITCH_BRAKE_REAR_ON = 38;
bool enableBrake = false;
bool isSendedLogBrake = false;
/* Control Freno */
unsigned long initTimeBrakeCmd = 0;
int LIMIT_TIME_BRAKE = 1400;
int LIMIT_TIME_BRAKE_OFF = 1500;
#define PWM_FRONT_BRAKE 100
#define PWM_REAR_BRAKE 100

int countChangeFrontBrake = 0;
/* TESTEAR */
unsigned long lastBrakeChangeTime = 0;
const unsigned long DEBOUNCE_DELAY = 200;    

struct {
  int turnOn;
  int turnOff;
} rotationBrakeFront, rotationBrakeRear;

struct {
  bool frontOff = false;
  bool frontOn = false;
  bool rearOff = false;
  bool rearOn = false;
} lastSwitchState;

struct {
  unsigned long frontOff = 0;
  unsigned long frontOn = 0;
  unsigned long rearOff = 0;
  unsigned long rearOn = 0;
} switchRisingTime;

struct {
  unsigned long FRONT_OFF = 300;
  unsigned long FRONT_ON = 300;
  unsigned long REAR_OFF = 300;
  unsigned long REAR_ON = 300;
} DELAY_BRAKE;


unsigned long last_time_front = 0;
unsigned long last_time_rear = 0;


/* Variables direccion */
const int SENSOR_DIRECTION_PIN = A3;
const int RIGHT_DIRECTION_PIN = 3;
const int LEFT_DIRECTION_PIN = 4;
const int ENABLE_DIRECTION_PIN = 5;
// 45 grados limite
int LIMIT_DIR_ANALOG_MIN = 296; //263
int LIMIT_DIR_ANALOG_MAX = 560; //521
int ZERO_DIR_ANALOG = 425;  //392
#define PWM_DIRECTION 140
#define TOLERANCE_SENSOR 5

float angleGoal = 0; // Is save angle goal in rads
float angleCurrent = 0;
int positionGoal = 0;
int sensorPositionValue = 0;
bool stopDirectionRight = true;
bool stopDirectionLeft = true;
bool enablePinDirection = false;

/* Variables acelerador */ 
const int ACCELERATOR_PIN = 12;
int PWM_MIN_ACCELERATOR = 43; // 0.843v
int PWM_MAX_ACCELERATOR = 83; // 1.625v
// const int PWM_MAX_ACCELERATOR = 90; // 1.765v
int acceleratorValue = PWM_MIN_ACCELERATOR;
int newAcceleratorValue = 0;
bool enableAccelerator = false;

/* Variable tacometro */ 
const int TACHOMETER_PIN = 2;
const int LIMIT_TIME_TACHOMETER = 500;
volatile unsigned long pulsoContador = 0;
volatile unsigned long lastDebounceTime = 0;
unsigned long lastTimeTachometer = 0;
const unsigned long debounceDelay = 30;

/* Tiempos para publicar */ 
unsigned long lastTimeStatus = 0;
unsigned long lastTimeImu = 0;
unsigned long lastTimeSecurity = 0;
unsigned long lastTimeLog = 0;
unsigned long lastTimeModePublish = 0;

// IMU
#include "ICM_20948.h"
#define AD0_VAL 0
#define WIRE_PORT Wire
#include <puma_msgs/ImuData.h>
ICM_20948_I2C myICM;

/* Variables ROS */ 
ros::NodeHandle nh;

void brakeCallback( const std_msgs::Bool& data_received );
void directionCallback( const puma_msgs::DirectionCmd& data_received);
void acceleratorCallback( const std_msgs::Int16& data_received );
void modeSelectorCallback( const std_msgs::String& data_received);
ros::Subscriber<std_msgs::Bool> brake_sub("puma/control/brake", brakeCallback);
ros::Subscriber<puma_msgs::DirectionCmd> dir_sub("puma/control/direction", directionCallback);
ros::Subscriber<std_msgs::Int16> accel_sub("puma/control/accelerator", acceleratorCallback);
ros::Subscriber<std_msgs::String> mode_sub("puma/control/current_mode", modeSelectorCallback);

void reconfigureCb( const puma_msgs::UpdateParamArduino& param);
ros::Subscriber<puma_msgs::UpdateParamArduino> param_sub("puma/arduino/param_mega", reconfigureCb);

puma_msgs::StatusArduino status_msg;
puma_msgs::StatusTachometer tacometer_msg;
puma_msgs::Log log_msg;
puma_msgs::ImuData icm20948_msg;
ros::Publisher arduinoStatusPub("puma/arduino/status", &status_msg);
ros::Publisher tacometerStatusPub("puma/sensors/tachometer", &tacometer_msg);
ros::Publisher icm20948Pub("puma/sensors/icm20948/raw", &icm20948_msg);
ros::Publisher logInfoPub("puma/logs/add_log", &log_msg);

void setup() {
  /* Configuracion ROS */
  configRos();
  /* Configuracion de pines */
  configPinMode();
  attachInterrupt(digitalPinToInterrupt(TACHOMETER_PIN), countPulse, RISING); 
  /* Config i2c */
  WIRE_PORT.begin();
  WIRE_PORT.setClock(400000);
  /* Configuracion IMU y Mag */
  configImuMag();
}

void loop() {
  // Si ros esta activado
  if (nh.connected()) {
    readSecurityAndControlMode();
    /* Controladores */ 
    acceleratorController();
    directionController();
    brakeController();
    /* Publicadores */
    publishTachometer();
    publishImuMag();
    publishMsgStatus();
    publishSecurityLog();
    publishModeLog();
  } else {  /* Si ros esta desactivado o fue apagado */
    deactivateControl();
  }
  nh.spinOnce();
}

void configRos() {
  nh.getHardware()->setBaud(500000);
  nh.initNode();
  nh.subscribe(brake_sub);
  nh.subscribe(dir_sub);
  nh.subscribe(accel_sub);
  nh.subscribe(mode_sub);
  nh.subscribe(param_sub);
  nh.advertise(arduinoStatusPub);
  nh.advertise(tacometerStatusPub);
  nh.advertise(icm20948Pub);
  nh.advertise(logInfoPub);
  /* Escribir tópicos en el mensaje de estado */
  status_msg.brake.topic = "puma/control/brake";
  status_msg.direction.topic = "puma/control/direction";
  status_msg.accelerator.topic = "puma/control/accelerator";
  /* Log info */
  log_msg.node = "arduino_mega";
}

void configPinMode() {
   /* Pin de seguridad */
  pinMode(SECURITY_PIN, INPUT);
  /* Switches brake detection */
  pinMode(SWITCH_BRAKE_FRONT_OFF, INPUT);
  pinMode(SWITCH_BRAKE_FRONT_ON, INPUT);
  pinMode(SWITCH_BRAKE_REAR_OFF, INPUT);
  pinMode(SWITCH_BRAKE_REAR_ON, INPUT);
  /* Pines control de frenos */
  pinMode(DIR_FRONT_PIN, OUTPUT);
  pinMode(DIR_REAR_PIN, OUTPUT);
  pinMode(STEP_FRONT_PIN, OUTPUT);
  pinMode(STEP_REAR_PIN, OUTPUT);
  /* Pines control de dirección */
  pinMode(ENABLE_DIRECTION_PIN, OUTPUT);
  pinMode(RIGHT_DIRECTION_PIN, OUTPUT);
  pinMode(LEFT_DIRECTION_PIN, OUTPUT);
  /* Pin de acelerador */
  pinMode(ACCELERATOR_PIN, OUTPUT);
  /* Pin Tacometro */
  pinMode(TACHOMETER_PIN, INPUT_PULLUP); 
  /* Revisar estado de los frenos */
  lastSwitchState.frontOff = digitalRead(SWITCH_BRAKE_FRONT_OFF);
  lastSwitchState.frontOn = digitalRead(SWITCH_BRAKE_FRONT_ON);
  lastSwitchState.rearOff = digitalRead(SWITCH_BRAKE_REAR_OFF);
  lastSwitchState.rearOn = digitalRead(SWITCH_BRAKE_REAR_ON);
  rotationBrakeFront.turnOn = LOW;
  rotationBrakeFront.turnOff = HIGH;
  rotationBrakeRear.turnOn = HIGH;
  rotationBrakeRear.turnOff = LOW;
}

void configImuMag() {
  bool initialized = false;
  while (!initialized)
  {
    myICM.begin(WIRE_PORT, AD0_VAL);

    if (myICM.status != ICM_20948_Stat_Ok)
      delay(500);
    else 
      initialized = true;
  }
  myICM.initializeDMP();
  // Sensores a medir
  myICM.enableDMPSensor(INV_ICM20948_SENSOR_ACCELEROMETER);
  myICM.enableDMPSensor(INV_ICM20948_SENSOR_GYROSCOPE);
  myICM.enableDMPSensor(INV_ICM20948_SENSOR_MAGNETIC_FIELD_UNCALIBRATED);
  // Configurar frecuencia DMP (ej. 100Hz)
  myICM.setDMPODRrate(DMP_ODR_Reg_Accel, 40);
  myICM.setDMPODRrate(DMP_ODR_Reg_Gyro, 40);
  myICM.setDMPODRrate(DMP_ODR_Reg_Cpass, 40);
  myICM.enableDMP();
  myICM.resetDMP();
}

void readSecurityAndControlMode() {
  int readSecurity = digitalRead(SECURITY_PIN);
  if (enableSecurity != readSecurity && readSecurity == 1) {
    initTimeBrakeCmd = 0;
  } 
  enableSecurity = (readSecurity == 1);
  /* Revisar deteccion de modo */
  isRunMode = millis() - lastTimeModeReceived < MILLIS_LIMIT_MODE ? isRunMode : false;
}

void deactivateControl() {
  /* Acelerador al minimo */
  analogWrite(ACCELERATOR_PIN, PWM_MIN_ACCELERATOR); 
  /* Desactivar direccion */ 
  digitalWrite(ENABLE_DIRECTION_PIN, LOW); 
  analogWrite(RIGHT_DIRECTION_PIN, 0);
  analogWrite(LEFT_DIRECTION_PIN, 0);
  /* Apagar frenos */ 
  analogWrite(STEP_FRONT_PIN, 0);
  analogWrite(STEP_REAR_PIN, 0);
}

void publishTachometer() {
  unsigned long currentTime = millis();
  if (currentTime - lastTimeTachometer >= LIMIT_TIME_TACHOMETER) {
    noInterrupts();
    tacometer_msg.pulsos = pulsoContador;
    pulsoContador = 0;
    interrupts();
    tacometer_msg.time_millis = LIMIT_TIME_TACHOMETER;
    tacometerStatusPub.publish(&tacometer_msg);
    lastTimeTachometer = currentTime;
  }
}

void acceleratorController(){
  /* Revisar siempre el estado de la señal de seguridad */
  if (!enableSecurity) {
    /* Revisar si ha recibido algun valor por topicos */
    if (enableAccelerator && isRunMode) {
      analogWrite(ACCELERATOR_PIN, acceleratorValue);
    } else {
      analogWrite(ACCELERATOR_PIN, PWM_MIN_ACCELERATOR);
      acceleratorValue = PWM_MIN_ACCELERATOR;
    }
  } else {
    analogWrite(ACCELERATOR_PIN, PWM_MIN_ACCELERATOR);
    acceleratorValue = PWM_MIN_ACCELERATOR;
  }
}

void directionController(){
  sensorPositionValue = analogRead(SENSOR_DIRECTION_PIN);
  if (sensorPositionValue < LIMIT_DIR_ANALOG_MIN -30 || sensorPositionValue > LIMIT_DIR_ANALOG_MAX + 30) {
    static unsigned long lastTimeLog = 0;
    unsigned long currentTime = millis();
    if (currentTime - lastTimeLog > 30000) {
      lastTimeLog = currentTime;
      log_msg.level = 2;
      log_msg.content = "Error en la lectura del sensor de direccion. Revisar conexión.";
      logInfoPub.publish(&log_msg);
    }
  } else {
    /* En caso de activarse la señal de seguridad, se define angulo objetivo en 0 
      Tambien en caso de detectar que no ha habido deteccion de modo, se define angulo en 0*/
    positionGoal = !isRunMode ? ZERO_DIR_ANALOG : int(angleGoal / ANALOG_TO_RAD) + ZERO_DIR_ANALOG;
    positionGoal = enableSecurity ? ZERO_DIR_ANALOG : positionGoal;
    angleCurrent = (sensorPositionValue - ZERO_DIR_ANALOG) * ANALOG_TO_RAD;

    /* Revisar si se esta en los límites */
    stopDirectionRight = sensorPositionValue < LIMIT_DIR_ANALOG_MIN;
    stopDirectionLeft = sensorPositionValue > LIMIT_DIR_ANALOG_MAX;

    /* Comprobar si se ha llegado a la direccion deseada */
    bool is_in_range_goal = (positionGoal + TOLERANCE_SENSOR > sensorPositionValue ) && (positionGoal - TOLERANCE_SENSOR < sensorPositionValue );
    bool is_in_range_max = (positionGoal + TOLERANCE_SENSOR > LIMIT_DIR_ANALOG_MAX ) && (positionGoal - TOLERANCE_SENSOR < LIMIT_DIR_ANALOG_MAX );
    bool is_in_range_min = (positionGoal + TOLERANCE_SENSOR > LIMIT_DIR_ANALOG_MIN ) && (positionGoal - TOLERANCE_SENSOR < LIMIT_DIR_ANALOG_MIN );
    bool is_in_max = (positionGoal > angleCurrent && is_in_range_max);
    bool is_in_min = (positionGoal < angleCurrent && is_in_range_min);
    if (is_in_range_goal || is_in_max || is_in_min){
      analogWrite(RIGHT_DIRECTION_PIN,0);
      analogWrite(LEFT_DIRECTION_PIN,0);
      return;
    }
    /* Comprobar si se debe mover a la izquierda */
    bool is_diff_angle_positiv = (angleCurrent < angleGoal) && (sensorPositionValue <= LIMIT_DIR_ANALOG_MAX);
    if (is_diff_angle_positiv && enablePinDirection) {
      analogWrite(RIGHT_DIRECTION_PIN,0);
      analogWrite(LEFT_DIRECTION_PIN,PWM_DIRECTION);
      return;
    }
    /* Comprobar si se debe mover a la derecha */
    bool is_diff_angle_negativ = (angleCurrent > angleGoal) && (sensorPositionValue >= LIMIT_DIR_ANALOG_MIN);
    if (is_diff_angle_negativ && enablePinDirection ){
      analogWrite(RIGHT_DIRECTION_PIN,PWM_DIRECTION);
      analogWrite(LEFT_DIRECTION_PIN,0);
      return;
    }
  }
}

void brakeController(){
  
  bool currentFrontOff = digitalRead(SWITCH_BRAKE_FRONT_OFF);
  bool currentRearOff = digitalRead(SWITCH_BRAKE_REAR_OFF);

  /* Detectar flancos de subida*/
  if (currentFrontOff && !lastSwitchState.frontOff) switchRisingTime.frontOff = millis();
  
  if (currentRearOff && !lastSwitchState.rearOff) switchRisingTime.rearOff = millis();


  if (millis() - lastBrakeChangeTime < DEBOUNCE_DELAY) return;

  if (initTimeBrakeCmd == 0) initTimeBrakeCmd = millis();

  bool check_to_activated = (enableSecurity || enableBrake || !isRunMode);
  
  frontBrakeController(currentFrontOff, check_to_activated);
  rearBrakeController(currentRearOff, check_to_activated);

}

void rearBrakeController(bool currentOff, bool activateMode) {
  if (activateMode) {

    if (digitalRead(SWITCH_BRAKE_REAR_ON)) {
      analogWrite(STEP_REAR_PIN, 0);
      return;
    }

    if ( millis() - initTimeBrakeCmd > LIMIT_TIME_BRAKE ) {
      analogWrite(STEP_REAR_PIN, 0);
      return;
    }

    digitalWrite(DIR_REAR_PIN, rotationBrakeRear.turnOn);
    analogWrite(STEP_REAR_PIN, PWM_REAR_BRAKE);
    
  } else {

    lastSwitchState.rearOff = currentOff;

    if (switchRisingTime.rearOff > 0 && (millis() - switchRisingTime.rearOff) < DELAY_BRAKE.REAR_OFF) {
      analogWrite(STEP_REAR_PIN, PWM_REAR_BRAKE);
      return;
    } else {
      switchRisingTime.rearOff = 0;
    }

    if (digitalRead(SWITCH_BRAKE_REAR_OFF) || millis() - initTimeBrakeCmd > LIMIT_TIME_BRAKE_OFF) {
      analogWrite(STEP_REAR_PIN, 0);
      return;
    }

    digitalWrite(DIR_REAR_PIN, rotationBrakeRear.turnOff);
    analogWrite(STEP_REAR_PIN, PWM_REAR_BRAKE);
  }
}

void frontBrakeController(bool currentOff, bool activateMode){
  if (activateMode) {

    if (digitalRead(SWITCH_BRAKE_FRONT_ON)) {
      analogWrite(STEP_FRONT_PIN, 0);
      return;
    }

    if ( millis() - initTimeBrakeCmd > LIMIT_TIME_BRAKE ) {
      analogWrite(STEP_FRONT_PIN, 0);
      return;
    }
    
    digitalWrite(DIR_FRONT_PIN, rotationBrakeFront.turnOn);
    analogWrite(STEP_FRONT_PIN, PWM_FRONT_BRAKE);
    
  } else {
    lastSwitchState.frontOff = currentOff;

    if (switchRisingTime.frontOff > 0 && (millis() - switchRisingTime.frontOff) < DELAY_BRAKE.FRONT_OFF) {
      analogWrite(STEP_FRONT_PIN, PWM_FRONT_BRAKE);
      return;
    } else {
      switchRisingTime.frontOff = 0;
    }

    if (currentOff || millis() - initTimeBrakeCmd > LIMIT_TIME_BRAKE_OFF) {
      analogWrite(STEP_FRONT_PIN, 0);
      return;
    }
  
    digitalWrite(DIR_FRONT_PIN, rotationBrakeFront.turnOff);
    analogWrite(STEP_FRONT_PIN, PWM_FRONT_BRAKE);
  }
}

void passLimitTimeBrake(bool cmdIsActivate) {
  analogWrite(STEP_FRONT_PIN, 0);
  analogWrite(STEP_REAR_PIN, 0);
  log_msg.level = 1;
  if (!isSendedLogBrake) {
    if (cmdIsActivate) {
      int readFrontOn   = digitalRead(SWITCH_BRAKE_FRONT_ON);
      int readRearOn    = digitalRead(SWITCH_BRAKE_REAR_ON);
      if (readFrontOn == 0 && readRearOn == 0) {
        log_msg.content = "No se logra detectar el accionar maximo del freno delantero y trasero.";
      } else if (readFrontOn == 0) {
        log_msg.content = "No se logra detectar el accionar maximo del freno delantero.";
      } else if (readRearOn == 0) {
        log_msg.content = "No se logra detectar el accionar maximo del freno trasero.";
      }
    } else {
      int readFrontOff  = digitalRead(SWITCH_BRAKE_FRONT_OFF);
      int readRearOff   = digitalRead(SWITCH_BRAKE_REAR_OFF);
      if (readFrontOff == 0 && readRearOff == 0) {
        log_msg.content = "No se logra detectar el accionar minimo del freno delantero y trasero.";
      } else if (readFrontOff == 0) {
        log_msg.content = "No se logra detectar el accionar minimo del freno delantero.";
      } else if (readRearOff == 0) {
        log_msg.content = "No se logra detectar el accionar minimo del freno trasero.";
      }
    }
    logInfoPub.publish(&log_msg);
    isSendedLogBrake = true;
  }
}

void countPulse() {
  unsigned long currentMillis = millis();
  if (currentMillis - lastDebounceTime >= debounceDelay) {
    pulsoContador++;  // Incrementamos el contador de pulsos
    lastDebounceTime = currentMillis;  // Actualizamos tiempo de debounce
  }
}

void publishSecurityLog() {
  if (enableSecurity) {
    unsigned long time = millis();
    if ( time - lastTimeSecurity > 60000) {
      log_msg.level = 2;
      log_msg.content = "Se encuentra activado la parada de emergencia.";
      lastTimeSecurity = time;
      logInfoPub.publish(&log_msg);
    }
  }
}

void publishModeLog() {
  if (!isRunMode) {
    unsigned long time = millis();
    if ( time - lastTimeModePublish > 30000) {
      log_msg.level = 1;
      log_msg.content = "No se ha detectado un modo de control en mas de 0.5 seg.";
      lastTimeModePublish = time;
      logInfoPub.publish(&log_msg);
    }
  }
}

void publishImuMag() {
  unsigned long currentTime = millis();
  if (currentTime - lastTimeImu >= TIME_PUBLISH_IMU) {
    /* Check if data read */
    lastTimeImu = currentTime;
    if (myICM.dataReady()) {
      myICM.getAGMT();
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
}

void publishMsgStatus() {
  unsigned long time = millis();
  if (time - lastTimeStatus >= TIME_PUBLISH_STATUS) {
    int readFrontOff  = digitalRead(SWITCH_BRAKE_FRONT_OFF);
    int readFrontOn   = digitalRead(SWITCH_BRAKE_FRONT_ON);
    int readRearOff   = digitalRead(SWITCH_BRAKE_REAR_OFF);
    int readRearOn    = digitalRead(SWITCH_BRAKE_REAR_ON);
    /* Brake info */
    status_msg.brake.activate = enableBrake;
    status_msg.brake.switch_front_on = readFrontOn == 1;
    status_msg.brake.switch_front_off = readFrontOff == 1;
    status_msg.brake.switch_rear_on = readRearOn == 1;
    status_msg.brake.switch_rear_off = readRearOff == 1;
    /* Direction info */
    status_msg.direction.analog_value = sensorPositionValue;
    status_msg.direction.radian_angle = (sensorPositionValue-ZERO_DIR_ANALOG)*ANALOG_TO_RAD;
    status_msg.direction.degree_angle = (sensorPositionValue-ZERO_DIR_ANALOG)*ANALOG_TO_RAD*RAD_TO_DEG;
    status_msg.direction.enable = enablePinDirection;
    status_msg.direction.is_limit_right= stopDirectionRight;
    status_msg.direction.is_limit_left = stopDirectionLeft;
    /* Accelerator info */
    status_msg.accelerator.enable = enableAccelerator;
    status_msg.accelerator.pwm = acceleratorValue;
    status_msg.accelerator.voltage_out = (acceleratorValue/255.0) * 5.0;
    /* Control info */
    status_msg.control.mode_signal_accept = isRunMode;
    status_msg.control.security_signal = enableSecurity;
    status_msg.control.mode_detected = latestMode.c_str();
    /* Param info */
    status_msg.param.limit_time_brake = LIMIT_TIME_BRAKE;
    status_msg.param.limit_time_brake_off = LIMIT_TIME_BRAKE_OFF;
    status_msg.param.delay_brake_rear_off = DELAY_BRAKE.REAR_OFF;
    status_msg.param.delay_brake_front_off = DELAY_BRAKE.FRONT_OFF;
    status_msg.param.limit_dir_analog_min = LIMIT_DIR_ANALOG_MIN;
    status_msg.param.limit_dir_analog_max = LIMIT_DIR_ANALOG_MAX;
    status_msg.param.zero_dir_analog = ZERO_DIR_ANALOG;
    status_msg.param.pwm_min_accelerator = PWM_MIN_ACCELERATOR;
    status_msg.param.pwm_max_accelerator = PWM_MAX_ACCELERATOR;
    arduinoStatusPub.publish(&status_msg);
    lastTimeStatus = millis(); // Reset time
  }
}

void acceleratorCallback( const std_msgs::Int16& data_received ) {
  newAcceleratorValue = data_received.data + PWM_MIN_ACCELERATOR;
  int verifyValue = max(PWM_MIN_ACCELERATOR, min(newAcceleratorValue, PWM_MAX_ACCELERATOR));

  acceleratorValue = verifyValue;
  enableAccelerator = true;
}

void directionCallback( const puma_msgs::DirectionCmd& data_received) {
  angleGoal = data_received.angle;
  if (data_received.activate) {
    digitalWrite(ENABLE_DIRECTION_PIN, HIGH);
    enablePinDirection = true;
  } else {
    angleGoal = 0;
    digitalWrite(ENABLE_DIRECTION_PIN, LOW);
    enablePinDirection = false;
    analogWrite(RIGHT_DIRECTION_PIN,0);
    analogWrite(LEFT_DIRECTION_PIN,0);
  }
}

void brakeCallback( const std_msgs::Bool& data_received ) {
  bool activateBrakeRos = data_received.data;
  if (activateBrakeRos != enableBrake) {
    initTimeBrakeCmd = 0;
    isSendedLogBrake = false;
    lastBrakeChangeTime = millis();
  }
  enableBrake = activateBrakeRos;
}

void modeSelectorCallback( const std_msgs::String& data_received) {
  lastTimeModeReceived = millis();
  latestMode = data_received.data;
  bool isCorrectMode = false;
  for (int i = 0; i < 4; i++) {
    isCorrectMode = posibleModes[i] == data_received.data;
    if (isCorrectMode) break;
  }
  isRunMode = isCorrectMode;
}

void reconfigureCb( const puma_msgs::UpdateParamArduino& param) {
  switch (param.param) {
    case 0:
      LIMIT_TIME_BRAKE = param.value;
      break;
    case 1:
      LIMIT_TIME_BRAKE_OFF = param.value;
      break;
    case 2:
      DELAY_BRAKE.REAR_OFF = param.value;
      break;
    case 3:
      DELAY_BRAKE.FRONT_OFF = param.value;
      break;
    case 4:
      LIMIT_DIR_ANALOG_MIN = param.value;
      break;
    case 5:
      LIMIT_DIR_ANALOG_MAX = param.value;
      break;
    case 6:
      ZERO_DIR_ANALOG = param.value;
      break;
    case 7:
      PWM_MIN_ACCELERATOR = param.value;
      break;
    case 8:
      PWM_MAX_ACCELERATOR = param.value;
      break;
    default:
      break;
  }
}