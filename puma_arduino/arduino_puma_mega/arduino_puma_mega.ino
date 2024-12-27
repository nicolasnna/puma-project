#include <ros.h>
#include <puma_msgs/DirectionCmd.h>
#include <puma_msgs/StatusArduino.h>
#include <puma_msgs/StatusTachometer.h>
#include <puma_msgs/Log.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int16.h>
#include <std_msgs/String.h>
#include "mpu6500.h"
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include <QMC5883LCompass.h>

#define ANALOG_TO_RAD 0.006135937
#define RAD_TO_DEG 57.2958279
#define TIME_PUBLISH_IMU 33 // freq 30.0
#define TIME_PUBLISH_STATUS 66 // freq 15

/* Señal seguridad */
const int SECURITY_PIN = 45;
bool enableSecurity = false;
/* Mode selector */
unsigned long lastTimeModeReceived = 0;
char *posibleModes[] = {"joystick", "idle", "web", "navegacion"};
bool isRunMode = false;
#define MILLIS_LIMIT_MODE 500
/* Variables freno */ 
const int STEP_FRONT_PIN = 11;
const int DIR_FRONT_PIN = 10;
const int STEP_REAR_PIN = 9;
const int DIR_REAR_PIN = 8;
const int SWITCH_BRAKE_FRONT_OFF = 26;
const int SWITCH_BRAKE_FRONT_ON = 28;
const int SWITCH_BRAKE_REAR_OFF = 36;
const int SWITCH_BRAKE_REAR_ON = 38;
bool enableBrake = false;
unsigned long initTimeBrakeCmd = 0;
#define LIMIT_TIME_BRAKE 3000
// OFFSET time -- en prueba
int TIME_FINISH_FRONT = 1100; // millis
int TIME_FINISH_REAR = 1100;
int EXTRA_TIME_FRONT = 450;
int EXTRA_TIME_REAR = 450;
#define PWM_FRONT_BRAKE 100
#define PWM_REAR_BRAKE 100

/* Variables direccion */
const int SENSOR_DIRECTION_PIN = A2;
const int RIGHT_DIRECTION_PIN = 3;
const int LEFT_DIRECTION_PIN = 4;
const int ENABLE_DIRECTION_PIN = 5;
// 45 grados limite
const int LIMIT_DIR_ANALOG_MIN = 291; //263
const int LIMIT_DIR_ANALOG_MAX = 549; //521
int ZERO_DIR_ANALOG = 420;  //392
#define PWM_DIRECTION 100

float angleGoal = 0; // Is save angle goal in rads
float angleCurrent = 0;
int positionGoal = 0;
int sensorPositionValue = 0;
bool stopDirectionRight = true;
bool stopDirectionLeft = true;
bool enablePinDirection = false;

/* Variables acelerador */ 
const int ACCELERATOR_PIN = 12;
const int PWM_MIN_ACCELERATOR = 43; // 0.843v
const int PWM_MAX_ACCELERATOR = 90; // 1.765v
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
unsigned long lastTimeModePublish = 0;
unsigned long lastTimeSwitchFrontBrake = 0;
unsigned long lastTimeSwitchRearBrake = 0;
// Imu objects
QMC5883LCompass compass;
bfs::Mpu6500 imu;

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

puma_msgs::StatusArduino status_msg;
puma_msgs::StatusTachometer tacometer_msg;
puma_msgs::Log log_msg;
sensor_msgs::Imu imu_msg;
sensor_msgs::MagneticField compass_msg;
ros::Publisher arduinoStatusPub("puma/arduino/status", &status_msg);
ros::Publisher tacometerStatusPub("puma/sensors/tachometer", &tacometer_msg);
ros::Publisher imuRawPub("puma/sensors/imu/raw", &imu_msg);
ros::Publisher compassRawPub("puma/sensors/magnetometer", &compass_msg);
ros::Publisher logInfoPub("puma/logs/add_log", &log_msg);

void setup() {
  /* Configuracion ROS */
  // nh.getHardware()->setBaud(115200);
  nh.getHardware()->setBaud(250000);
  nh.initNode();
  nh.subscribe(brake_sub);
  nh.subscribe(dir_sub);
  nh.subscribe(accel_sub);
  nh.advertise(arduinoStatusPub);
  nh.advertise(tacometerStatusPub);
  nh.advertise(imuRawPub);
  nh.advertise(compassRawPub);
  nh.advertise(logInfoPub);
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
  digitalWrite(ENABLE_DIRECTION_PIN, LOW);
  /* Pin de acelerador */
  pinMode(ACCELERATOR_PIN, OUTPUT);
  /* Pin de tacometro */
  pinMode(TACHOMETER_PIN, INPUT_PULLUP); 
  attachInterrupt(digitalPinToInterrupt(TACHOMETER_PIN), countPulse, RISING); 
  /* Config i2c */
  Wire.begin();
  Wire.setClock(400000);
  /* IMU */
  imu.Config(&Wire, bfs::Mpu6500::I2C_ADDR_PRIM);
  imu.Begin();
  imu.ConfigSrd(19);
  imu.ConfigAccelRange(bfs::Mpu6500::ACCEL_RANGE_4G);
  imu.ConfigGyroRange(bfs::Mpu6500::GYRO_RANGE_500DPS);
  imu.ConfigDlpfBandwidth(bfs::Mpu6500::DLPF_BANDWIDTH_20HZ);
  /* Compass Magnetometer */ 
  compass.init();
  compass.setSmoothing(10, true);
  compass.setCalibrationOffsets(177.00, 155.00, -70.00);
  compass.setCalibrationScales(0.72, 0.72, 4.83);
  imu_msg.header.frame_id = compass_msg.header.frame_id = "gps_link";
  /* Write topics to msg status */ 
  status_msg.topic_brake = "puma/control/brake";
  status_msg.topic_dir = "puma/control/direction";
  status_msg.topic_accel = "puma/control/accelerator";
  /* Log info */
  log_msg.node =  "arduino_mega";
}

void loop() {
  // Si ros esta activado
  if (nh.connected()) {
    /* Revisar señal de seguridad */
    int readSecurity = digitalRead(SECURITY_PIN);
    enableSecurity = readSecurity == 1;
    /* Revisar deteccion de modo 
      Mantener bool si esta dentro del tiempo limite, si no indicar falso  */
    isRunMode = millis() - lastTimeModeReceived < MILLIS_LIMIT_MODE ? isRunMode : false;
    /* Controladores */ 
    acceleratorController();
    directionController();
    brakeController();

    // Publicar pulsos tacometro
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
    /* Publicar Imu y Mag */
    publishImuMag();
    /* Publicar estado arduino */
    publishMsgStatus();
    /* Publicar log de la señal de seguridad */
    publishSecurityLog();
    /* Publicar log en caso de no detectar modo */
    publishModeLog();
  } else {  /* Si ros esta desactivado o fue apagado */
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
  nh.spinOnce();
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
  if (sensorPositionValue < LIMIT_DIR_ANALOG_MIN -10 || sensorPositionValue > LIMIT_DIR_ANALOG_MAX + 10) {
    log_msg.level = 2;
    log_msg.content = "Error en la lectura del sensor de direccion. Revisar conexión.";
  } else {
    /* En caso de activarse la señal de seguridad, se define angulo objetivo en 0 
      Tambien en caso de detectar que no ha habido deteccion de modo, se define angulo en 0*/
    positionGoal =  enableSecurity || !isRunMode ? ZERO_DIR_ANALOG : int(angleGoal / ANALOG_TO_RAD) + ZERO_DIR_ANALOG;
    angleCurrent = (sensorPositionValue - ZERO_DIR_ANALOG) * ANALOG_TO_RAD;

    /* Revisar si se esta en los límites */
    if(sensorPositionValue > LIMIT_DIR_ANALOG_MIN){
      stopDirectionRight = false;
    } else {
      if (!stopDirectionRight) {
        nh.logwarn("Direccion a la derecha maxima alcanzada");
      }
      stopDirectionRight = true;
    }
    if(sensorPositionValue < LIMIT_DIR_ANALOG_MAX){
      stopDirectionLeft = false;
    } else {  
      if (!stopDirectionLeft) {
        nh.logwarn("Direccion a la izquierda maxima alcanzada");
      }
      stopDirectionLeft = true;
    }

    /* Comprobar direccion a realizar giro y si es necesario */
    bool is_missing_goal = (positionGoal - 10 > sensorPositionValue ) || (positionGoal + 10 < sensorPositionValue );
    bool is_diff_angle_positiv = (angleCurrent < angleGoal) && (sensorPositionValue <= LIMIT_DIR_ANALOG_MAX) && (sensorPositionValue >= LIMIT_DIR_ANALOG_MIN-20);
    bool is_diff_angle_negativ = (angleCurrent > angleGoal) && (sensorPositionValue <= LIMIT_DIR_ANALOG_MAX+20) && (sensorPositionValue >= LIMIT_DIR_ANALOG_MIN);

    if (is_diff_angle_negativ && enablePinDirection && is_missing_goal){
      analogWrite(LEFT_DIRECTION_PIN,0);
      analogWrite(RIGHT_DIRECTION_PIN,PWM_DIRECTION);
    } else if (is_diff_angle_positiv && enablePinDirection && is_missing_goal){
      analogWrite(RIGHT_DIRECTION_PIN,0);
      analogWrite(LEFT_DIRECTION_PIN,PWM_DIRECTION);
    } else {
      analogWrite(RIGHT_DIRECTION_PIN,0);
      analogWrite(LEFT_DIRECTION_PIN,0);
    }
  }
}

void brakeController(){
  /* Lectura de Switches */
  int readFrontOff  = digitalRead(SWITCH_BRAKE_FRONT_OFF);
  int readFrontOn   = digitalRead(SWITCH_BRAKE_FRONT_ON);
  int readRearOff   = digitalRead(SWITCH_BRAKE_REAR_OFF);
  int readRearOn    = digitalRead(SWITCH_BRAKE_REAR_ON);

  /* Activar frenos con la señal normal, con la de seguridad o al no detectar un modo de control*/
  if (enableBrake || enableSecurity || !isRunMode) {
    digitalWrite(DIR_FRONT_PIN, HIGH); digitalWrite(DIR_REAR_PIN, HIGH);
    initTimeBrakeCmd = initTimeBrakeCmd == 0 ? millis() : initTimeBrakeCmd;

    /* Revisar si ha pasado el tiempo limite de funcionamiento */
    if (millis() - initTimeBrakeCmd > LIMIT_TIME_BRAKE){
      passLimitTimeBrake(true);
    } else {
      /* Freno frontal */
      analogWrite(STEP_FRONT_PIN, readFrontOn == 1 ? 0 : PWM_FRONT_BRAKE); 
      /* Freno trasero */
      analogWrite(STEP_REAR_PIN, readRearOn == 1 ? 0 : PWM_REAR_BRAKE);
    }
  } else {
    digitalWrite(DIR_FRONT_PIN, LOW); digitalWrite(DIR_REAR_PIN, LOW);
    initTimeBrakeCmd = initTimeBrakeCmd == 0 ? millis() : initTimeBrakeCmd;

    /* Revisar si ha pasado el tiempo limite de funcionamiento */
    if (millis() - initTimeBrakeCmd > LIMIT_TIME_BRAKE){
      passLimitTimeBrake(false);
    } else {
      /* Freno frontal */
      analogWrite(STEP_FRONT_PIN, readFrontOff == 1 ? 0 : PWM_FRONT_BRAKE); 
      /* Freno trasero */
      analogWrite(STEP_REAR_PIN, readRearOff == 1 ? 0 : PWM_REAR_BRAKE);
    }
  }
}

void passLimitTimeBrake(bool cmdIsActivate) {
  analogWrite(STEP_FRONT_PIN, 0);
  analogWrite(STEP_REAR_PIN, 0);
  log_msg.level = 1;
  if (cmdIsActivate) {
    int readFrontOn   = digitalRead(SWITCH_BRAKE_FRONT_ON);
    int readRearOn    = digitalRead(SWITCH_BRAKE_REAR_ON);
    if (readFrontOn == 0) {
      log_msg.content = "No se logra detectar el accionar maximo del freno delantero.";
      publishLogLimitBrake(log_msg, lastTimeSwitchFrontBrake);
    }
    if (readRearOn == 0) {
      log_msg.content = "No se logra detectar el accionar maximo del freno trasero.";
      publishLogLimitBrake(log_msg, lastTimeSwitchRearBrake);
    }
  } else {
    int readFrontOff  = digitalRead(SWITCH_BRAKE_FRONT_OFF);
    int readRearOff   = digitalRead(SWITCH_BRAKE_REAR_OFF);
    if (readFrontOff == 0) {
      log_msg.content = "No se logra detectar el accionar minimo del freno delantero.";
      publishLogLimitBrake(log_msg, lastTimeSwitchFrontBrake);
    }
    if (readRearOff == 0) {
      log_msg.content = "No se logra detectar el accionar minimo del freno trasero.";
      publishLogLimitBrake(log_msg, lastTimeSwitchRearBrake);
    }
  }
}

void publishLogLimitBrake(puma_msgs::Log msg, unsigned long lastTime){
  unsigned long time = millis();
  if (time - lastTime > 3000) {
    logInfoPub.publish(&msg);
    lastTime = time;
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
    if ( time - lastTimeSecurity > 3000) {
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
    if ( time - lastTimeModePublish > 3000) {
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
    imu_msg.header.stamp = compass_msg.header.stamp = nh.now();
    if (imu.Read()) {
      imu_msg.linear_acceleration.x = -imu.accel_x_mps2();
      imu_msg.linear_acceleration.y = imu.accel_y_mps2();
      imu_msg.linear_acceleration.z = imu.accel_z_mps2();
      imu_msg.angular_velocity.x = imu.gyro_x_radps();
      imu_msg.angular_velocity.y = -imu.gyro_y_radps();
      imu_msg.angular_velocity.z = -imu.gyro_z_radps();
      imuRawPub.publish(&imu_msg);
      // Serial.print(imu.die_temp_c());
    }
    compass.read();
    /* Convertir Gauss -> Tesla */
    compass_msg.magnetic_field.x = compass.getX() / 10000.0;
    compass_msg.magnetic_field.y = compass.getY() / 10000.0;
    compass_msg.magnetic_field.z = compass.getZ() / 10000.0;
    compassRawPub.publish(&compass_msg);
    lastTimeImu = currentTime;
  }
}

// REVISAR
void publishMsgStatus() {
  unsigned long time = millis();
  if (time - lastTimeStatus >= TIME_PUBLISH_STATUS) {
    // Brake info
    status_msg.activate_brake = enableBrake;
    // Dir info
    status_msg.current_position_dir = sensorPositionValue;
    status_msg.current_angle_rad_dir = (sensorPositionValue-ZERO_DIR_ANALOG)*ANALOG_TO_RAD;
    status_msg.current_angle_deg_dir = (sensorPositionValue-ZERO_DIR_ANALOG)*ANALOG_TO_RAD*RAD_TO_DEG;
    status_msg.enable_dir = enablePinDirection;
    status_msg.is_limit_right_dir= stopDirectionRight;
    status_msg.is_limit_left_dir = stopDirectionLeft;
    // Accel info
    status_msg.enable_accel = enableAccelerator;
    status_msg.pwm_accel = acceleratorValue;
    status_msg.voltage_accel = (acceleratorValue/255.0) * 5.0;
    arduinoStatusPub.publish(&status_msg);
    lastTimeStatus = millis(); // Reset time
  }
}

void acceleratorCallback( const std_msgs::Int16& data_received ) {
  newAcceleratorValue = data_received.data + PWM_MIN_ACCELERATOR;
  if (newAcceleratorValue>= PWM_MIN_ACCELERATOR && newAcceleratorValue < PWM_MAX_ACCELERATOR){
    acceleratorValue = newAcceleratorValue;
    enableAccelerator = true;
  } else {
    enableAccelerator = false;
    nh.logwarn("Valor de acceleracion recibida invalida");
  }
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
    lastTimeSwitchFrontBrake = lastTimeSwitchRearBrake = 0;
  }

  enableBrake = activateBrakeRos;
}

void modeSelectorCallback( const std_msgs::String& data_received) {
  lastTimeModeReceived = millis();
  bool isCorrectMode = false;
  for (int i = 0; i < 4; i++) {
    isCorrectMode = posibleModes[i] == data_received.data;
    if (isCorrectMode) {break;}
  }
  isRunMode = isCorrectMode;
}
