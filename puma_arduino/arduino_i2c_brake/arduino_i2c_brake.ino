#include <Wire.h> // Wire para I2C
#include <AccelStepper.h> // Control de motores pp

// Variable para la comunicacion de datos i2c
String inString = "";
String outString = "";
byte bytes_out[100];
int k_dataWrite = -1;

// Variables de configuracion
byte steps_res[3]; //Convertir a char para usar
int accel_motor, vel_motor, pose_motor;
int current_pose_motor = 0;

// Pines motor pp - MPP1 motor 1 - MPP2 motor 2
const int MS_MPP1[3] = {0,1,2};
const int MS_MPP2[3] = {3,4,5};
const int DIR_MPP1 = 8;
const int STEP_MPP1 = 9;
const int DIR_MPP2 = 10;
const int STEP_MPP2 = 11;

// Variables control de motores
AccelStepper freno_trasero(1, STEP_MPP1, DIR_MPP1);
AccelStepper freno_delantero(1, STEP_MPP2, DIR_MPP2); 

int i;
bool receiveAny = false;
bool configFailed = false;

void setup() {
  // Iniciar comunicacion serial - DEBUG
  Serial.begin(9600);
  
  // Conectar i2c
  Wire.begin(0x8);

  // Config pinMode
  for( i=0; i<3; i++){
    pinMode(MS_MPP1[i], OUTPUT);
    pinMode(MS_MPP2[i], OUTPUT);
    digitalWrite(MS_MPP1[i],HIGH);
    digitalWrite(MS_MPP2[i],HIGH);
  }
  // CallBack cuando se recibe datos
  Wire.onReceive(receiveData);
  Wire.onRequest(sendData);
  
  // Mensaje predeterminado
  outString = "Listo para configurar";
  outString.getBytes(bytes_out,  sizeof(bytes_out)+1);
}

void loop(){
  if (receiveAny) {
    analizeData(); //Analizar los datos obtenidos
  }
  if (accel_motor != NULL and vel_motor != NULL and pose_motor != NULL){
    freno_trasero.run();
    freno_delantero.run();
    //Serial.println("Moviendo motores...");
  }
  //Serial.println(steps_res[0]);
  //delay(400);
}

// Funcion ejecutada al recibir datos
void receiveData(int howMany) {
  inString = "";
  receiveAny = true;

  while(Wire.available()){
    int number = Wire.read();
    inString += (char)number; 
  } 
}

// Funcion para envio de datos 
void sendData() {
  if(k_dataWrite==-1){  // Primer mensaje envia el tamano
    Wire.write(outString.length());
  } else {
    Wire.write(bytes_out[k_dataWrite]);
  }

  k_dataWrite++;
  if (k_dataWrite==outString.length()){
    k_dataWrite = -1;
    outString = "";
    outString.getBytes(bytes_out, sizeof(bytes_out)+1);
  }
  // Pequeña espera para no colapsar la comunicacion
  delay(50);
}

void analizeData() {
    //Example config data: "cfg steps:110,pose:1000,acel:250,vel:500,
    String aux = "";
    String cmd = inString.substring(1,4);
    
    if(cmd=="cfg"){
      //Serial.println("Entrando en configuracion");
      aux = inString.substring(4+1);
      // BUCLE PARA ANALIZAR LAS OPCIONES
      while (aux.length() > 1) {
        /* // DEBUG
        Serial.print("Tamaño del texto"); 
        Serial.println(aux.length()); */
        String option = aux.substring(0, aux.indexOf(":"));
        String value = aux.substring(aux.indexOf(":")+1, aux.indexOf(","));
        //Serial.println(option);

        if (option=="steps"){
          // CONFIGURACION PARA LA RESOLUCION 
          //Serial.println("Entrando en la opcion steps");
          value.getBytes(steps_res,4);
          for( i=0; i<3; i++){
            if(char(steps_res[i]) == '1') {
                digitalWrite(MS_MPP1[i],HIGH);
                digitalWrite(MS_MPP2[i],HIGH);
            } else {
                digitalWrite(MS_MPP1[i],LOW);
                digitalWrite(MS_MPP2[i],LOW);
            }
          }
        } else if (option=="acel") {
          // CONFIGURACION PARA LA ACELERACION
          //Serial.println("Entrando en la opcion acel");
          accel_motor = value.toInt();
          freno_trasero.setAcceleration(accel_motor);
          freno_delantero.setAcceleration(accel_motor);
        } else if (option=="vel") {
          // CONFIGURACION PARA LA VELOCIDAD
          //Serial.println("Entrando en la opcion vel");
          vel_motor = value.toInt();
          freno_trasero.setMaxSpeed(vel_motor);
          freno_delantero.setMaxSpeed(vel_motor);
        } else if (option=="pose"){
          // CONFIGURACION PARA LA POSICION
          //Serial.println("Entrando en la opcion pose");
          pose_motor = value.toInt();
          if (pose_motor != current_pose_motor) {
            freno_trasero.moveTo(pose_motor);
            freno_delantero.moveTo(pose_motor);
            current_pose_motor = pose_motor;
          }
          
        } else {
          outString = "Opcion invalida";
          outString.getBytes(bytes_out, sizeof(bytes_out)+1);
          k_dataWrite = -1;
          configFailed = true;
          break;
        }
        aux = aux.substring(aux.indexOf(",")+1); // SIEMPRE TERMINAR COMANDOS EN ,
        delay(100);
      }
      if(configFailed){
        outString = "Configuracion no realizada por comando invalido";
      } else {
        outString = "Configuracion realizada";
      }
      outString.getBytes(bytes_out, sizeof(bytes_out)+1);
      k_dataWrite = -1;
      
    } else {
      outString = "Comando invalido";
      outString.getBytes(bytes_out, sizeof(bytes_out)+1);
      k_dataWrite = -1;
    }
    
    // Limpiar el input
    inString = "";
    receiveAny = false;
}

