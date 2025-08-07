  //Archivos para pines
#include "Pines_Fire_Beetle_2.h"
#include "Conf_pines.h"
#include "Parametros_Controlador.h"

//Librerías
#include <PID_v1.h>  // https://github.com/br3ttb/Arduino-PID-Library/tree/master
#include <AccelStepper.h>

AccelStepper stepper(AccelStepper::DRIVER, STEP_PIN, DIR_PIN, false);

//Serial 
const byte BUFFER_SIZE = 10; 
char buffer[BUFFER_SIZE];    
byte index_PC = 0;
boolean bandera = false; //Para mandar un mensaje al Master Mind

//Activar extrusor
int activar_calefactor = 0;
int activar_extrusor = 0; //Activar el giro del extrusor

//Velocida Stepper. Pulsos por segundos. Configurado para 1600 pulsos en el driver
float velocidad = 0; 

float VoltageDividerResistor[2] = {10000, 910}; //Resistencias para bajas (0) y altas temperaturas (1)
int Indice_T = 0;
float lim_HL_T[2] = {100, 85}; //En ºC, siendo (1) la temperatura límite superior para considerar que trabaja a baja T y (2) el límite inferior para alta

//Control PID
double Input, Output;   //Definición de variables iniciales
double Setpoint = 0; //Temperatura consigna
boolean giro = false; //Giro del motor.
                                  //Set Point -> Temperatura consigna del termistor
                                  //Input -> Valor de entrada de la temperatura
                                  //Output -> Valor del tiempo
double Kp=40, Ki=3, Kd=500;       //Parámetros iniciales del PID   

                              // Valor original de PID -> Kp=40, Ki=3, Kd=500
unsigned long respuestaUltimaTemperatura = 0;   //Variable que almacena la última temperatura
unsigned long lastPIDCalculation = 0;           //Variable que almacena el valor de PID calculado
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);


//Ajustes iniciales del termistor
float prevTemperature = -9999.0; //Tº inicial 
float termistorRes = 0.0;        //Resistencia del termistor
float steinhart = 0;             //Temperatura (Ecuación Steinhart


void setup(){
  Serial.begin(115200);

  //Ajustes de los pines
  //Termistor
  pinMode(termistorPin, INPUT);
  pinMode(relePin, OUTPUT);
  pinMode(relePIDPin, OUTPUT);

  //Driver
  pinMode(ENA_PIN, OUTPUT);

  //Configuración de los pines OUTPUT
  digitalWrite(relePin, LOW);
  digitalWrite(ENA_PIN, LOW);  // Habilita el TB6600 (active-low)

  //Ajustes del PID 
  myPID.SetOutputLimits(0, tiempoCiclo);                           
  myPID.SetSampleTime(tiempoCiclo);
  myPID.SetMode(AUTOMATIC);


 //Ajustes del Driver
 
  stepper.setMaxSpeed(3200);  // Velocidad máxima en pasos/segundo
  stepper.setAcceleration(10000);  // Aceleración en pasos/s^2
  stepper.setSpeed(velocidad);  // Establece la velocidad inicial
 
}

void loop(){

   if (millis() - respuestaUltimaTemperatura >= tiempoCiclo) {
    leerT(Indice_T);
    Input = steinhart;
    Indice_T = Alta_Baja_T(Indice_T, Input);
    myPID.Compute();
    
    if(Input <= 130){
      Output = 1000; 
    }

    if(Input >= (Setpoint+2)){
      Output = 0; 
    }
       
    lastPIDCalculation = millis();
    Serial.println(steinhart); //Imprimir valor de temperatura
    
    //Serial.println(Output);
    respuestaUltimaTemperatura = millis();

    
  }

  if(activar_calefactor){ //Activar el calefactor
      control();
  }

  if(Input >= (Setpoint - 5) && activar_calefactor){

      if(!bandera){
        Serial.println("TEMP_OK");
        bandera = true;
      }

      if(activar_extrusor){
           stepper.setSpeed(velocidad);  // Establece la velocidad inicial
      }
    
      else{
           stepper.setSpeed(0);  // Establece la velocidad inicial
      }
      
      stepper.runSpeed();
    
      }
 
}

//Info por serial

void serialEvent() {
    while (Serial.available()) {
    char incomingByte = Serial.read(); 
    

    if (incomingByte != '\r' && incomingByte != '\n') {
      buffer[index_PC] = incomingByte; 
      index_PC++;                     
 
      if (index_PC >= BUFFER_SIZE) {
 
        index_PC = BUFFER_SIZE - 1; 
        break;
      }
      
    } else {
      
      buffer[index_PC] = '\0';

      processString(buffer, activar_calefactor, Setpoint, velocidad, activar_extrusor, bandera);

      
      index_PC = 0;
    }
  }
}

void processString(char* str, int &activar_calefactor, double &Setpoint, float &velocidad, int &activar_extrusor, boolean &bandera) {

  char* coma = strchr(str, ','); //Índice para encontrar la coma
  
  if (strcmp(str, "OFF") == 0 && activar_calefactor == true) {
      activar_calefactor = false;
      activar_extrusor = false;
      digitalWrite(relePIDPin, LOW); //Apagar el relé del calefactor
      bandera = false;
      Serial.println("OFF"); Serial.println(bandera);
  } 
  
  else{
      if(activar_calefactor  == true && activar_extrusor == false && !strcmp(str, "EXT_OK")){
         activar_extrusor = true; 
         Serial.println("ON2");      
      }
  
      else{
  
            if(activar_calefactor  == false && coma != NULL) {
              *coma = '\0'; // Creamos un carácter nulo en la coma para separar la cadena
               Setpoint = atoi(str);        // Primera parte (antes de la coma)
               velocidad = atoi(coma + 1);   // Segunda parte (después de la coma)
               activar_calefactor  = true;
               Serial.println(velocidad);
               Serial.println(Setpoint);
              
            }
  
            else{
               Serial.println("WRONG");
            }
      }
    }
}