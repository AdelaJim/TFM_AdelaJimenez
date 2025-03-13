//#include <SPI.h>

#include "pinout.h"
#include "thermoregulator_v2.h"

#define TEMP_CONS_INF 50
#define TEMP_CONS_SUP 70
#define FREQ 500


const byte BUFFER_SIZE = 10; 
char buffer[BUFFER_SIZE];    
byte index_PC = 0;
volatile boolean activated = false; 

thermoregulator thermoregulator(MAX6675_SCK, MAX6675_CS, MAX6675_SO,RELAY, FREQ);

void setup() {
  
  pinMode(RELAY, OUTPUT);
  digitalWrite(RELAY, LOW); 
  Serial.begin(2400);
  delay(1000); 
  
  while (!Serial) {
    ; 
  }
  
  //Serial.println("Iniciando comunicación serial");
  thermoregulator.initializeSystem();
  
}

void loop() {
  //Serial.println();
  thermoregulator.controlTemperature(activated);
}


void serialEvent() {
    while (Serial.available()) {
    char incomingByte = Serial.read(); 
    
    //Serial.println("Entra por serial: "); 
    //Serial.println(incomingByte);

    if (incomingByte != '\r' && incomingByte != '\n') {
      buffer[index_PC] = incomingByte; 
      index_PC++;                     
 
      if (index_PC >= BUFFER_SIZE) {
 
        index_PC = BUFFER_SIZE - 1; 
        break;
      }
      
    } else {
      
      buffer[index_PC] = '\0';
      Serial.println("Mensaje completo recibido: ");
      Serial.println(buffer);
      processString(buffer, activated, thermoregulator.temp_cons);

      
      index_PC = 0;
    }
  }
}

void processString(char* str, volatile boolean &activated, int &temp_cons) {
  
  if (strcmp(str, "OFF") == 0 && activated == true) {
    activated = false;
    Serial.println("Termostato apagado");
  } 
  
  else{

    if(activated == false) {
       thermoregulator.temp_cons = atoi(str); 

       if (thermoregulator.temp_cons >= TEMP_CONS_INF && thermoregulator.temp_cons <= TEMP_CONS_SUP) {
            //Serial.println("El número está en el rango de 50 a 90");
            Serial.println("Termostato encendido");
            activated = true;
         } 
         
       else {
            //Serial.println("ERROR: Temperatura no válida");
            activated = false;
         }
       }
  }
}
