#include <Arduino.h>
#include <SPI.h>
#include "Adafruit_MAX31855.h"
#include "thermoregulator_v2.h"
#include "Ticker.h"

Ticker thermoregulator::_ticker; 
bool thermoregulator::periodicValue = false; 
bool thermoregulator::activatedBefore = false; 

thermoregulator::thermoregulator(uint8_t pin_SLK, uint8_t pin_CS, uint8_t pin_DO, uint8_t pin_RELAY, int FREQ)
    : thermocouple(pin_SLK, pin_CS, pin_DO) {
    this->pin_RELAY = pin_RELAY;
    this->FREQ = FREQ;
}

void thermoregulator::initializeSystem(){
  int i = 0;
  delay(500); 
  //Serial.println("Tomando medidas iniciales del entorno");
  //Serial.println();

  for(i; i <= LAST_DATA; i++){
    temp[i] = thermocouple.readCelsius();
    delay(FREQ);
  }
  
  //Serial.println("Toma de medidas realizada");
  //Serial.println("Inicio de programa");
  //Serial.println();
  delay(FREQ);
}


void thermoregulator::controlTemperature(boolean activated){ 
   this->activated = activated;
   updateAndAverageTemperatures();
   //createIntervals();
   checkStateChange();
   if(activated == true) {
       if((temp[LAST_DATA] <= temp_cons) && periodicValue) {
           digitalWrite(pin_RELAY, HIGH);
       } 
       else {
           digitalWrite(pin_RELAY, LOW);  
       }
   }

  else{
      digitalWrite(pin_RELAY, LOW); }

  activatedBefore = activated;
  sendDataToCPU();
  
  }


void thermoregulator::updateAndAverageTemperatures() { 

   int i = 0; 
   int plus = 0;

   for(i; i < LAST_DATA; i++) {
     temp[i] = temp[i+1];
     plus = temp[i] + plus;
   }

   temp[LAST_DATA] = thermocouple.readCelsius();
   plus = temp[LAST_DATA] + plus;
 
   average = plus/10.0;
   
}

void thermoregulator::createMessages() { 
  
   if(activated) {
       if(average < LIM_INF) message = 0; //Heating
       if(average > LIM_INF && average < LIM_SUP) message = 1; //Heated
       if(temp[LAST_DATA] < (LIM_SUP + 10) && average > LIM_SUP) message = 2; //Overheated
   }

   else{
       if(average > INITIAL && average < (LIM_SUP + 10)) message = 3; //Cooling
       if(average < INITIAL) message = 4; //Cooled
    };
}


void thermoregulator::sendDataToCPU() {
  
  createMessages();
  //Serial.println();
  //Serial.print("Estado -> "); Serial.println(activated);
  //Serial.print("Temperatura -> "); 
  Serial.println(temp[LAST_DATA]);
  //Serial.print("Temperatura consigna -> "); Serial.println(temp_cons);
  //Serial.print("Límite superior -> "); Serial.println(LIM_SUP);
  //Serial.print("Límite inferior -> "); Serial.println(LIM_INF);
  //Serial.print("Temperatura media -> "); Serial.println(average);
  //Serial.print("Mensaje -> "); Serial.println(message);
  // Serial.print("Temporizador -> "); Serial.println(periodicValue);
  //Serial.println();

  // Enviar TEMP_OK si la temperatura promedio está en el rango aceptable
  if (average >= LIM_INF && average <= LIM_SUP) {
      Serial.println("TEMP_OK");
  }
  delay(FREQ);
}

void thermoregulator::_onTick() {
  periodicValue = !periodicValue; 
}

void thermoregulator::checkStateChange() {
  if(activated != activatedBefore && activated) { 
    _ticker.attach(10, _onTick); 
    periodicValue = true; 
    LIM_SUP = temp_cons + 3;
    LIM_INF = temp_cons - 3;
    }
  if(activated != activatedBefore && !activated) {
    _ticker.detach(); 
    periodicValue = false; 
    temp_cons = 0; 
    LIM_SUP = 0; 
    LIM_INF = 0;};
}
