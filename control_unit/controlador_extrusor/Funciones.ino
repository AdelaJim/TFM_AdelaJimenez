int Alta_Baja_T(int indice, int steinhart){

  int Indice_T = indice;

  float limite = lim_HL_T[indice];

  if(steinhart >= limite && indice == 0){
    digitalWrite(relePin, HIGH);
    Indice_T = 1;
  };

  if(steinhart < limite && indice == 1){
    digitalWrite(relePin, LOW);
    Indice_T = 0;
  };

  return Indice_T;
  
}

void leerT(int Indice_T){

  termistorRes = ((float)analogRead (termistorPin)* VoltageDividerResistor[Indice_T])/(4095 - (float)analogRead (termistorPin));
  //Serial.println();
  //Serial.print((float)analogRead (termistorPin));
  //Serial.print(", ");
  //Serial.println(termistorRes);
  
  steinhart = termistorRes / termistorNominalRes;     // (R/Ro)
  steinhart = log(steinhart);                         // ln(R/Ro)
  steinhart /= termistorBValue;                       // 1/B * ln(R/Ro)
  steinhart += 1.0 / (termistorNominalTemp + 273.15); // + (1/To)
  steinhart = 1.0 / steinhart;                        // Invert  
  steinhart -= 273.15;                                // convert to C
}

void control() {

    if ((millis() <= (lastPIDCalculation + Output)) || (Output == tiempoCiclo)) {
    // Power on:
    digitalWrite(relePIDPin, HIGH);
  } else {
    // Power off:
    digitalWrite(relePIDPin, LOW);
  }

}
