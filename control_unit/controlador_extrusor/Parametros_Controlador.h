//Parámetros de funcionamiento del controlador del extrusor

//Ciclo para actualizar la apertura del relé conectado al calefactor
#define tiempoCiclo 1000  // El tiempo en el que se actualiza la temperatura

//Parámetros de funcionamiento del termistor -> Obtenidos de la hoja de especificaciones
#define termistorNominalRes 100000  //Valor de la resistencia?
#define termistorNominalTemp 25     //Temperatura mínima de la resistencia?
#define termistorBValue 4267    //Valor B de la ecuación de Steinhart 