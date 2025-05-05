//Configuración de los pines del controlador del extrusor
//Placa: Fire Beetle 2


//Pines para la configuración del termistor

#define termistorPin A0  //Pin A0 del WROOM
#define relePin D3       // D3
#define relePIDPin D5    // D5


//Pines para los pines del driver TB6600(Active-Low)

#define STEP_PIN D6 //Driver -> 
#define DIR_PIN D7  //Driver -> Dir +
#define ENA_PIN D9 // Optional No enchufado
