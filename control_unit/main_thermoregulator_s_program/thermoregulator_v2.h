#include <SPI.h>
#include "Adafruit_MAX31855.h"
#include "Ticker.h"

#define MAX_DATA 10
#define LAST_DATA MAX_DATA-1
#define INITIAL 30

class thermoregulator {
  public:
     thermoregulator(uint8_t pin_SLK, uint8_t pin_CS, uint8_t pin_DO, uint8_t pin_RELAY, int FREQ);
     void initializeSystem(); //Set the 10 initial temperatures
     void controlTemperature(boolean activated);
     int temp_cons;

     
      
  private:

     Adafruit_MAX31855 thermocouple;
     uint8_t pin_RELAY;
     int FREQ;
     boolean activated;
     float temp[MAX_DATA];
     float average;
     int LIM_SUP;
     int LIM_INF;
     int message;
     void updateAndAverageTemperatures();
     void createMessages();
     static void _onTick(); 
     static Ticker _ticker; 
     static bool periodicValue; 
     static bool activatedBefore; 
     void checkStateChange();
     void sendDataToCPU();

};
