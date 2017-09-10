#include "Arduino.h"
#include "fasthal.h"
#include "pumpcontroller.h"

using namespace fasthal;

FASTHAL_INITADC(AdcRef::Default);
FASTHAL_ARDUINO_TIME();


//typedef arduino::PinD13 LedPin;

typedef arduino::PinD2 CriticalLevelPin;
typedef arduino::PinD3 NormalLevelPin;
typedef InvertedPin<arduino::PinD4> PumpPin;

typedef arduino::PinD13 StatusPin;
typedef AdcRms<AdcZero<arduino::AdcA0, constInt<509>>, fasthal::Time::freqToMicros(50)> PumpSensorAdc;
typedef Acs712_30A<PumpSensorAdc, 5, 1024> PumpSensor;

PumpController<PumpPin, StatusPin, CriticalLevelPin, NormalLevelPin, PumpSensor> pumpController;

//bool pumpOn = false;

void setup(){
    pumpController.begin();

    #ifdef DEBUG
    Serial.begin(9600);
    #endif
}

void loop(){
    pumpController.update();    

    // auto current = PumpSensor::readAC();
    // Serial.print("current: ");
    // Serial.print(current);
    // auto power = current * 230;
    // Serial.print(" power: ");
    // Serial.println(power);
    // delay(1000);
    // LedPin::toggle();
}