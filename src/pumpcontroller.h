#pragma once

#ifndef PUMPCONTROLLER_H_
#define PUMPCONTROLLER_H_

#include "fasthal.h"

enum PumpControllerState{
    PumpControllerStateCritical = 0,
    PumpControllerStateNormal = 1,
    PumpControllerStateFilling = 2,
    PumpControllerStateWaitingForWater = 3
};

template<
    class PumpPin,
    class StatusPin,
    class CriticalLevelPin,
    class NormalLevelPin,
    class PumpAcs712>
class PumpController{
private:
    static const uint32_t LevelSensorDebounceTime = 200UL;
    static const uint32_t PumpMonitoringDelay = 2UL * 1000UL; // 2 seconds
    
    #ifdef DEBUG
    static const uint32_t WaitingForWaterTime = 10UL * 1000UL; // 10 seconds (debug)
    static constexpr float PumpMaxCurrent = 2;
    static constexpr float PumpMinCurrent = 1;
    #else
    static const uint32_t WaitingForWaterTime = 10UL * 60UL * 1000UL; // 10 minutes
    static constexpr float PumpMaxCurrent = 7;
    static constexpr float PumpMinCurrent = 4;
    #endif

    // n/c state of pin == HIGH, this should correspond to critical level reached
    // down = critical level, up = ok
    fasthal::Button<CriticalLevelPin> _criticalLevel;
    fasthal::Button<NormalLevelPin> _normalLevel;
    fasthal::PinBlink<StatusPin> _statusBlink;

    PumpControllerState _currentState = PumpControllerStateCritical;
    bool _criticalLevelReached = true;
    bool _normalLevelReached = true;    
    fasthal::ElapsedMs _countdown;    


    void setState(PumpControllerState state){
        if (state == _currentState)
            return;

        #ifdef DEBUG
        if (Serial){
            Serial.print("New state:");
            Serial.println(state); 
        }
        #endif

        _currentState = state;
        _countdown.reset();
        PumpPin::set(_currentState == PumpControllerStateFilling);           


        switch (_currentState){
            case PumpControllerStateFilling:
                _statusBlink.change(200, 200);           
                break;
            case PumpControllerStateCritical:
                _statusBlink.change(3000, 1000);           
                break;
            case PumpControllerStateNormal:
                _statusBlink.change(200, 3000);           
                break;
            case PumpControllerStateWaitingForWater:
                _statusBlink.change(1000, 1000);           
                break;
        }
    }
    
    void readLevels(){
        if (_criticalLevel.debounce(LevelSensorDebounceTime))
            _criticalLevelReached = _criticalLevel.pressed();

        if (_normalLevel.debounce(LevelSensorDebounceTime))
            _normalLevelReached = _normalLevel.pressed();
    }

public:
    PumpController(): _statusBlink(3000, 1000){

    }

    void begin(){
        PumpPin::clear();
        PumpPin::setMode(fasthal::PinMode::Output);

        _statusBlink.begin(true);
        //LedPin::setMode(PinMode::Output);    
    
        _criticalLevel.begin(fasthal::PinMode::InputPullup);
        _normalLevel.begin(fasthal::PinMode::InputPullup);

        _countdown.reset();
    }

    void update(){
        _statusBlink.update();
        readLevels();

        switch (_currentState){
            case PumpControllerStateCritical:
                if (!_criticalLevelReached){
                    // critical level gone, phew
                    setState(PumpControllerStateNormal);
                }
                break;
            case PumpControllerStateNormal:
                if (_criticalLevelReached){
                    // normal is still ok, but we reached critical... sensor failure
                    setState(PumpControllerStateCritical);
                } else if (!_normalLevelReached){
                    // normal level gone, start filling
                    setState(PumpControllerStateFilling);
                }
                break;
            case PumpControllerStateFilling:
                if (_normalLevelReached)
                {
                    // reached top
                    setState(PumpControllerStateNormal);
                } else if (_criticalLevelReached){
                    // sensor failure, haven't reached normal but reached critical
                    setState(PumpControllerStateCritical);
                } else if (_countdown.elapsed(PumpMonitoringDelay)){
                    // monitor pump current
                    float current = PumpAcs712::read();
                    
                    #ifdef DEBUG
                    if (Serial)
                        Serial.println(current);
                    #endif
                    
                    if ((current >= PumpMaxCurrent) || (current <= PumpMinCurrent))
                        setState(PumpControllerStateWaitingForWater);
                }
                break;
            case PumpControllerStateWaitingForWater:
                if (_countdown.elapsed(WaitingForWaterTime)){
                    // we've waited long enough - go to Normal state to check what to do
                    setState(PumpControllerStateNormal);
                }
                break;
            default:
                // shouldn't be here
                setState(PumpControllerStateCritical);
                break;
        }
    }

};
    


#endif