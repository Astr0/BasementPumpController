#pragma once

#ifndef PUMPCONTROLLER_H_
#define PUMPCONTROLLER_H_

#include "fasthal.h"

enum PumpControllerState{
    Critical = 0,
    Normal = 1,
    Filling = 2,
    WaitingForWater = 3
};

template<
    class PumpPin,
    class StatusPin,
    class CriticalLevelPin,
    class NormalLevelPin,
    class PumpAcs712>
class PumpController{
private:
    static const uint16_t LevelSensorDebounceTime = 200;
    static const uint16_t PumpMonitoringDelay = 2 * 1000; // 2 seconds
    
    #ifdef DEBUG
    static const uint16_t WaitingForWaterTime = 10 * 1000; // 10 seconds (debug)
    static constexpr float PumpMaxCurrent = 2;
    static constexpr float PumpMinCurrent = 1;
    #else
    static const uint16_t WaitingForWaterTime = 10U * 60U * 1000U; // 10 minutes
    static constexpr float PumpMaxCurrent = 7;
    static constexpr float PumpMinCurrent = 4;
    #endif

    // n/c state of pin == HIGH, this should correspond to critical level reached
    // down = critical level, up = ok
    fasthal::Bounce<CriticalLevelPin, LevelSensorDebounceTime> _criticalLevel;
    fasthal::Bounce<NormalLevelPin, LevelSensorDebounceTime> _normalLevel;
    fasthal::PinBlink<StatusPin> _statusBlink;

    PumpControllerState _currentState = PumpControllerState::Critical;
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
        PumpPin::set(_currentState == PumpControllerState::Filling);           


        switch (_currentState){
            case PumpControllerState::Filling:
                _statusBlink.change(200, 200);           
                break;
            case PumpControllerState::Critical:
                _statusBlink.change(3000, 1000);           
                break;
            case PumpControllerState::Normal:
                _statusBlink.change(200, 3000);           
                break;
            case PumpControllerState::WaitingForWater:
                _statusBlink.change(1000, 1000);           
                break;
        }
    }
    
public:
    PumpController():
        _criticalLevel(true),
        _normalLevel(true),
        _statusBlink(3000, 1000)
    {
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
        _criticalLevel.update();
        bool criticalLevelReached = _criticalLevel.read();
        _normalLevel.update();
        bool normalLevelReached = _normalLevel.read();

        switch (_currentState){
            case PumpControllerState::Critical:
                if (!criticalLevelReached){
                    // critical level gone, phew
                    setState(PumpControllerState::Normal);
                }
                break;
            case PumpControllerState::Normal:
                if (criticalLevelReached){
                    // normal is still ok, but we reached critical... sensor failure
                    setState(PumpControllerState::Critical);
                } else if (!normalLevelReached){
                    // normal level gone, start filling
                    setState(PumpControllerState::Filling);
                }
                break;
            case PumpControllerState::Filling:
                if (normalLevelReached)
                {
                    // reached top
                    setState(PumpControllerState::Normal);
                } else if (criticalLevelReached){
                    // sensor failure, haven't reached normal but reached critical
                    setState(PumpControllerState::Critical);
                } else if (_countdown.elapsed(PumpMonitoringDelay)){
                    // monitor pump current
                    float current = PumpAcs712::read();
                    
                    #ifdef DEBUG
                    if (Serial)
                        Serial.println(current);
                    #endif
                    
                    if ((current >= PumpMaxCurrent) || (current <= PumpMinCurrent))
                        setState(PumpControllerState::WaitingForWater);
                }
                break;
            case PumpControllerState::WaitingForWater:
                if (_countdown.elapsed(WaitingForWaterTime)){
                    // we've waited long enough - go to Normal state to check what to do
                    setState(PumpControllerState::Normal);
                }
                break;
            default:
                // shouldn't be here
                setState(PumpControllerState::Critical);
                break;
        }
    }

};
    


#endif