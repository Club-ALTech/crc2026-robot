#pragma once
#include <Arduino.h>
#include <CrcLib.h>

class Motor
{
public:
    uint8_t _pin;
    float _low_power_limit, _high_power_limit;

    Motor(uint8_t pin, float low = -1, float high = 1)
        : _pin(pin)
    {
        this->set_power_limits(low, high);
    }

    void setup()
    {
        CrcLib::InitializePwmOutput(this->_pin);
    }

    uint8_t get_id()
    {
        return this->_pin;
    }
    
    void set_power_limits(float low, float high)
    {
        this->_low_power_limit = constrain(low, -1, 0);
        this->_high_power_limit = constrain(high, 0, 1);
    }

    void set_power(
        float power // between -1 and 1 (counterclockwise/clockwise)
    )
    {
        power = constrain(power, this->_low_power_limit, this->_high_power_limit);
        CrcLib::SetPwmOutput(this->_pin, power * 127);
    }
};
