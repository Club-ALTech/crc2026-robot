#pragma once
#include <Arduino.h>

class RevAbsEncoder {
    public:
    
    uint8_t _pin;
    unsigned long _timeout;

    bool _angle_is_initialized = false;
    float angle;

    RevAbsEncoder(uint8_t pin, unsigned long timeout): _pin(pin), _timeout(timeout) {
    }

    unsigned long _read_raw() {
        return pulseIn(this->_pin, HIGH, this->_timeout); // TODO: add timeout to constructor
    }

    float read_angle() {
        auto PULSE_MIN = 3.9, PULSE_MAX = 988.0;
        auto ANGLE_MIN = 0, ANGLE_MAX = 360;        
        auto pulse = this->_read_raw();
        if (pulse < PULSE_MIN || pulse > PULSE_MAX) {
            return this->angle;
        }
        this->_angle_is_initialized = true;
        this->angle = ((pulse - PULSE_MIN) * ANGLE_MAX) / (PULSE_MAX - PULSE_MIN);
        return this->angle;
    }

    bool is_angle_initialized() {
        return this->_angle_is_initialized;
    }
};


class Encoder
{
public:
    uint8_t _pin;
    unsigned long _timeout;

    Encoder(uint8_t pin, unsigned long timeout)
        : _pin(pin), _timeout(timeout)
    {
    }

    unsigned long getPulseHigh()
    {
        return pulseIn(_pin, HIGH, _timeout);
    }

    unsigned long getPulseLow()
    {
        return pulseIn(_pin, LOW, _timeout);
    }

    String getCurrentValue()
    {
        auto value = getPulseHigh();

        return String(value);
    }
};