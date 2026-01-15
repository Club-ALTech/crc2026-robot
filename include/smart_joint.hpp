#pragma once
#include <Arduino.h>
#include <PID_v1.h>
#include "motors.hpp"
#include "encoders.hpp"

class SmartJoint
{

public:
    Motor &m;
    RevAbsEncoder &e;
    PID pid;

    float min_angle = 0;
    float max_angle = 360;
    float target_angle = 0;
    bool target_is_set = false;

    double _input, _setpoint, _output;

    SmartJoint(Motor &motor, RevAbsEncoder &encoder)
        : m(motor),
          e(encoder),
          pid(&this->_input, &this->_output, &this->_setpoint, 0.0, 0.0, 0.0, DIRECT)
    {
        this->_setpoint = 0;
    }

    void begin(float kp, float ki, float kd)
    {
        // initialize PID
        pid.SetTunings(kp, ki, kd);
        pid.SetMode(AUTOMATIC);
        pid.SetOutputLimits(-1, 1); // motor power limits
    }

    // called once per loop, if needed
    void update()
    {
        if (target_is_set && e.is_angle_initialized())
        {
            float current_angle = e.read_angle();
            float error = target_angle - current_angle; // not really used

            this->_input = current_angle;
            this->_setpoint = target_angle;

            this->pid.Compute();
            m.set_power(this->_output);
        }
    }

    // void set_angular_limits(float limMin, float limMax)
    // {
    //     min_angle = limMin;
    //     max_angle = limMax;
    // }

    void set_target_angle(float target)
    {
        // set the angle at which the joint must move to
        target_angle = constrain(target, min_angle, max_angle);
        target_is_set = true;
    }
};