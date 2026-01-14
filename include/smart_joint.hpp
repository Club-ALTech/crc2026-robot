#pragma once
#include <Arduino.h>
#include "motors.hpp"
#include "encoders.hpp"

class SmartJoint {

public:
    Motor &m;
    RevAbsEncoder &e;
    
    float min_angle = 0;
    float max_angle = 360;
    float target_angle = 0;
    bool target_is_set = false;

    float Kp = 0;
    float Ki = 0;
    float Kd = 0; 

    SmartJoint(Motor &motor, RevAbsEncoder &encoder)
        : m(motor), e(encoder) {
    }

    // called once per loop, if needed
    void update() {
        if (target_is_set && e.is_angle_initialized()) {
            float current_angle = e.read_angle();
            float error = target_angle - current_angle;

            float kP = 0.01;
            float kI = 0.001;
            float kD = 0.001;

            static float integral = 0;
            static float last_error = 0;

            integral += error;
            float derivative = error - last_error;
            last_error = error;

            float power = kP * error + kI * integral + kD * derivative;
            m.set_power(power);
        }
    }

    void set_angular_limits(float limMin, float limMax) {
        min_angle = limMin;
        max_angle = limMax;
    }

    void set_target_angle(float target) {
        // set the angle at which the joint must move to
        target_angle = constrain(target, min_angle, max_angle);
        target_is_set = true;
    }

    void set_pid(float kp, float ki, float kd) {
        Kp = kp;
        Ki = ki;
        Kd = kd;
    }
};