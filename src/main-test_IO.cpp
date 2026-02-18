#include <Arduino.h>
#include <CrcLib.h>
#include <Wire.h>
#include <math.h>
#include "navx.hpp"
#include <Servo.h>
#include <Smoothed.h>

const int WHEEL_FL_M_p = CRC_PWM_11;
const int WHEEL_FR_M_p = CRC_PWM_2;
const int WHEEL_BL_M_p = CRC_PWM_12;
const int WHEEL_BR_M_p = CRC_PWM_4;

const int LIFT_L_M_p = CRC_PWM_10;
const int LIFT_R_M_p = CRC_PWM_3;
const int LIFT_E_p = CRC_DIG_3; // lift height

const int MANIP_PITCH_E_p = CRC_DIG_4;
const int MANIP_PITCH_M_p = CRC_PWM_8;

const int MANIP_ROLL_E_p = CRC_DIG_2;
const int MANIP_ROLL_M_p = CRC_PWM_1; // NOTE: limit to 20% is a good default speed

const int MANIP_BELT_A_p = CRC_PWM_9;
const int MANIP_BELT_B_p = CRC_PWM_5;

const int BEAM_p = CRC_DIG_1;

const int PRINT_TIMER_DELAY = 1000 / 20; // 20Hz
// Servo manip_belt_a, manip_belt_b;

CrcLib::Timer print_timer, battery_low_timeout;

class ReadPWM
{
    uint32_t _last, _timeout;
    uint8_t _pin, _mode;

public:
    ReadPWM(uint8_t pin, uint8_t mode = HIGH, uint32_t timeout = 1050)
        : _last(0), _timeout(timeout), _pin(pin), _mode(mode)
    {
        pinMode(LIFT_E_p, INPUT);
    }

    bool read(uint32_t &into)
    {
        into = pulseIn(this->_pin, this->_mode, this->_timeout);
        if (into == 0)
        {
            into = this->_last;
            return false;
        }
        else
        {
            this->_last = into;
            return true;
        }
    }
};

class PwmToAngleConverter
{
public:
    float _offset_deg;
    int8_t _reverse;
    uint32_t _min_pulse, _max_pulse;

    PwmToAngleConverter(bool reverse = false, float offset_deg = 0, uint32_t min_pulse = 1, uint32_t max_pulse = 1024)
        : _offset_deg(offset_deg), _reverse(reverse ? 1 : -1), _min_pulse(min_pulse), _max_pulse(max_pulse) {}

    float convert(uint32_t pwm)
    {
        auto deg = _reverse * (pwm - this->_min_pulse) * 360.0 / (this->_max_pulse - this->_min_pulse);
        return deg + _offset_deg;
    }

    void set_offset(float offset_deg)
    {
        this->_offset_deg = offset_deg;
    }
};

float travel_deg(float from, float to)
{
    auto zeroed = to - from;
    if (zeroed > 180)
    {
        return zeroed - 360;
    }
    else if (zeroed < -PI)
    {
        return zeroed + 360;
    }
    else
    {
        return zeroed;
    }
}

/**
 * in degrees
 */
template <const int window_size>
class AngleMovingAvg
{
    float _values[window_size];
    size_t _ptr;

    AngleMovingAvg() : _ptr(0) {}

    void add(float v)
    {
        this->_values[this->_ptr] = v this._ptr++;
        this._ptr %= window_size;
    }

    float calc()
    {
        const size_t num_deltas = window_size - 1;
        float start_angle = this->_values[this->_ptr];

        float deltas[num_deltas];
        float last_angle = start_angle;
        for (int i = 0; i < num_deltas; i++)
        {
            auto next_val_ptr = (i + 1 + this->_ptr) % window_size;
            deltas[i] = dist(last_angle, this->_values[next_val_ptr]);
            last_angle = this->_values[next_val_ptr];
        }

        float total_deltas = 0;
        for (auto delta : deltas)
        {
            total_deltas += delta;
        }

        float avg_delta = total_deltas / num_deltas;
        return start_angle + avg_delta;
    }
};
// example:

ReadPWM lift_PWM(LIFT_E_p), pitch_PWM(MANIP_PITCH_E_p), roll_PWM(MANIP_ROLL_E_p);
PwmToAngleConverter lift_converter(), pitch_converter(), roll_converter();

void setup()
{
    Serial.begin(115200);

    CrcLib::Initialize();

    CrcLib::InitializePwmOutput(WHEEL_FL_M_p, false);
    CrcLib::InitializePwmOutput(WHEEL_FR_M_p, true); // Is normally true
    CrcLib::InitializePwmOutput(WHEEL_BL_M_p, false);
    CrcLib::InitializePwmOutput(WHEEL_BR_M_p, true); // Is normally true

    CrcLib::InitializePwmOutput(LIFT_L_M_p, true);
    CrcLib::InitializePwmOutput(LIFT_R_M_p, true);

    CrcLib::InitializePwmOutput(MANIP_PITCH_M_p, false);
    CrcLib::InitializePwmOutput(MANIP_ROLL_M_p, false);

    pinMode(BEAM_p, INPUT);

    // manip_belt_a.attach(MANIP_BELT_A_p);
    // manip_belt_b.attach(MANIP_BELT_B_p);

    print_timer.Start(PRINT_TIMER_DELAY);
}

void soft_kill()
{
    CrcLib::SetPwmOutput(WHEEL_BL_M_p, 0);
    CrcLib::SetPwmOutput(WHEEL_BR_M_p, 0);
    CrcLib::SetPwmOutput(WHEEL_FL_M_p, 0);
    CrcLib::SetPwmOutput(WHEEL_FR_M_p, 0);
    CrcLib::SetPwmOutput(LIFT_L_M_p, 0);
    CrcLib::SetPwmOutput(LIFT_R_M_p, 0);
    CrcLib::SetPwmOutput(MANIP_PITCH_M_p, 0);
    CrcLib::SetPwmOutput(MANIP_ROLL_M_p, 0);
    // manip_belt_a.write(0);
    // manip_belt_b.write(0);
}

int8_t clean_joystick_input(int8_t input)
{
    if (abs(input) < 10)
        return 0;
    auto constrained = constrain(input, -127, 127); // Fix max value to be symmetric
    return constrained;
}

void loop()
{
    CrcLib::Update();

    /**
     * GUARDS
     */

    static float battery_voltage_limit = 11.0;
    if (CrcLib::GetBatteryVoltage() < battery_voltage_limit)
    {
        battery_voltage_limit = 15.0f;
        Serial.println("Battery LOW: " + String(CrcLib::GetBatteryVoltage()));
        soft_kill();
        return;
    }

    if (!CrcLib::IsCommValid())
    {
        // block everything if controller is not connected
        soft_kill();
        Serial.println("no com");
        return;
    }

/**
 * CONTROLLER INPUT
 */
#define RAC(channel) CrcLib::ReadAnalogChannel(channel)
    int8_t left_X = clean_joystick_input(RAC(ANALOG::JOYSTICK1_X)) / 5;
    int8_t left_Y = clean_joystick_input(RAC(ANALOG::JOYSTICK1_Y)) / 5;
    int8_t right_X = clean_joystick_input(RAC(ANALOG::JOYSTICK2_X)) / 5;
    // int8_t right_Y = clean_joystick_input(RAC(ANALOG::JOYSTICK2_Y)) / 5;

    int8_t trig_L = RAC(ANALOG::GACHETTE_L);
    int8_t trig_R = RAC(ANALOG::GACHETTE_R);

    /**
     * SENSOR AQUISITIONS
     */

    uint32_t manip_pitch;
    pitch_PWM.read(manip_pitch);
    uint32_t manip_roll;
    roll_PWM.read(manip_roll);
    uint32_t lift_height;
    lift_PWM.read(lift_height);

    // uint32_t manip_pitch = pulseIn(MANIP_PITCH_E_p, HIGH, 1025);
    // uint32_t manip_roll = pulseIn(MANIP_ROLL_E_p, HIGH, 1025);
    // uint32_t lift_height = pulseIn(LIFT_E_p, HIGH, 1025);

    // uint32_t manip_pitch  = CrcLib::GetAnalogInput(MANIP_PITCH_E_p);
    // uint32_t manip_roll = CrcLib::GetAnalogInput(MANIP_ROLL_E_p);
    // uint32_t lift_height = CrcLib::GetAnalogInput(LIFT_E_p);

    bool beam_obstructed = CrcLib::GetDigitalInput(BEAM_p);

    /**
     * MOTOR_OUTPUTS
     */

    if (true)
    {
        /* WHEELS */
        CrcLib::MoveHolonomic(
            left_Y,
            right_X,
            left_X,
            WHEEL_FL_M_p, WHEEL_BL_M_p, WHEEL_FR_M_p, WHEEL_BR_M_p);
    }

    if (false && CrcLib::ReadDigitalChannel(BUTTON::COLORS_UP))
    {
        /* LIFT */
        CrcLib::SetPwmOutput(LIFT_L_M_p, trig_L);
        CrcLib::SetPwmOutput(LIFT_R_M_p, trig_R);
    }

    if (false && CrcLib::ReadDigitalChannel(BUTTON::COLORS_DOWN))
    {
        /* MANIPULATOR PITCH/ROLL */
        CrcLib::SetPwmOutput(MANIP_PITCH_M_p, trig_L);
        CrcLib::SetPwmOutput(MANIP_ROLL_M_p, trig_R);
    }

    if (false && CrcLib::ReadDigitalChannel(BUTTON::COLORS_RIGHT))
    {
        /* BELTS */
        // TODO: UNIMPLEMENTED
    }

    /**
     * SERIAL REPORTING
     */
    if (print_timer.IsFinished())
    {
        print_timer.Start(PRINT_TIMER_DELAY); // this will play catchup, but for now whatever
        Serial.println("Battery voltage: " + String(CrcLib::GetBatteryVoltage()));

        Serial.println("beam: " + String(beam_obstructed));

        Serial.println("h: " + String(lift_height * 360 / 1025) + "\tp: " + String(manip_pitch * 360 / 1025) + "\tr: " + String(manip_roll * 360 / 1025));

        Serial.println("triggers:\tL:" + String(trig_L) + "\tR: " + String(trig_R));
    }
}