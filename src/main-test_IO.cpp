#include <Arduino.h>
#include <CrcLib.h>
#include <Wire.h>
#include <math.h>
#include "navx.hpp"
#include <Servo.h>

const int wheel_FL_m = CRC_PWM_1;
const int wheel_FR_m = CRC_PWM_3;
const int wheel_BL_m = CRC_PWM_2;
const int wheel_BR_m = CRC_PWM_4;

const int lift_L_m = CRC_PWM_5;
const int lift_R_m = CRC_PWM_6;
const int lift_e = CRC_DIG_2; // lift height

const int manip_pitch_e = CRC_DIG_2;
const int manip_pitch_m = CRC_PWM_7;

const int manip_roll_e = CRC_DIG_2;
const int manip_roll_m = CRC_PWM_8;

const int manip_belt_a_p = CRC_PWM_9;
const int manip_belt_b_p = CRC_PWM_10;

const int beam_p = CRC_DIG_1;

Servo manip_belt_a, manip_belt_b;

void setup()
{
    CrcLib::Initialize();

    CrcLib::InitializePwmOutput(wheel_FL_m, false);
    CrcLib::InitializePwmOutput(wheel_FR_m, true); // Is normally true
    CrcLib::InitializePwmOutput(wheel_BL_m, false);
    CrcLib::InitializePwmOutput(wheel_BR_m, true); // Is normally true

    CrcLib::InitializePwmOutput(lift_L_m, true);
    CrcLib::InitializePwmOutput(lift_R_m, true);

    CrcLib::InitializePwmOutput(manip_pitch_m, false);
    CrcLib::InitializePwmOutput(manip_roll_m, false);

    manip_belt_a.attach(manip_belt_a_p);
    manip_belt_b.attach(manip_belt_b_p);

    Serial.begin(115200);
}

void soft_kill()
{
    CrcLib::SetPwmOutput(wheel_BL_m, 0);
    CrcLib::SetPwmOutput(wheel_BR_m, 0);
    CrcLib::SetPwmOutput(wheel_FL_m, 0);
    CrcLib::SetPwmOutput(wheel_FR_m, 0);
    CrcLib::SetPwmOutput(lift_L_m, 0);
    CrcLib::SetPwmOutput(lift_R_m, 0);
    CrcLib::SetPwmOutput(manip_pitch_m, 0);
    CrcLib::SetPwmOutput(manip_roll_m, 0);
    manip_belt_a.write(0);
    manip_belt_b.write(0);
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
     * SENSOR AQUISITIONS
     */

#define RAC(channel) CrcLib::ReadAnalogChannel(channel)
    int8_t left_X = clean_joystick_input(RAC(ANALOG::JOYSTICK1_X));
    int8_t left_Y = clean_joystick_input(RAC(ANALOG::JOYSTICK1_Y));
    int8_t right_X = clean_joystick_input(RAC(ANALOG::JOYSTICK2_X));
    int8_t right_Y = clean_joystick_input(RAC(ANALOG::JOYSTICK2_Y));

    int8_t trig_L = RAC(ANALOG::GACHETTE_L);
    int8_t trig_R = RAC(ANALOG::GACHETTE_R);

    uint32_t manip_pitch = pulseIn(manip_pitch_e, HIGH);
    uint32_t manip_roll = pulseIn(manip_roll_e, HIGH);
    uint32_t lift_height = pulseIn(lift_e, HIGH);

    bool beam_obstructed = CrcLib::GetDigitalInput(beam_p);

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
     * MOTOR_OUTPUTS
     */

    if (true)
    {
        /* WHEELS */
        CrcLib::MoveHolonomic(
            left_Y,
            right_X,
            left_X,
            wheel_FL_m, wheel_BL_m, wheel_FR_m, wheel_BR_m);
    }

    if (false && CrcLib::ReadDigitalChannel(BUTTON::COLORS_UP))
    {
        /* LIFT */
        CrcLib::SetPwmOutput(lift_L_m, trig_L);
        CrcLib::SetPwmOutput(lift_R_m, trig_R);
    }

    if (false && CrcLib::ReadDigitalChannel(BUTTON::COLORS_DOWN))
    {
        /* MANIPULATOR PITCH/ROLL */
        CrcLib::SetPwmOutput(manip_pitch_m, trig_L);
        CrcLib::SetPwmOutput(manip_roll_m, trig_R);
    }

    if (false && CrcLib::ReadDigitalChannel(BUTTON::COLORS_RIGHT))
    {
        /* BELTS */
        // TODO: UNIMPLEMENTED
    }

    /**
     * SERIAL REPORTING
     */
    const long freq_hz = 40;
    if (millis() % (1000 / freq_hz) != 0)
    {
        return;
    }

    Serial.println("Battery voltage: " + String(CrcLib::GetBatteryVoltage()));

    Serial.println("beam: " + String(beam_obstructed));

    Serial.println("h: " + String(lift_height) + "\tp: " + String(manip_pitch) + "\tr: " + String(manip_roll));

    Serial.println("triggers:\tL:" + String(trig_L) + "\tR: " + String(trig_R));
}