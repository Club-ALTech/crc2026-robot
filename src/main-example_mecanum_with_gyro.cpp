#include <Arduino.h>
#include <CrcLib.h>
#include <Wire.h>
#include <math.h>
#include "navx.hpp"
#include <PID_v1.h>


const int wheel_FL = CRC_PWM_1;
const int wheel_FR = CRC_PWM_3;
const int wheel_BL = CRC_PWM_2;
const int wheel_BR = CRC_PWM_4;
const int pin_test = CRC_PWM_12;


byte data[512];
float battery_voltage_limit = 11.0f;

double pid_input, pid_output, pid_setpoint = 0;
double kp = 3, ki=0.00000, kd=0.0;
PID pid(&pid_input, &pid_output, &pid_setpoint, kp, ki, kd, P_ON_E, DIRECT);
double OUTPUT_LIM = 60;

void setup()
{
    CrcLib::Initialize();

    CrcLib::InitializePwmOutput(wheel_BL, false);
    CrcLib::InitializePwmOutput(wheel_BR, true); // Is normally true
    CrcLib::InitializePwmOutput(wheel_FL, false);
    CrcLib::InitializePwmOutput(wheel_FR, true); // Is normally true
    Serial.begin(115200);

    Wire.begin(); // join i2c bus (address optional for master)

    memset(data, 0, sizeof(data)); // Setting all data to zero
    pid.SetMode(AUTOMATIC);
    pid.SetSampleTime(1);
    pid.SetOutputLimits(-OUTPUT_LIM, OUTPUT_LIM);
}

void soft_kill()
{
    CrcLib::SetPwmOutput(wheel_BL, 0);
    CrcLib::SetPwmOutput(wheel_BR, 0);
    CrcLib::SetPwmOutput(wheel_FL, 0);
    CrcLib::SetPwmOutput(wheel_FR, 0);
}

int8_t clean_joystick_input(int8_t input)
{
    if (abs(input) < 10)
        return 0;
    auto constrained = constrain(input, -127, 127); // Fix max value to be symmetric
    return constrained;
}

static double travel(float source_deg, float destination_deg)
{
    auto zeroed = destination_deg - source_deg;
    if (zeroed > 180)
    {
        return zeroed - 360;
    }
    else if (zeroed < -180)
    {
        return zeroed + 360;
    }
    else
    {
        return zeroed;
    }
}

//   ^
//   |
// Y |
//   |
//   |
//   +------------->
//          X
void loop()
{
    CrcLib::Update();

    // Serial.println("Battery voltage: " + String(CrcLib::GetBatteryVoltage()));
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

    NavX::Heading h = NavX::get_gyro_info(data);
    float current_rotation = h.yaw;
    
    // TODO: fix navx instead, will make a better resolution
    if (current_rotation < 180) {
        current_rotation = map(current_rotation, 0.0, 100.0, 0.0, 180.0);
    } else {
        current_rotation = map(current_rotation, 260.0, 360.0, 180.0, 360.0);
    }
    
    static float target_rotation = NAN;
    if (isnan(target_rotation))
    {
        // NOTE: intialize target to the current rotation to avoid the robot trying to move right after turning on
        target_rotation = current_rotation;
    }

#define RAC(channel) CrcLib::ReadAnalogChannel(channel)
    int8_t left_X = clean_joystick_input(RAC(ANALOG::JOYSTICK1_X));
    int8_t left_Y = clean_joystick_input(RAC(ANALOG::JOYSTICK1_Y));
    int8_t right_X = clean_joystick_input(RAC(ANALOG::JOYSTICK2_X));
    int8_t right_Y = clean_joystick_input(RAC(ANALOG::JOYSTICK2_Y));

    if (abs(right_X) > 5 || abs(right_Y) > 5)
    {
        target_rotation = 180 - (atan2((float)right_X, (float)right_Y) * 180 / PI);
    }

    

    auto travel_deg = travel(current_rotation, target_rotation);
    pid_input = travel_deg;

    if (!pid.Compute()) {
        return;
    }

    Serial.println("output: " + String(pid_output));
    
    // Convert joystick inputs to field-centric
    NavX::FieldCentricInput robotCentric = NavX::convertToRobotCentric(
        left_Y,  // Forward
        -left_X, // Strafe
        pid_output, // Rotation
        0        // h.yaw * 2
    );

    // Apply converted values to motors
    CrcLib::MoveHolonomic(
        robotCentric.forward,
        robotCentric.rotation,
        robotCentric.strafe,
        wheel_FL, wheel_BL, wheel_FR, wheel_BR);

    if (millis() % 25 != 0)
    {
        return;
    }
    
    // Get current heading from gyro
    Serial.println("expected: " + String(target_rotation) + "\tactual: " + String(current_rotation));
    
}