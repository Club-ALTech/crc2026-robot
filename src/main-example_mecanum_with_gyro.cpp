#include <Arduino.h>
#include <CrcLib.h>
#include <Wire.h>
#include <math.h>
#include "navx.hpp"

const int pin_FL = CRC_PWM_2;
const int pin_FR = CRC_PWM_3;
const int pin_BL = CRC_PWM_1;
const int pin_BR = CRC_PWM_4;

byte data[512];

float battery_voltage_limit = 11.0f;

void setup()
{
  CrcLib::Initialize();

  CrcLib::InitializePwmOutput(pin_BL, false);
  CrcLib::InitializePwmOutput(pin_BR, true); // Is normally true
  CrcLib::InitializePwmOutput(pin_FL, false);
  CrcLib::InitializePwmOutput(pin_FR, true); // Is normally true
  Serial.begin(115200);

  Wire.begin(); // join i2c bus (address optional for master)

  memset(data, 0, sizeof(data)); // Setting all data to zero
}

int8_t clean_joystick_input(int8_t input)
{
  if (abs(input) < 10)
    return 0;
  auto constrained = constrain(input, -127, 127); // Fix max value to be symmetric
  return constrained;
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

  if (millis() % 10 != 0)
  {
    return;
  }

  Serial.println("Battery voltage: " + String(CrcLib::GetBatteryVoltage()));
  if (CrcLib::GetBatteryVoltage() < battery_voltage_limit)
  {
    battery_voltage_limit = 15.0f;
    Serial.println("Battery voltage: " + String(CrcLib::GetBatteryVoltage()));
    CrcLib::SetPwmOutput(pin_BL, 0);
    CrcLib::SetPwmOutput(pin_BR, 0);
    CrcLib::SetPwmOutput(pin_FL, 0);
    CrcLib::SetPwmOutput(pin_FR, 0);
    return;
  }

  if (CrcLib::IsCommValid())
  {
#define RAC(channel) CrcLib::ReadAnalogChannel(channel)
    int8_t joy_stick_state_left_X = clean_joystick_input(RAC(ANALOG::JOYSTICK1_X));
    int8_t joy_stick_state_left_Y = clean_joystick_input(RAC(ANALOG::JOYSTICK1_Y));
    int8_t joy_stick_state_right_X = clean_joystick_input(RAC(ANALOG::JOYSTICK2_X));

    // Get current heading from gyro
    NavX::Heading h = NavX::get_gyro_info(data);

    // Convert joystick inputs to field-centric
    NavX::FieldCentricInput robotCentric = NavX::convertToRobotCentric(
        joy_stick_state_left_Y,   // Forward
        -joy_stick_state_left_X,  // Strafe
        -joy_stick_state_right_X, // Rotation
        h.yaw * 2);

    // Apply converted values to motors
    CrcLib::MoveHolonomic(
        robotCentric.forward,
        robotCentric.rotation,
        robotCentric.strafe,
        pin_FL, pin_BL, pin_FR, pin_BR);
  }
}