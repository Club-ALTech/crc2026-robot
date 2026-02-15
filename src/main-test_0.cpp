#include <Arduino.h>
#include <CrcLib.h>

const bool CLOCKWISE = true, COUNTER_CLOCKWISE = false;
const char MAX_CLOCKWISE = 127, MAX_COUNTER_CLOCKWISE = -128;

const int wheel_FL = CRC_PWM_3;
const int wheel_FR = CRC_PWM_10;
const int wheel_BL = CRC_PWM_1;
const int wheel_BR = CRC_PWM_11;

void setup()
{
  CrcLib::Initialize();
  CrcLib::InitializePwmOutput(wheel_BL);
 CrcLib::InitializePwmOutput(wheel_BR, true);
  CrcLib::InitializePwmOutput(wheel_FL, true);
  CrcLib::InitializePwmOutput(wheel_FR);
  Serial.begin(115200);
}

int8_t clean_joystick_input(int8_t input) {
  if (abs(input) < 15) {
    return 0;
  }
  auto constrainted = constrain(input, -127, 127); 
  return constrainted;
}

void loop()
{
    CrcLib::Update();

    CrcLib::SetPwmOutput(wheel_BL, 0);
    CrcLib::SetPwmOutput(wheel_BR, 0);
    CrcLib::SetPwmOutput(wheel_FL, 0);
    CrcLib::SetPwmOutput(wheel_FR, 0);
}