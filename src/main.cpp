#include <Arduino.h>
#include <CrcLib.h>

const int motor_speed_controller_pin = 6;

void setup()
{
  CrcLib::Initialize();
  CrcLib::InitializePwmOutput(motor_speed_controller_pin);
  Serial.begin(115200);
}

void loop()
{
  CrcLib::Update();
  static unsigned char pwm_output = 0;

  if (millis() % 100 == 0) {
    CrcLib::SetPwmOutput(motor_speed_controller_pin, pwm_output);

    pwm_output += 1; // overflows, we dont mind.
  }
}