#include <Arduino.h>
#include <CrcLib.h>

const int motor_speed_controller_pin = 6;
const bool CLOCKWISE = true, COUNTER_CLOCKWISE = false;
const char MAX_CLOCKWISE = 127, MAX_COUNTER_CLOCKWISE = -128;

void setup()
{
  CrcLib::Initialize();
  CrcLib::InitializePwmOutput(motor_speed_controller_pin);
  Serial.begin(115200);
}

void loop()
{
  CrcLib::Update();
  static char pwm_output = 0;
  static bool direction = CLOCKWISE;

  if (millis() % 100 == 0)
  {
    CrcLib::SetPwmOutput(motor_speed_controller_pin, pwm_output);

    if (direction == CLOCKWISE)
    {
      pwm_output += 1;
    }
    else if (direction == COUNTER_CLOCKWISE)
    {
      pwm_output -= 1;
    }

    if (pwm_output == MAX_CLOCKWISE)
    {
      direction = COUNTER_CLOCKWISE;
    }
    else if (pwm_output == MAX_COUNTER_CLOCKWISE)
    {
      direction = CLOCKWISE;
    }
  }
}