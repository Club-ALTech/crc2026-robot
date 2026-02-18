#include <Arduino.h>
#include <CrcLib.h>

const int pin = CRC_PWM_12;

void setup()
{
    CrcLib::Initialize();
    CrcLib::InitializePwmOutput(pin, false);
}

void loop()
{
    CrcLib::Update();
    if (!CrcLib::IsCommValid())
    {
        CrcLib::SetPwmOutput(pin, 0);
        return;
    }

    auto val = CrcLib::ReadAnalogChannel(ANALOG::JOYSTICK1_X)/5;

    CrcLib::SetPwmOutput(pin, val);
}