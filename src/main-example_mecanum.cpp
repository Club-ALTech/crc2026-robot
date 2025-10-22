#include <Arduino.h>
#include <CrcLib.h>

const bool CLOCKWISE = true, COUNTER_CLOCKWISE = false;
const char MAX_CLOCKWISE = 127, MAX_COUNTER_CLOCKWISE = -128;

const int motor_speed_controller_pin_FL = CRC_PWM_3;
const int motor_speed_controller_pin_FR = CRC_PWM_10;
const int motor_speed_controller_pin_BR = CRC_PWM_11;
const int motor_speed_controller_pin_BL = CRC_PWM_12;

void setup()
{
  CrcLib::Initialize();
  CrcLib::InitializePwmOutput(motor_speed_controller_pin_BL);
  CrcLib::InitializePwmOutput(motor_speed_controller_pin_BR);
  CrcLib::InitializePwmOutput(motor_speed_controller_pin_FL);
  CrcLib::InitializePwmOutput(motor_speed_controller_pin_FR);
  Serial.begin(115200);
}


// manette: CrcLib::ReadAnalogChannel(ANALOG::JOYSTICK1_X)
void loop()
{
  CrcLib::Update();

  if(millis() % 100 != 0){ 
    return;
  }

  if (CrcLib::IsCommValid()) {   
    int8_t joy_stick_state_left_X = CrcLib::ReadAnalogChannel(ANALOG::JOYSTICK1_X);
    int8_t joy_stick_state_left_Y = CrcLib::ReadAnalogChannel(ANALOG::JOYSTICK1_Y);
    int8_t joy_stick_state_right_X = CrcLib::ReadAnalogChannel(ANALOG::JOYSTICK2_X);
    int8_t joy_stick_state_right_Y = CrcLib::ReadAnalogChannel(ANALOG::JOYSTICK2_Y);

    joy_stick_state_left_X = constrain(joy_stick_state_left_X, -127, 127);
    joy_stick_state_left_Y = constrain(joy_stick_state_left_Y, -127, 127);
    joy_stick_state_right_X = constrain(joy_stick_state_right_X, -127, 127);
    joy_stick_state_right_Y = constrain(joy_stick_state_right_Y, -127, 127);

    CrcLib::SetPwmOutput(motor_speed_controller_pin_BL, joy_stick_state_left_Y);
    CrcLib::SetPwmOutput(motor_speed_controller_pin_BR, -joy_stick_state_left_Y);
    CrcLib::SetPwmOutput(motor_speed_controller_pin_FL, joy_stick_state_left_Y);
    CrcLib::SetPwmOutput(motor_speed_controller_pin_FR, -joy_stick_state_left_Y);
    Serial.println("=================================");

    Serial.print("XG" + String(joy_stick_state_left_X)+ "\t");
    Serial.print("YG" + String(joy_stick_state_left_Y)+ "\t");
    Serial.print("XD" + String (joy_stick_state_right_X)+ "\t");
    Serial.println("YD" + String(joy_stick_state_right_Y));
    Serial.println("================");
    // Serial.print("BL" + String()+ "\t");
    // Serial.print("BR" + String(-)+ "\t");
    // Serial.print("FL" + String()+ "\t");
    // Serial.println("FR" + String(-));
    // Serial.print("BL" + String(motor_speed_controller_pin_BL)+ "\t");
    // Serial.print("BR" + String(motor_speed_controller_pin_BR)+ "\t");
    // Serial.print("FL" + String(motor_speed_controller_pin_FL)+ "\t");
    // Serial.println("FR" + String(motor_speed_controller_pin_FR));


  }
  else {
    CrcLib::SetPwmOutput(motor_speed_controller_pin_BL, 0);
    CrcLib::SetPwmOutput(motor_speed_controller_pin_BR, 0);
    CrcLib::SetPwmOutput(motor_speed_controller_pin_FL, 0);
    CrcLib::SetPwmOutput(motor_speed_controller_pin_FR, 0);
  }
}
