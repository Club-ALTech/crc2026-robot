#include <Arduino.h>
#include <CrcLib.h>

const bool CLOCKWISE = true, COUNTER_CLOCKWISE = false;
const char MAX_CLOCKWISE = 127, MAX_COUNTER_CLOCKWISE = -128;

const int pin_FL = CRC_PWM_3;
const int pin_FR = CRC_PWM_10;
const int pin_BL = CRC_PWM_1;
const int pin_BR = CRC_PWM_11;

void setup()
{
  CrcLib::Initialize();
  CrcLib::InitializePwmOutput(pin_BL);
  CrcLib::InitializePwmOutput(pin_BR, true);
  CrcLib::InitializePwmOutput(pin_FL, true);
  CrcLib::InitializePwmOutput(pin_FR);
  Serial.begin(115200);
}

int8_t clean_joystick_input(int8_t input) {
  if (abs(input) < 15) {
    return 0;
  }
  auto constrainted = constrain(input, -127, 127); 
  return constrainted;
}

// manette: CrcLib::ReadAnalogChannel(ANALOG::JOYSTICK1_X)
void loop()
{
  CrcLib::Update();

  if(millis() % 100 != 0){ 
    return;
  }

  if (CrcLib::IsCommValid()) {   
    #define RAC(channel) CrcLib::ReadAnalogChannel(channel)
    int8_t joy_stick_state_left_X = clean_joystick_input(RAC(ANALOG::JOYSTICK1_X));
    int8_t joy_stick_state_left_Y = clean_joystick_input(RAC(ANALOG::JOYSTICK1_Y));
    // int8_t joy_stick_state_right_X = clean_joystick_input(RAC(ANALOG::JOYSTICK2_X));
    // int8_t joy_stick_state_right_Y = clean_joystick_input(RAC(ANALOG::JOYSTICK2_Y));
    
    if(millis() % 1 == 0){
      Serial.println("=================================");
      Serial.print("pin_BL_LY" + String((int8_t)joy_stick_state_left_Y) + "\t");
      Serial.print("pin_BR_LY" + String((int8_t)-joy_stick_state_left_Y)+ "\t");
      Serial.print("pin_FL_LY" + String((int8_t)joy_stick_state_left_Y)+ "\t");
      Serial.println("pin_FR_LY" + String((int8_t)-joy_stick_state_left_Y));
      Serial.println("================");
      Serial.print("pin_BL_LX" + String((int8_t)joy_stick_state_left_X) + "\t");
      Serial.print("pin_BR_LX" + String((int8_t)-joy_stick_state_left_X)+ "\t");
      Serial.print("pin_FL_LX" + String((int8_t)joy_stick_state_left_X)+ "\t");
      Serial.println("pin_FR_LX" + String((int8_t)-joy_stick_state_left_X));
      Serial.println("=================================");
    }

    CrcLib::MoveHolonomic(ANALOG::JOYSTICK1_Y,ANALOG::JOYSTICK2_X, ANALOG::JOYSTICK1_X, pin_FL, pin_BL, pin_FR, pin_BR);

    // Movement vers l'avant/arriere
    // CrcLib::SetPwmOutput(pin_BL, joy_stick_state_left_Y);
    // CrcLib::SetPwmOutput(pin_BR, -joy_stick_state_left_Y);
    // CrcLib::SetPwmOutput(pin_FL, joy_stick_state_left_Y);
    // CrcLib::SetPwmOutput(pin_FR, -joy_stick_state_left_Y);
    // // Movement sur les cotes(droite/gauche)
    // CrcLib::SetPwmOutput(pin_BL, joy_stick_state_left_X);
    // CrcLib::SetPwmOutput(pin_BR, joy_stick_state_left_X);
    // CrcLib::SetPwmOutput(pin_FL, -joy_stick_state_left_X);
    // CrcLib::SetPwmOutput(pin_FR, -joy_stick_state_left_X);
    // Serial.println("=================================");

    // Serial.print("XG" + String(joy_stick_state_left_X)+ "\t");
    // Serial.print("YG" + String(joy_stick_state_left_Y)+ "\t");
    // Serial.print("XD" + String (joy_stick_state_right_X)+ "\t");
    // Serial.println("YD" + String(joy_stick_state_right_Y));
    // Serial.println("================");


  }

  else {
    CrcLib::SetPwmOutput(pin_BL, 0);
    CrcLib::SetPwmOutput(pin_BR, 0);
    CrcLib::SetPwmOutput(pin_FL, 0);
    CrcLib::SetPwmOutput(pin_FR, 0);
  }
}
