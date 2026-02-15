#include <Arduino.h>
#include <CrcLib.h>

const bool CLOCKWISE = true, COUNTER_CLOCKWISE = false;
const char MAX_CLOCKWISE = 127, MAX_COUNTER_CLOCKWISE = -128;

const int wheel_FL = CRC_PWM_3;
const int wheel_FR = CRC_PWM_10;
const int wheel_BL = CRC_PWM_1;
const int wheel_BR = CRC_PWM_11;

// Current motor speeds
int8_t current_FL = 0;
int8_t current_FR = 0;
int8_t current_BL = 0;
int8_t current_BR = 0;

// Acceleration parameters
const float ACCEL_RATE = 0.45; // Rate of acceleration (0-1). Higher = faster response

void setup()
{
  CrcLib::Initialize();
  CrcLib::InitializePwmOutput(wheel_BL, false);
  CrcLib::InitializePwmOutput(wheel_BR, false);
  CrcLib::InitializePwmOutput(wheel_FL, false);
  CrcLib::InitializePwmOutput(wheel_FR, false);
  Serial.begin(115200);
}

int8_t clean_joystick_input(int8_t input) {
  if (abs(input) < 15) {
    return 0;
  }
  auto constrainted = constrain(input, -127, 127); 
  return constrainted;
}

// Function to smoothly accelerate towards target speed
int8_t accelerateTowards(int8_t current, int8_t target) {
    if (current == target) {
        return current;
    }
    
    float diff = target - current;
    int8_t change = (int8_t)(diff * ACCEL_RATE);
    
    // Ensure we always move at least 1 unit if we're not at target
    if (change == 0) {
        change = (diff > 0) ? 1 : -1;
    }
    
    return current + change;
}

void MoveHolonomic(
    int8_t forwardChannel,
    int8_t yawChannel,
    int8_t strafeChannel,
    unsigned char frontLeftMotor,
    unsigned char backLeftMotor,
    unsigned char frontRightMotor,
    unsigned char backRightMotor)
{
    // Calculate target speeds
    int8_t target_FR = constrain(yawChannel + forwardChannel + strafeChannel, -128, 127);
    int8_t target_FL = constrain(yawChannel - forwardChannel + strafeChannel, -128, 127);
    int8_t target_BR = constrain(yawChannel + forwardChannel - strafeChannel, -128, 127);
    int8_t target_BL = constrain(yawChannel - forwardChannel - strafeChannel, -128, 127);

    // Accelerate towards target speeds
    current_FL = accelerateTowards(current_FL, target_FL);
    current_FR = accelerateTowards(current_FR, target_FR);
    current_BL = accelerateTowards(current_BL, target_BL);
    current_BR = accelerateTowards(current_BR, target_BR);

    // Apply the accelerated speeds to motors
    CrcLib::SetPwmOutput(frontLeftMotor, current_FL);
    CrcLib::SetPwmOutput(backLeftMotor, current_BL);
    CrcLib::SetPwmOutput(frontRightMotor, current_FR);
    CrcLib::SetPwmOutput(backRightMotor, current_BR);
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

  if(millis() % 100 != 0){ 
    return;
  }

  if (CrcLib::IsCommValid()) {   
    #define RAC(channel) CrcLib::ReadAnalogChannel(channel)
    int8_t joy_stick_state_left_X = clean_joystick_input(RAC(ANALOG::JOYSTICK1_X));
    int8_t joy_stick_state_left_Y = clean_joystick_input(RAC(ANALOG::JOYSTICK1_Y));
    int8_t joy_stick_state_right_X = clean_joystick_input(RAC(ANALOG::JOYSTICK2_X));
    // int8_t joy_stick_state_right_Y = clean_joystick_input(RAC(ANALOG::JOYSTICK2_Y));

    if (millis() % 1 == 0)
    {
      // Serial.println("=================================");
      // Serial.print("pin_BL_LY" + String((int8_t)joy_stick_state_left_Y) + "\t");
      // Serial.print("pin_BR_LY" + String((int8_t)-joy_stick_state_left_Y) + "\t");
      // Serial.print("pin_FL_LY" + String((int8_t)joy_stick_state_left_Y) + "\t");
      // Serial.println("pin_FR_LY" + String((int8_t)-joy_stick_state_left_Y));
      // Serial.println("================");
      // Serial.print("pin_BL_LX" + String((int8_t)joy_stick_state_left_X) + "\t");
      // Serial.print("pin_BR_LX" + String((int8_t)-joy_stick_state_left_X) + "\t");
      // Serial.print("pin_FL_LX" + String((int8_t)joy_stick_state_left_X) + "\t");
      // Serial.println("pin_FR_LX" + String((int8_t)-joy_stick_state_left_X));
      // Serial.println("=================================");

      MoveHolonomic(-joy_stick_state_left_Y, joy_stick_state_right_X, joy_stick_state_left_X, wheel_FL, wheel_BL, wheel_FR, wheel_BR);

      // // Movement vers l'avant/arriere
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

      Serial.print("LX" + String(joy_stick_state_left_X)+ "\t");
      Serial.print("LY" + String(joy_stick_state_left_Y)+ "\t");
      Serial.print("RX" + String (joy_stick_state_right_X)+ "\t");
      // Serial.println("YD" + String(joy_stick_state_right_Y));
      Serial.println("================");
    }
  }

  else {
    CrcLib::SetPwmOutput(wheel_BL, 0);
    CrcLib::SetPwmOutput(wheel_BR, 0);
    CrcLib::SetPwmOutput(wheel_FL, 0);
    CrcLib::SetPwmOutput(wheel_FR, 0);
  }
}
