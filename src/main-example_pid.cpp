#include <Arduino.h>
#include <CrcLib.h>

const bool CLOCKWISE = true, COUNTER_CLOCKWISE = false;
const char MAX_CLOCKWISE = 127, MAX_COUNTER_CLOCKWISE = -128;

const int pin_motor = CRC_PWM_2;

void setup()
{
  CrcLib::Initialize();
  CrcLib::InitializePwmOutput(pin_motor, false);
  Serial.begin(115200);
}

int8_t clean_joystick_input(int8_t input) {
  if (abs(input) < 5) {
    return 0;
  }
  auto constrainted = constrain(input, -127, 128); 
  return constrainted;
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
    int8_t frontRight = constrain(yawChannel + forwardChannel + strafeChannel, -128, 127); // Determines the power of the front left wheel
    int8_t frontLeft  = constrain(yawChannel - forwardChannel + strafeChannel, -128, 127); // Determines the power of the front left wheel
    int8_t backRight  = constrain(yawChannel + forwardChannel - strafeChannel, -128, 127); // Determines the power of the right wheels
    int8_t backLeft   = constrain(yawChannel - forwardChannel - strafeChannel, -128, 127); // Determines the power of the front left wheel

    CrcLib::SetPwmOutput(frontLeftMotor, frontLeft);
    CrcLib::SetPwmOutput(backLeftMotor, backLeft);
    CrcLib::SetPwmOutput(frontRightMotor, frontRight);
    CrcLib::SetPwmOutput(backRightMotor, backRight);
}

void loop()
{
  CrcLib::Update();

  if(millis() % 1 != 0){ 
    return;
  }

  if (CrcLib::IsCommValid()) {   
    #define RAC(channel) CrcLib::ReadAnalogChannel(channel)
    int8_t joy_stick_state_left_X = clean_joystick_input(RAC(ANALOG::JOYSTICK1_X));
    int8_t joy_stick_state_left_Y = clean_joystick_input(RAC(ANALOG::JOYSTICK1_Y));
    int8_t joy_stick_state_right_X = clean_joystick_input(RAC(ANALOG::JOYSTICK2_X));
    int8_t joy_stick_state_right_Y = clean_joystick_input(RAC(ANALOG::JOYSTICK2_Y));


    // if (millis() % 100 == 0)
    {
      MoveHolonomic(-joy_stick_state_left_Y, joy_stick_state_right_X, joy_stick_state_left_X, pin_FL, pin_BL, pin_FR, pin_BR);
      // CrcLib::MoveHolonomic(joy_stick_state_left_Y, joy_stick_state_right_X, joy_stick_state_left_X, pin_FL, pin_BL, pin_FR, pin_BR);

      Serial.print("LX" + String(joy_stick_state_left_X)+ "\t");
      Serial.print("LY" + String(joy_stick_state_left_Y)+ "\t");
      Serial.print("RX" + String (joy_stick_state_right_X)+ "\t");
      Serial.println("YD" + String(joy_stick_state_right_Y));
      Serial.println("=================================");
    }
  }

}
