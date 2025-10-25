#include <Arduino.h>
#include <CrcLib.h>

const bool CLOCKWISE = true, COUNTER_CLOCKWISE = false;
const char MAX_CLOCKWISE = 127, MAX_COUNTER_CLOCKWISE = -128;

// Pin definitions
const int pin_FL = CRC_PWM_3;
const int pin_FR = CRC_PWM_10;
const int pin_BL = CRC_PWM_1;
const int pin_BR = CRC_PWM_11;

// Robot physical parameters (in meters)
const float WHEEL_RADIUS = 0.086;  // 2 inches = 0.0508 meters
const float L = 0.625;  // Length between wheel centers (9 inches = 0.2286 meters)
const float W = 0.280;  // Width between wheel centers (9 inches = 0.2286 meters)

// Maximum speeds
const float MAX_LINEAR_SPEED = 1.0;  // meters per second
const float MAX_ANGULAR_SPEED = 2.0;  // radians per second

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
  CrcLib::InitializePwmOutput(pin_BL, false);
  CrcLib::InitializePwmOutput(pin_BR, false);
  CrcLib::InitializePwmOutput(pin_FL, false);
  CrcLib::InitializePwmOutput(pin_FR, false);
  Serial.begin(115200);
}

int8_t clean_joystick_input(int8_t input) {
  if (abs(input) < 15) {
    return 0;
  }
  auto constrainted = constrain(input, -127, 127); 
  return constrainted;
}

// Convert joystick input (-127 to 127) to actual speeds
float joystickToLinearVelocity(int8_t input) {
    return (float)input * MAX_LINEAR_SPEED / 127.0;
}

float joystickToAngularVelocity(int8_t input) {
    return (float)input * MAX_ANGULAR_SPEED / 127.0;
}

// Inverse kinematics for mecanum drive
// vx: forward velocity (m/s)
// vy: sideways velocity (m/s)
// omega: angular velocity (rad/s)
void inverseKinematics(float vx, float vy, float omega, float* wheel_speeds) {
    // Calculate wheel speeds using inverse kinematics matrix
    wheel_speeds[0] = (vx - vy - (L + W) * omega) / WHEEL_RADIUS;  // Front Left
    wheel_speeds[1] = (vx + vy + (L + W) * omega) / WHEEL_RADIUS;  // Front Right
    wheel_speeds[2] = (vx + vy - (L + W) * omega) / WHEEL_RADIUS;  // Back Left
    wheel_speeds[3] = (vx - vy + (L + W) * omega) / WHEEL_RADIUS;  // Back Right

    // Find the maximum speed
    float max_speed = 0;
    for (int i = 0; i < 4; i++) {
        if (abs(wheel_speeds[i]) > max_speed) {
            max_speed = abs(wheel_speeds[i]);
        }
    }

    // Normalize speeds if they exceed maximum
    if (max_speed > MAX_LINEAR_SPEED) {
        float scale = MAX_LINEAR_SPEED / max_speed;
        for (int i = 0; i < 4; i++) {
            wheel_speeds[i] *= scale;
        }
    }

    // Convert to PWM range (-127 to 127)
    for (int i = 0; i < 4; i++) {
        wheel_speeds[i] = (wheel_speeds[i] * 127.0) / MAX_LINEAR_SPEED;
    }
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
    // Convert joystick inputs to velocities
    float vx = joystickToLinearVelocity(forwardChannel);   // Forward velocity
    float vy = joystickToLinearVelocity(strafeChannel);    // Sideways velocity
    float omega = joystickToAngularVelocity(yawChannel);   // Angular velocity

    // Calculate wheel speeds using inverse kinematics
    float wheel_speeds[4];
    inverseKinematics(vx, vy, omega, wheel_speeds);

    // Set target speeds from kinematic calculations
    int8_t target_FL = (int8_t)wheel_speeds[0];
    int8_t target_FR = (int8_t)wheel_speeds[1];
    int8_t target_BL = (int8_t)wheel_speeds[2];
    int8_t target_BR = (int8_t)wheel_speeds[3];

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

      MoveHolonomic(-joy_stick_state_left_Y, joy_stick_state_right_X, joy_stick_state_left_X, pin_FL, pin_BL, pin_FR, pin_BR);

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
    CrcLib::SetPwmOutput(pin_BL, 0);
    CrcLib::SetPwmOutput(pin_BR, 0);
    CrcLib::SetPwmOutput(pin_FL, 0);
    CrcLib::SetPwmOutput(pin_FR, 0);
  }
}
