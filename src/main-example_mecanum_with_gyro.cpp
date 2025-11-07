#include "AHRSProtocol.h"
#include <Arduino.h>
#include <CrcLib.h>
#include <Wire.h>
#include <math.h>

const bool CLOCKWISE = true, COUNTER_CLOCKWISE = false;
const char MAX_CLOCKWISE = 127, MAX_COUNTER_CLOCKWISE = -128;

const int pin_FL = CRC_PWM_2;
const int pin_FR = CRC_PWM_3;
const int pin_BL = CRC_PWM_1;
const int pin_BR = CRC_PWM_4;

byte data[512];

#define ITERATION_DELAY_MS 10
#define NAVX_SENSOR_DEVICE_I2C_ADDRESS_7BIT 0x32
#define NUM_BYTES_TO_READ 8

int register_address = NAVX_REG_YAW_L;
float battery_voltage_limit = 11.5f;

void setup()
{
  CrcLib::Initialize();

  CrcLib::InitializePwmOutput(pin_BL, false);
  CrcLib::InitializePwmOutput(pin_BR, true); // Is normally true
  CrcLib::InitializePwmOutput(pin_FL, false);
  CrcLib::InitializePwmOutput(pin_FR, true); // Is normally true
  Serial.begin(115200);
  Wire.begin(); // join i2c bus (address optional for master)

  memset(data, 0, sizeof(data));
}

struct Heading
{
  float yaw;
  float pitch;
  float roll;
  float heading;
};

struct FieldCentricInput
{
  int8_t forward;
  int8_t strafe;
  double rotation;
};

// Convert field-centric inputs to robot-centric
FieldCentricInput convertToRobotCentric(double forward, double strafe, double rotation, double gyroAngle)
{
  // Convert gyro angle to radians
  double angleRad = (gyroAngle * PI) / 180.0;

  FieldCentricInput result;
  // Perform field-centric to robot-centric conversion
  result.forward = constrain(forward * cos(angleRad) + strafe * sin(angleRad), -127, 128);
  result.strafe = constrain(-forward * sin(angleRad) + strafe * cos(angleRad), -127, 128);
  result.rotation = rotation;
  Serial.print("forward: " + String(forward) + " strafe: " + String(strafe) + " rotation: " + String(rotation));
  Serial.println(" forward: " + String(result.forward) + " strafe: " + String(result.strafe) + " rotation: " + String(result.rotation));

  return result;
}

Heading gyro_thingy()
{
  // CrcLib::Update();
  /* Transmit I2C data request */
  Wire.beginTransmission(NAVX_SENSOR_DEVICE_I2C_ADDRESS_7BIT); // Begin transmitting to navX-Sensor
  Wire.write(register_address);                                // Sends starting register address
  Wire.write(NUM_BYTES_TO_READ);                               // Send number of bytes to read
  Wire.endTransmission();                                      // Stop transmitting

  /* Receive the echoed value back */
  Wire.beginTransmission(NAVX_SENSOR_DEVICE_I2C_ADDRESS_7BIT);              // Begin transmitting to navX-Sensor
  Wire.requestFrom(NAVX_SENSOR_DEVICE_I2C_ADDRESS_7BIT, NUM_BYTES_TO_READ); // Send number of bytes to read
  for (size_t i = 0; Wire.available(); i++)
  { // Read data (slave may send less than requested)
    data[i++] = Wire.read();
  }
  Wire.endTransmission(); // Stop transmitting

  /* Decode received data to floating-point orientation values */
  float pitch = IMURegisters::decodeProtocolSignedHundredthsFloat((char *)&data[0]) / 2.55;     // The cast is needed on arduino
  float yaw = IMURegisters::decodeProtocolSignedHundredthsFloat((char *)&data[2]) / 2.55;       // The cast is needed on arduino
  float roll = IMURegisters::decodeProtocolSignedHundredthsFloat((char *)&data[4]) / 2.55;      // The cast is needed on arduino
  float heading = IMURegisters::decodeProtocolUnsignedHundredthsFloat((char *)&data[6]) / 2.55; // The cast is needed on arduino

  auto h = (Heading){.yaw = yaw * 360, .pitch = pitch * 360, .roll = roll * 360, .heading = heading * 360};
  // /* Display orientation values */
  // Serial.print("yaw: ");
  // Serial.print(h.yaw);
  // Serial.print("  pitch:  ");
  // Serial.print(h.pitch);
  // Serial.print("  roll:  ");
  // Serial.print(h.roll);
  // Serial.print("  heading:  ");
  // Serial.print(h.heading);
  // Serial.println("");

  return h;
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

  // Serial.println("Battery voltage: " + String(CrcLib::GetBatteryVoltage()));
  // if (CrcLib::GetBatteryVoltage() < battery_voltage_limit) {
  //   battery_voltage_limit = 15.0f;
  //   Serial.println("Battery voltage: " + String(CrcLib::GetBatteryVoltage()));
  //   CrcLib::SetPwmOutput(pin_BL, 0);
  //   CrcLib::SetPwmOutput(pin_BR, 0);
  //   CrcLib::SetPwmOutput(pin_FL, 0);
  //   CrcLib::SetPwmOutput(pin_FR, 0);
  //   return;
  // }

  if (CrcLib::IsCommValid())
  {
#define RAC(channel) CrcLib::ReadAnalogChannel(channel)
    int8_t joy_stick_state_left_X = clean_joystick_input(RAC(ANALOG::JOYSTICK1_X));
    int8_t joy_stick_state_left_Y = clean_joystick_input(RAC(ANALOG::JOYSTICK1_Y));
    int8_t joy_stick_state_right_X = clean_joystick_input(RAC(ANALOG::JOYSTICK2_X));

    // Get current heading from gyro
    Heading h = gyro_thingy();

    // Convert joystick inputs to field-centric
    FieldCentricInput robotCentric = convertToRobotCentric(
        joy_stick_state_left_Y,   // Forward
        -joy_stick_state_left_X,  // Strafe
        -joy_stick_state_right_X, // Rotation
        h.yaw * 2
    );

    // Apply converted values to motors
    CrcLib::MoveHolonomic(
        robotCentric.forward,
        robotCentric.rotation,
        robotCentric.strafe,
        pin_FL, pin_BL, pin_FR, pin_BR);
  }
}
