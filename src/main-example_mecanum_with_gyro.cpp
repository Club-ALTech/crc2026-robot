#include <Arduino.h>
#include <CrcLib.h>
#include <Wire.h>
#include "AHRSProtocol.h" // navX-Sensor Register Definition header file

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

void setup()
{
  CrcLib::Initialize();
  CrcLib::InitializePwmOutput(pin_BL, false);
  CrcLib::InitializePwmOutput(pin_BR, true);
  CrcLib::InitializePwmOutput(pin_FL, false);
  CrcLib::InitializePwmOutput(pin_FR, true);
  Serial.begin(115200);
  // CrcLib::Initialize();
  Wire.begin(); // join i2c bus (address optional for master)

  for (size_t i = 0; i < sizeof(data); i++)
  {
    data[i] = 0;
  }
}

float gyro_thingy()
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
  float pitch = IMURegisters::decodeProtocolSignedHundredthsFloat((char *)&data[0]) / 2.55;       // The cast is needed on arduino
  float yaw = IMURegisters::decodeProtocolSignedHundredthsFloat((char *)&data[2]) / 2.55;     // The cast is needed on arduino
  float roll = IMURegisters::decodeProtocolSignedHundredthsFloat((char *)&data[4]) / 2.55;      // The cast is needed on arduino
  float heading = IMURegisters::decodeProtocolUnsignedHundredthsFloat((char *)&data[6]) / 2.55; // The cast is needed on arduino

  /* Display orientation values */
  Serial.print("yaw:  ");
  Serial.print(yaw, 2);
  Serial.print("  pitch:  ");
  Serial.print(pitch, 2);
  Serial.print("  roll:  ");
  Serial.print(roll, 2);
  Serial.print("  heading:  ");
  Serial.print(heading, 2);
  Serial.println("");

  return yaw, pitch, roll, heading;

  // delay(ITERATION_DELAY_MS);
}

int8_t clean_joystick_input(int8_t input) {
  if (abs(input) < 0) {
    return 0;
  }
  auto constrainted = constrain(input, -127,128); 
  return constrainted;
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

  if(millis() % 10 != 0){ 
    return;
  }

  // Serial.println("Battery voltage: " + String(CrcLib::GetBatteryVoltage()));

  if (CrcLib::IsCommValid()) {   
    #define RAC(channel) CrcLib::ReadAnalogChannel(channel)
    int8_t joy_stick_state_left_X = clean_joystick_input(RAC(ANALOG::JOYSTICK1_X));
    int8_t joy_stick_state_left_Y = clean_joystick_input(RAC(ANALOG::JOYSTICK1_Y));
    int8_t joy_stick_state_right_X = clean_joystick_input(RAC(ANALOG::JOYSTICK2_X));
    int8_t joy_stick_state_right_Y = clean_joystick_input(RAC(ANALOG::JOYSTICK2_Y));


    // MoveHolonomic(-joy_stick_state_left_Y, joy_stick_state_right_X, joy_stick_state_left_X, pin_FL, pin_BL, pin_FR, pin_BR);
    CrcLib::MoveHolonomic(joy_stick_state_left_Y, -joy_stick_state_right_X, -joy_stick_state_left_X, pin_FL, pin_BL, pin_FR, pin_BR);

    Serial.print("LX" + String(joy_stick_state_left_X)+ "\t");
    Serial.print("LY" + String(joy_stick_state_left_Y)+ "\t");
    Serial.print("RX" + String (joy_stick_state_right_X)+ "\t");
    Serial.println("YD" + String(joy_stick_state_right_Y));
    Serial.println("=================================");
  }

  // gyro_thingy();

}
