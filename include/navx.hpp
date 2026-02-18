#include "AHRSProtocol.h"
#include <Arduino.h>
#include <Wire.h>

#define ITERATION_DELAY_MS 10
#define NAVX_SENSOR_DEVICE_I2C_ADDRESS_7BIT 0x32
#define NUM_BYTES_TO_READ 8

int register_address = NAVX_REG_YAW_L;

namespace NavX
{

  struct Heading
  {
    float yaw;
    float pitch;
    float roll;
    float heading;
  };

  Heading get_gyro_info(byte data[512])
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

    /* Display orientation values */

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
    // Serial.print("forward: " + String(forward) + " strafe: " + String(strafe) + " rotation: " + String(rotation));
    // Serial.println(" forward: " + String(result.forward) + " strafe: " + String(result.strafe) + " rotation: " + String(result.rotation));

    return result;
  }
}