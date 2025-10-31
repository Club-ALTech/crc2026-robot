#include <Arduino.h>
#include <Wire.h>
#include "AHRSProtocol.h" // navX-Sensor Register Definition header file
#include <CrcLib.h>

byte data[512];

#define ITERATION_DELAY_MS 10
#define NAVX_SENSOR_DEVICE_I2C_ADDRESS_7BIT 0x32
#define NUM_BYTES_TO_READ 8

void setup()
{
  Serial.begin(115200);
  // CrcLib::Initialize();
  Wire.begin(); // join i2c bus (address optional for master)

  for (size_t i = 0; i < sizeof(data); i++)
  {
    data[i] = 0;
  }
}

int register_address = NAVX_REG_YAW_L;

struct Heading
{
  float yaw;
  float pitch;
  float roll;
  float heading;
};


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
  float pitch = IMURegisters::decodeProtocolSignedHundredthsFloat((char *)&data[0]) / 2.55;       // The cast is needed on arduino
  float yaw = IMURegisters::decodeProtocolSignedHundredthsFloat((char *)&data[2]) / 2.55;     // The cast is needed on arduino
  float roll = IMURegisters::decodeProtocolSignedHundredthsFloat((char *)&data[4]) / 2.55;      // The cast is needed on arduino
  float heading = IMURegisters::decodeProtocolUnsignedHundredthsFloat((char *)&data[6]) / 2.55; // The cast is needed on arduino

  Heading h;
  h.yaw = yaw * 100;
  h.pitch = pitch * 100;
  h.roll = roll * 100;
  h.heading = heading * 100;

  /* Display orientation values */
  Serial.print("yaw: ");
  Serial.print(h.yaw);
  Serial.print("  pitch:  ");
  Serial.print(h.pitch);
  Serial.print("  roll:  ");
  Serial.print(h.roll);
  Serial.print("  heading:  ");
  Serial.print(h.heading);
  Serial.println("");

  return h;
}

void loop()
{
  CrcLib::Update();
  gyro_thingy();
}
