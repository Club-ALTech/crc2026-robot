#include <Arduino.h>
#include <CrcLib.h>
#include <Decodeur.h>
#include <Encoder.h>
#include <PID_v1.h>

Encoder enc(CRC_ENCO_A, CRC_ENCO_B);
Decodeur d(&Serial);
int pin = CRC_PWM_1;
float tour = 535.857142857;

double Setpoint, // rpm expected
    Input,       //  rpm actual
    Output;      // voltage

// Specify the links and initial tuning parameters
double Kp = 30, Ki = 20, Kd = 0;
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

static auto target_rps = 4;

void setup()
{
  CrcLib::Initialize();
  Serial.begin(115200);
  Serial.println("Basic Encoder Test:");
  CrcLib::InitializePwmOutput(pin);
  Setpoint = target_rps; // rpm expected  
  
  //turn the PID on
  myPID.SetOutputLimits(-128, 127);
  myPID.SetMode(AUTOMATIC);
}

/**
 * returns rps
 */
float speed(int64_t time, int ticks)
{
  float nb_tours_fait = ticks / tour;
  // Serial.print(ticks);
  return nb_tours_fait / (float(time) / 1000);
}


long last_time = 0, delay_ms = 1000 / 50;
long delta_time = 0;

bool timer(long now){
  delta_time = (now - last_time);
  if(delta_time >= delay_ms){
    last_time = now;
    return true;
  }
  return false;
}

void loop()
{
  CrcLib::Update();
  myPID.Compute();
  d.refresh();


  if (d.isAvailable()) {
    if (d.getCommand() == 's')
    {
     Setpoint = d.getArg(0);
    }
  }

  if (timer(millis()))
  {

    static int32_t last = 0;
    auto current = enc.read();
    auto delta_ticks = last - current;
    auto current_rpm = speed(delta_time, delta_ticks);
    last = current;

    Input = current_rpm, //  rpm actual
    Serial.println("rpm: " + String(current_rpm) + "\toutput: " + String(Output)+ "\terr: " + String(Setpoint-Input));
    CrcLib::SetPwmOutput(pin, Output);
  }

  // myPID.Compute();

  // analogWrite(PIN_OUTPUT, Output);
}
