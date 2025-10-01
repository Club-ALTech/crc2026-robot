#include <Arduino.h>

#include <Decodeur.h> //Pour utiliser la librairie Decodeur

Decodeur monDecodeur(&Serial);

// #include <CrcLib>

// int led_pin = 3;
// int speed_controller = 69;
// bool on_state = false;

/*
void setup()
{
  CrcLib::Initialize();
  CrcLib::SetDigitalPinMode(led_pin, OUTPUT);
  InitializePwmOutput(speed_controller);
}

void loop()
{
  CrcLib::Update();
  if (millis() % 1000 == 0)
  {
    on_state = !on_state;
  }

  CrcLib::SetDigitalOutput(led_pin, on_state);
}
  */

/*

  Mega analogWrite() test

  This sketch fades LEDs up and down one at a time on digital pins 2 through 13.

  This sketch was written for the Arduino Mega, and will not work on other boards.

  The circuit:

  - LEDs attached from pins 2 through 13 to ground.

  created 8 Feb 2009

  by Tom Igoe

  This example code is in the public domain.

  https://www.arduino.cc/en/Tutorial/AnalogWriteMega

*/

// These constants won't change. They're used to give names to the pins used:

const int pin = 6;

void setup() {
  pinMode(pin, OUTPUT);
  Serial.begin(115200);
  analogWrite(pin, 0);
}

void loop() {
  monDecodeur.refresh();

  // iterate over the pins:
  // for (int i = 0; i <=255; i++) {
  //   analogWrite(led_pin, i);
    
  //   delay(100);

  //   Serial.println(i);
  // }

  
  
  if (monDecodeur.isAvailable()) // Si du texte a été reçu par le décodeur;
  {
    switch (monDecodeur.getCommand())
    {
    case 'S':
      auto motor_pwm = monDecodeur.getArg(0);
      analogWrite(pin, motor_pwm);
      break;
    default:
      break;
    }
  }
}