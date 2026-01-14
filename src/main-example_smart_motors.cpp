#include <Arduino.h>
#include <CrcLib.h>
#include "motors.hpp"
#include "encoders.hpp"

Encoder encodeur(CRC_DIG_1, 2000);
Motor moteur(CRC_PWM_1);
// unsigned long pulseWidth = 0;

void setup()
{
    CrcLib::Initialize();
    // pinMode(encoder, INPUT);
    Serial.begin(115200);
    // ledcSetup(0, 1000, 10); // Utiliser LEDC pour mesurer le rapport cyclique si nécessaire
    //                         // Ou utiliser une librairie comme "PulsePosition" ou "InputCapture"

    moteur.setup();
}

void loop()
{
    CrcLib::Update();

    Serial.println("current: " + encodeur.getCurrentValue());
    
    // Mesure approximative (non fiable pour 975 Hz)
    // auto pulseWidth = pulseIn(encoder, HIGH, 2000); // Timeout 2ms

    
    // if (pulseWidth > 0)
    // {
    //     Serial.println("current: " + String(pulseWidth));
    // }
    // {
    //     float angle = (pulseWidth - 1.0) / 1023.0 * 360.0; // Convertir en degrés
    //     Serial.print("Angle: ");
    //     Serial.println(angle);
    // }


    moteur.set_power(0.5);
}




// const int encoderPin = 4;
// unsigned long pulseWidth = 0;

// void setup()
// {
//     Serial.begin(115200);
//     ledcSetup(0, 1000, 10); // Utiliser LEDC pour mesurer le rapport cyclique si nécessaire
//                             // Ou utiliser une librairie comme "PulsePosition" ou "InputCapture"
// }

// void loop()
// {
//     // Mesure approximative (non fiable pour 975 Hz)
//     pulseWidth = pulseIn(encoderPin, HIGH, 2000); // Timeout 2ms

//     if (pulseWidth > 0)
//     {
//         float angle = (pulseWidth - 1.0) / 1023.0 * 360.0; // Convertir en degrés
//         Serial.print("Angle: ");
//         Serial.println(angle);
//     }
//     delay(100);
// }