#include <Arduino.h>
#include <CrcLib.h>
#include "motors.hpp"
#include "encoders.hpp"
#include "smart_joint.hpp"

RevAbsEncoder encodeur(CRC_DIG_1, 2000);
Motor moteur(CRC_PWM_1);
// unsigned long pulseWidth = 0;

SmartJoint joint(moteur, encodeur);


void setup()
{
    CrcLib::Initialize();
    // pinMode(encoder, INPUT);
    Serial.begin(115200);
    // ledcSetup(0, 1000, 10); // Utiliser LEDC pour mesurer le rapport cyclique si nécessaire
    //                         // Ou utiliser une librairie comme "PulsePosition" ou "InputCapture"

    moteur.setup();
    joint.begin(0.0001, 0.0, 0.0005);
}

void loop()
{
    CrcLib::Update();

    Serial.println("current: " + String(encodeur._read_raw()) + " // angle: " + String(encodeur.read_angle()));
    
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

    joint.set_target_angle(180);
    joint.update();

    // moteur.set_power(0.5);
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