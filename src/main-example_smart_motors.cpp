#include <Arduino.h>
#include <CrcLib.h>

uint8_t encoder = CRC_ANA_4;
// unsigned long pulseWidth = 0;

void setup()
{
    CrcLib::Initialize();
    // pinMode(encoder, INPUT);
    Serial.begin(115200);
    // ledcSetup(0, 1000, 10); // Utiliser LEDC pour mesurer le rapport cyclique si nécessaire
    //                         // Ou utiliser une librairie comme "PulsePosition" ou "InputCapture"
}

void loop()
{
    CrcLib::Update();
    auto e_val = pulseIn(encoder, HIGH, 1000);
    Serial.println("current: " + String(e_val));
    
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