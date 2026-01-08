#include <Arduino.h>
// #include <CrcLib.h>
#include <Encoder.h>

auto e = Encoder(48, 50);

uint8_t encoder = 46;
// unsigned long pulseWidth = 0;

void setup()
{
    // CrcLib::Initialize();
    // pinMode(encoder, INPUT);

    Serial.begin(115200);
    pinMode(53, OUTPUT);
    digitalWrite(53, HIGH);
    // pinMode(51, INPUT);
}

void loop()
{

    auto mmin = 3.9, mmax = 1100.0;



    // CrcLib::Update();
    auto e_val = pulseIn(encoder, HIGH, 2000);
    if (e_val > 0) {
        auto n_e_val = constrain(e_val, mmin, mmax);
        auto perc = (n_e_val-mmin)/(mmax-mmin);
        auto angle = perc*360;
        
        Serial.println("current: " + String(angle));
    }
}


