#include <Decodeur.h>
#include <CrcLib.h>

Decodeur d(&Serial);

void setup() {
    Serial.begin(9600);
    // while (!Serial)
    // {
    //     /* code */
    // }
    
}

void loop() {
    Serial.println("a");
    d.refresh();
    if (d.isAvailable()) {
        if (d.getCommandString() == "ping") {
            delay(1000);
            Serial.println("pong");
        }
    }
}