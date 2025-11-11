#include <Arduino.h>
#include <CrcLib.h>
#include <Decodeur.h>
#include <Encoder.h>

Encoder enc(CRC_ENCO_A, CRC_ENCO_B);
Decodeur d(&Serial);

void setup() {
  CrcLib::Initialize(); 
  Serial.begin(115200); 
  Serial.println("Basic Encoder Test:");
}

void loop()
{
  CrcLib::Update();
  d.refresh();
  
  Serial.println(enc.read());

  if (d.isAvailable()) // Si du texte a été reçu par le décodeur;
  {
    auto command = d.getCommandString();
    Serial.println("Received command: " + command);
    if (command == "echo" ) {
      Serial.println(d.getArgString(0));
    }
    if (command == "ping") {
      Serial.println("pong"); 
    }
  }

}
