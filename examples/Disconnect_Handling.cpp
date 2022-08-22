#include <Arduino.h>
#include <Arduino-CRSF.h>

CRSF crsf;

void setup()
{
  Serial.begin(115200);
  crsf.begin(&Serial1, 115200);

  crsf.onDisconnected([]()
                      {
      Serial.println("Connection Lost!");
      Serial.print("Connected: ");
      Serial.println(crsf.isConnected()); });
}

void loop()
{
}

void serialEvent1()
{
  crsf.readPacket();
}