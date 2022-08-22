#include <Arduino.h>
#include <Arduino-CRSF.h>

CRSF crsf;

void setup()
{
  Serial.begin(115200);
  crsf.begin(&Serial1, 115200);

  crsf.onDataReceived([](const uint16_t channels[])
                      {
  char buffer[100];
  sprintf(buffer, "CH1: %d\tCH2: %d\tCH3: %d\tCH4: %d\tCH5: %d\tCH6: %d\tCH7: %d\tCH8: %d\n",
         channels[0],
         channels[1],
         channels[2],
         channels[3],
         channels[4],
         channels[5],
         channels[6],
         channels[7]);
  Serial.print(buffer); });
}

void loop()
{
}

void serialEvent1()
{
  crsf.readPacket();
}