#include "../lib/main.hpp"

#include "BluetoothSerial.h"

// D4:8A:FC:CF:AC:C6
BluetoothSerial SerialBT;

char terminator = '\n';
int *commands;
void setup()
{
  // put your setup code here, to run once:
  Serial.begin(115200);
  SerialBT.begin("ESP32test");
  Serial.println(".....The device is up now and can be connected to.....");
}

void loop()
{
  // put your main code here, to run repeatedly:
  String input;
  if (SerialBT.available())
  {
    input = SerialBT.readStringUntil(terminator);
    input.trim();
    commands = tokenizetoint(input);

    for (int i = 0; i < MAX_COMMANDS; i++)
    {
      Serial.print(commands[i]);
      Serial.println();
    }
    delete[] commands;
  }
  else 
  {
    Serial.println("Waiting for connection....")
  }
}
