//
// Teensy 4.x/3.x/LC simple internal temperature
//

#include <InternalTemperature.h>

void setup()
{
  Serial.begin(115200);
  while (!Serial);
}

void loop()
{
  Serial.print("Temperature: ");
  Serial.print(InternalTemperature.readTemperatureC(), 1);
  Serial.println("°C");
  delay(10000);
}
