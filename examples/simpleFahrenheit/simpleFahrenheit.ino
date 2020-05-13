//
// Teensy 4.x/3.x/LC simple internal temperature
//

#include <InternalTemperature.h>

void setup()
{
  InternalTemperature.begin(TEMPERATURE_NO_ADC_SETTING_CHANGES);

  Serial.begin(115200);
  while (!Serial);
}

void loop()
{
  Serial.print("Temperature: ");
  Serial.print(InternalTemperature.readTemperatureF(), 1);
  Serial.println("°F");
  delay(10000);
}
