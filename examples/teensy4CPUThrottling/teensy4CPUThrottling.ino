//
// Teensy 4 CPU Throttling at high temperature
//

// This example only works for Teensy 4 - it will not compile for Teensy 3.
// It sets the clock to 600 MHz until the CPU temperature gets above 45C,
// then sets the clock to 150 MHz until it cools to 40C, and repeats.

#include <InternalTemperature.h>

extern "C" uint32_t set_arm_clock(uint32_t frequency); // required prototype

#define LOW_TEMP 40
#define HIGH_TEMP 45

void setup()
{
  Serial.begin(115200);
  while (!Serial);

  // set an alarm at high temperature
  InternalTemperature.attachHighTempInterruptCelsius (HIGH_TEMP, &HighAlarmISR);

  Serial.println("Starting at 600 MHz");
}

void loop()
{
  Serial.print("Temperature: ");
  Serial.print(InternalTemperature.readTemperatureC(), 1);
  Serial.println("°C");
  delay(1000);
}

void HighAlarmISR (void) {
  Serial.println(" (Overtemp ISR - switching to 150 MHz) ");
  set_arm_clock (150000000);

  // set an alarm at low temperature
  InternalTemperature.attachLowTempInterruptCelsius (LOW_TEMP, &LowAlarmISR);
}

void LowAlarmISR (void) {
  Serial.println(" (Undertemp ISR - switching to 600 MHz) ");
  set_arm_clock (600000000);

  // set an alarm at high temperature
  InternalTemperature.attachHighTempInterruptCelsius (HIGH_TEMP, &HighAlarmISR);
}
