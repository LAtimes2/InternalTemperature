//
// Teensy simple internal temperature
//

#include <InternalTemperature.h>

bool highAlarmTriggered = false;
bool lowAlarmTriggered = false;

const bool ShouldNotTrigger = false;
const bool ShouldTrigger = true;

void setup()
{
  float currentTemperature;

  Serial.begin(115200);
  while (!Serial);

  // wait 1 second for temperature to stabilize a bit
  delay (1000);

  currentTemperature = InternalTemperature.readTemperatureF ();

  testHighAlarm (currentTemperature + 5.0, ShouldNotTrigger);
  testHighAlarm (currentTemperature - 5.0, ShouldTrigger);
  InternalTemperature.detachHighTempInterrupt ();

  testLowAlarm (currentTemperature - 5.0, ShouldNotTrigger);
  testLowAlarm (currentTemperature + 5.0, ShouldTrigger);
  testLowAlarm (currentTemperature - 5.0, ShouldNotTrigger);

  currentTemperature = InternalTemperature.readTemperatureF ();

  // Low Alarm is still set
  testHighAlarm (currentTemperature + 5.0, ShouldNotTrigger);
  testHighAlarm (currentTemperature - 2.0, ShouldTrigger);

  testHighAlarm (currentTemperature + 5.0, ShouldNotTrigger);

  // High Alarm is still set
  testLowAlarm (currentTemperature - 5.0, ShouldNotTrigger);
  testLowAlarm (currentTemperature + 5.0, ShouldTrigger);
}

void loop()
{
  Serial.print("Temperature: ");
  Serial.print(InternalTemperature.readTemperatureF(), 1);
  Serial.println("°F");
  delay(1000);
}

void HighAlarmISR (void) {
  highAlarmTriggered = true;
  Serial.print(" (Overtemp ISR) ");
}

void LowAlarmISR (void) {
  lowAlarmTriggered = true;
  Serial.print(" (Undertemp ISR) ");
}

void testHighAlarm (float temperature, bool alarmExpected) {

  Serial.print ("Setting High Temp Alarm to ");
  Serial.print (temperature);
  Serial.print (" degrees - ");
  highAlarmTriggered = false;
  InternalTemperature.attachHighTempInterruptFahrenheit (temperature, &HighAlarmISR);

  // allow time for conversions
  delay (100);

  if (alarmExpected) {
    if (highAlarmTriggered) {
      Serial.println("triggered as expected");
    }
    else {
      Serial.println("did not trigger - FAILED");
    }
  }
  else {  // not expected
    if (highAlarmTriggered) {
      Serial.println("triggered - FAILED");
    }
    else {
      Serial.println("did not trigger, as expected");
    }
  }
}

void testLowAlarm (float temperature, bool alarmExpected) {

  Serial.print ("Setting Low Temp Alarm to ");
  Serial.print (temperature);
  Serial.print (" degrees - ");
  lowAlarmTriggered = false;
  InternalTemperature.attachLowTempInterruptFahrenheit (temperature, &LowAlarmISR);

  delay (500);

  if (alarmExpected) {
    if (lowAlarmTriggered) {
      Serial.println("triggered as expected");
    }
    else {
      Serial.println("did not trigger - FAILED");
    }
  }
  else {  // not expected
    if (lowAlarmTriggered) {
      Serial.println("triggered - FAILED");
    }
    else {
      Serial.println("did not trigger, as expected");
    }
  }
}
