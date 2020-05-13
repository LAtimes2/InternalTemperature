//
// Teensy 3.x/LC single point calibration example
//

#include <InternalTemperature.h>

boolean celsius = true;

void setup()
{
  float currentTemperature;

  Serial.begin(115200);
  while (!Serial);

  Serial.print("Teensy unique id : ");
  Serial.println(InternalTemperature.getUniqueID(), HEX);

  Serial.print("Enter 1 for Celsius or 2 for Fahrenheit : ");
  while (!Serial.available());

  if (Serial.parseInt() == 2) {
    celsius = false;
  }

  Serial.println("");

  Serial.print("Enter current temperature : ");
  Serial.clear();
  while (!Serial.available());

  currentTemperature = Serial.parseFloat();

  if (celsius) {
    if (!InternalTemperature.singlePointCalibrationC(currentTemperature, InternalTemperature.readTemperatureC())) {
      Serial.println(" ERROR - invalid calibration temperature");
    }
  } else {
    if (!InternalTemperature.singlePointCalibrationF(currentTemperature, InternalTemperature.readTemperatureF())) {
      Serial.println(" ERROR - invalid calibration temperature");
    }
  }

  Serial.println("");
  Serial.println("");

  Serial.println("To make change permanent in a sketch for this Teensy, add these lines after call to temperature.begin:");
  Serial.println("");

  Serial.print("  if (temperature.getUniqueID() == 0x");
  Serial.print(InternalTemperature.getUniqueID(), HEX);
  Serial.println(") {");

  Serial.print("    temperature.setSlope(");
  Serial.print(InternalTemperature.getSlope(), 6);
  Serial.println(");");

  Serial.print("    temperature.setVTemp25(");
  Serial.print(InternalTemperature.getVTemp25(), 4);
  Serial.println(");");

  Serial.println("  }");
  Serial.println("");
}

void loop()
{
  Serial.print("Calibrated Temperature: ");
  if (celsius) {
    Serial.print(InternalTemperature.readTemperatureC(), 1);
    Serial.print("°C");
  } else {
    Serial.print(InternalTemperature.readTemperatureF(), 1);
    Serial.print("°F");
  }
  Serial.print(", Uncalibrated Temperature: ");
  if (celsius) {
    Serial.print(InternalTemperature.readUncalibratedTemperatureC(), 1);
    Serial.print("°C");
  } else {
    Serial.print(InternalTemperature.readUncalibratedTemperatureF(), 1);
    Serial.print("°F");
  }
  Serial.println("");
  delay(10000);
}
