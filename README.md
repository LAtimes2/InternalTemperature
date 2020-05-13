# InternalTemperature

---
The Kinetis Cortex-M processor on the Teensy 3/LC boards has a built-in temperature sensor. This library provides functions to read the temperature in both Celsius and Fahrenheit.

The Teensy 4 has a built-in temperature sensor, along with built-in functions to read temperature. This library provides a wrapper around the built-in functions (to support common code for Teensy 3 and 4).

In addition, the library has high and low temperature alarm functions to perform actions when temperature limits are exceeded.

Examples are provided to show how to read temperatures, set alarms, and, for Teensy 4, change the CPU clock speed based on temperature.

Here is a simple example of how to read the temperature:
```c++
#include <InternalTemperature.h>

void setup()
{
}

void loop()
{
  float temp = InternalTemperature.readTemperatureC();
}
```

For more details and information on calibration, see

https://github.com/LAtimes2/InternalTemperature/blob/master/InternalTemperature.pdf 
