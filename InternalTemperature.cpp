/* InternalTemperature - read internal temperature of ARM processor
 * Copyright (C) 2021 LAtimes2
 *
 * MIT License
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission
 * notice shall be included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#include <stdint.h>

#include "InternalTemperature.h"
#include <Arduino.h>

// common static variables
InternalTemperatureClass::alarmType InternalTemperatureClass::alarm = InternalTemperatureClass::NoAlarm;
InternalTemperatureClass::voidFuncPtr InternalTemperatureClass::highTempISR = NULL;
InternalTemperatureClass::voidFuncPtr InternalTemperatureClass::lowTempISR = NULL;

// singleton instance of the class
InternalTemperatureClass InternalTemperature;

// Teensy 4.0 is at the end
#if not defined(__IMXRT1062__)

// Teensy 3.0,3.1,3.2
#if defined(__MK20DX128__) || defined(__MK20DX256__)  
  #define DEFAULT_VTEMP25 0.719    // volts
  #define DEFAULT_SLOPE   0.00172  // volts/degrees C
#else
  #define DEFAULT_VTEMP25 0.716    // volts
  #define DEFAULT_SLOPE   0.00162  // volts/degrees C
#endif

#if defined(__MK64FX512__) || defined(__MK66FX1M0__)   // 3.5, 3.6
  #define TEMPERATURE_CHANNEL 26
  #define TEMPERATURE_PIN 70
  #define VREF_PIN 71
#else
  #define TEMPERATURE_CHANNEL 26
  #define TEMPERATURE_PIN 38
  #define VREF_PIN 39
#endif

#define MIN_COUNT 0
#define MAX_COUNT 0xFFFF

// static variables
bool InternalTemperatureClass::initialized = false;
int InternalTemperatureClass::temperatureSettingsType = TEMPERATURE_MAX_ACCURACY;
float InternalTemperatureClass::slope = DEFAULT_SLOPE;
float InternalTemperatureClass::vTemp25 = DEFAULT_VTEMP25;
float InternalTemperatureClass::voltsPerBit = 1.0;

// Constructor
InternalTemperatureClass::InternalTemperatureClass()
{
}

bool InternalTemperatureClass::begin (int temperature_settings_type) {

  // TeensyLC needs the Bandgap voltage for measurements. All Teensy's need
  // the Bandgap turned on in low power mode.
  // So always turn it on. If it is already on, it doesn't hurt anything.
  enableBandgap ();

  temperatureSettingsType = temperature_settings_type;

  if (temperature_settings_type == TEMPERATURE_MAX_ACCURACY)
  {
    // set to internal 1.2V reference
    analogReference(INTERNAL);

    // set to maximum number of bits
    analogReadResolution(16);

    // set to maximum averaging
    analogReadAveraging(32);
  }

  voltsPerBit = computeVoltsPerBit ();

  initialized = true;

  return true;
}

void InternalTemperatureClass::enableBandgap (void) {

  // Bandgap enable
  PMC_REGSC |= PMC_REGSC_BGEN | PMC_REGSC_BGBE;

  // Delay for regulator to start up
  delay (2);
}

float InternalTemperatureClass::readRawTemperatureVoltage () {
  return readRawVoltage (TEMPERATURE_PIN);
}

float InternalTemperatureClass::readRawVoltage (int signalNumber) {

  int analogValue;
  float volts;

  if (!initialized) {
    begin ();
  }

  // save previous values because this routine or readAnalog modify them
  int previousADC0_CFG1 = ADC0_CFG1;
  int previousADC0_CFG2 = ADC0_CFG2;
  int previousADC0_SC1A = ADC0_SC1A;
  int previousADC0_SC2  = ADC0_SC2;
  int previousADC0_SC3  = ADC0_SC3;

  //
  // set to max sample time (for best accuracy). This is
  // especially needed when using external Vref.
  //

  // enable long sample times
  ADC0_CFG1 |= ADC_CFG1_ADLSMP;

  // 00 = 32 clocks
  ADC0_CFG2 &= ~ADC_CFG2_ADLSTS(3);

  // disable compare (prevents COCO)
  ADC0_SC2 &= ~ADC_SC2_ACFE;  

  //
  // read temperature
  //
  analogValue = analogRead(signalNumber);

/*
  Serial.print(" 0x");
  Serial.print(analogValue, HEX);
  Serial.print(" ");

  Serial.print("SC1A: 0x");
  Serial.print(ADC0_SC1A, HEX);
  Serial.print(" ");

  Serial.print("SC2: 0x");
  Serial.print(ADC0_SC2, HEX);
  Serial.print(" ");
*/
  // convert analog temperature reading to volts
  volts = analogValue * computeVoltsPerBit ();

/*
  Serial.print(", ");
  Serial.print(volts, 3);
  Serial.print(" V ");
*/
  // restore previous values
  ADC0_CFG1 = previousADC0_CFG1;
  ADC0_CFG2 = previousADC0_CFG2;
  ADC0_SC1A = previousADC0_SC1A;
  ADC0_SC2  = previousADC0_SC2;
  ADC0_SC3  = previousADC0_SC3;
  ADC0_SC1A = previousADC0_SC1A;

/*
  Serial.print("SC1A: 0x");
  Serial.print(previousADC0_SC1A, HEX);
  Serial.print(" ");

  Serial.print("SC2: 0x");
  Serial.print(ADC0_SC2, HEX);
  Serial.print(" ");

  Serial.print("SC3: 0x");
  Serial.print(ADC0_SC3, HEX);
  Serial.print(" ");

delay (5);

  Serial.print("SC1A: 0x");
  Serial.print(ADC0_SC1A, HEX);
  Serial.print(" ");
*/

  return volts;
}

float InternalTemperatureClass::computeVoltsPerBit () {

#if defined(__MKL26Z64__)   // Teensy LC 
  // uses Bandgap voltage since it doesn't have a reference voltage
  const float vRef = 1.0;
#else
  const float vRef = 1.195;
#endif

  int analogVref;
  float floatVref;
  float voltsPerBit;

  int previousADC0_CFG1 = ADC0_CFG1;
  int previousADC0_CFG2 = ADC0_CFG2;
  int previousADC0_SC1A = ADC0_SC1A;
  int previousADC0_SC2  = ADC0_SC2;
  int previousADC0_SC3  = ADC0_SC3;

  // disable compare (prevents COCO)
  ADC0_SC2 &= ~ADC_SC2_ACFE;  

  // read Vref
  analogVref = analogRead(VREF_PIN);

  // restore previous values
  ADC0_CFG1 = previousADC0_CFG1;
  ADC0_CFG2 = previousADC0_CFG2;
  ADC0_SC1A = previousADC0_SC1A;
  ADC0_SC2  = previousADC0_SC2;
  ADC0_SC3  = previousADC0_SC3;
  ADC0_SC1A = previousADC0_SC1A;

/*
  Serial.print(" 0x");
  Serial.print(analogVref, HEX);
  Serial.print(" ");
*/
  
  floatVref = analogVref;

  // compute scaling for Least Significant Bit
  voltsPerBit = vRef / floatVref;

  return voltsPerBit;
}

float InternalTemperatureClass::convertTemperatureC (float volts, float vTemp25, float slope) {

  float temperatureCelsius;

  // convert voltage to temperature using equation from CPU Reference Manual
  temperatureCelsius = 25 - ((volts - vTemp25) / slope);
  
  return temperatureCelsius;
}

float InternalTemperatureClass::convertTemperatureC (float volts) {

  return convertTemperatureC (volts, vTemp25, slope);
}

float InternalTemperatureClass::convertUncalibratedTemperatureC (float volts) {
  return convertTemperatureC (volts, DEFAULT_VTEMP25, DEFAULT_SLOPE);
}

float InternalTemperatureClass::readTemperatureC () {
  return convertTemperatureC (readRawTemperatureVoltage ());
}

float InternalTemperatureClass::readTemperatureF () {
  // convert celsius to fahrenheit
  return toFahrenheit(readTemperatureC());
}

float InternalTemperatureClass::readUncalibratedTemperatureC () {
  return convertUncalibratedTemperatureC (readRawTemperatureVoltage ());
}

float InternalTemperatureClass::readUncalibratedTemperatureF () {
  // convert celsius to fahrenheit
  return toFahrenheit(readUncalibratedTemperatureC());
}

//
//  Temperature Alarm functions
//

int InternalTemperatureClass::attachInterruptCelsius (float triggerTemperature, temperatureAlarmType whichTemperature) {

  int returnValue = 0;
  uint32_t temperatureCount;

  // if don't change settings, do nothing and return error code
  if (temperatureSettingsType == TEMPERATURE_NO_ADC_SETTING_CHANGES) {

    alarm = NoAlarm;

    returnValue = -1;

  } else {

    if (!initialized) {
      begin ();
    }

    temperatureCount = celsiusToCount (triggerTemperature);

    // limit to max value
    if (temperatureCount > MAX_COUNT) {
      temperatureCount = MAX_COUNT;
    }

    // disable interrupts and compare while setting up
    ADC0_SC1A &= ~ADC_SC1_AIEN;
    ADC0_SC2 &= ~ADC_SC2_ACFE;
  
    // 3 cases for compare:
    //  1. only high - use CV1, less than
    //  2. only low - use CV1, greater than
    //  3. both - high in CV1, low in CV2, less than (outside range)

    if (alarm == HighTempAlarm) {
      ADC0_CV1 = temperatureCount;
      ADC0_SC2 &= ~ADC_SC2_ACFGT;  // less than
      ADC0_SC2 &= ~ADC_SC2_ACREN;  // range disable
    }
    else if (alarm == LowTempAlarm) {
      ADC0_CV1 = temperatureCount;
      ADC0_CV2 = temperatureCount; // set second in case a range is used
      ADC0_SC2 |= ADC_SC2_ACFGT;   // greater than
      ADC0_SC2 &= ~ADC_SC2_ACREN;  // range disable
    }
    else if (alarm == BothAlarms) {
      if (whichTemperature == LowTemperature) {
        ADC0_CV2 = temperatureCount;
      }
      else {
        ADC0_CV1 = temperatureCount;        
      }
      ADC0_SC2 &= ~ADC_SC2_ACFGT;  // less than
      ADC0_SC2 |= ADC_SC2_ACREN;   // range enable
    }

    // enable continuous conversions
    ADC0_SC3 |= ADC_SC3_ADCO;
    
    attachInterruptVector(IRQ_ADC0, &InternalTemperatureClass::alarmISR);

    // read register to clear COCO
    (void) ADC0_RA;

    // enable compare
    ADC0_SC2 |= ADC_SC2_ACFE;

    // enable interrupts
    ADC0_SC1A |= ADC_SC1_AIEN;
  
    // clear channel, then set channel to temperature sensor.
    // This also initiates a conversion to start the continuous conversions
    ADC0_SC1A &= ~ADC_SC1_ADCH(0xFF);
    ADC0_SC1A |= ADC_SC1_ADCH(TEMPERATURE_CHANNEL);

/*
  Serial.println("");
  Serial.print("SC1A: 0x");
  Serial.print(ADC0_SC1A, HEX);
  Serial.print(" ");

  Serial.print("SC2: 0x");
  Serial.print(ADC0_SC2, HEX);
  Serial.print(" ");

  Serial.print("SC3: 0x");
  Serial.print(ADC0_SC3, HEX);
  Serial.println(" ");
  Serial.print(" ");

  Serial.print("CV1: 0x");
  Serial.print(ADC0_CV1, HEX);
  Serial.print(", ");
  Serial.print("CV2: 0x");
  Serial.print(ADC0_CV2, HEX);
  Serial.println(" ");
*/
    NVIC_CLEAR_PENDING(IRQ_ADC0);
    NVIC_ENABLE_IRQ(IRQ_ADC0);
  }

  return returnValue;
}

int InternalTemperatureClass::detachHighTempInterrupt () {

  int returnValue = 0;

  // if don't change settings, do nothing and return error code
  if (temperatureSettingsType == TEMPERATURE_NO_ADC_SETTING_CHANGES) {

    returnValue = -1;

  } else {

    // if High Temp is only alarm
    if (alarm == HighTempAlarm) {
      alarm = NoAlarm;
      ADC0_CV1 = MIN_COUNT;   // prevent more interrupts
      NVIC_DISABLE_IRQ(IRQ_ADC0);
    }
    else if (alarm == BothAlarms) {
      alarm = LowTempAlarm;
      ADC0_CV1 = MIN_COUNT;   // prevent more interrupts
    }

  }

  return returnValue;
}

int InternalTemperatureClass::detachLowTempInterrupt () {

  int returnValue = 0;

  // if don't change settings, do nothing and return error code
  if (temperatureSettingsType == TEMPERATURE_NO_ADC_SETTING_CHANGES) {

    returnValue = -1;

  } else {

    // if Low Temp is only alarm
    if (alarm == LowTempAlarm) {
      alarm = NoAlarm;
      ADC0_CV1 = MAX_COUNT;   // prevent more interrupts
      NVIC_DISABLE_IRQ(IRQ_ADC0);
    }
    else if (alarm == BothAlarms) {
      alarm = HighTempAlarm;
      ADC0_SC2 &= ~ADC_SC2_ACREN;   // range disable
    }

  }

  return returnValue;
}

// convert celsius temperature to a raw count value
uint32_t InternalTemperatureClass::celsiusToCount (float temperatureCelsius) {

  uint32_t rawCount;
  float volts;

  // convert temperature to voltage using equation from CPU Reference Manual solved for volts
  volts = -((temperatureCelsius - 25) * slope - vTemp25);

  rawCount = volts / voltsPerBit;

  return rawCount;
}

void InternalTemperatureClass::getRegisterCounts (int *currentCount, int *highTempCount, int *lowTempCount) {
  *currentCount = (int)ADC0_RA;
  *highTempCount = (int)ADC0_CV1;
  *lowTempCount = (int)ADC0_CV2;
}

//
//  Calibration functions
//

bool InternalTemperatureClass::singlePointCalibrationC (
  float actualTemperatureC, float measuredTemperatureC, bool fromDefault) {

  float theSlope = slope;
  float theVTemp25 = vTemp25;

  if (fromDefault) {
    theSlope = DEFAULT_SLOPE;
    theVTemp25 = DEFAULT_VTEMP25;
  }

  // adjust vTemp25 for the delta temperature
  float deltaTemperature = measuredTemperatureC - actualTemperatureC;

  float deltaVolts = deltaTemperature * theSlope;

  return setVTemp25 (theVTemp25 - deltaVolts);
}

bool InternalTemperatureClass::singlePointCalibrationF (
  float actualTemperatureF, float measuredTemperatureF, bool fromDefault) {

  return singlePointCalibrationC (toCelsius(actualTemperatureF), toCelsius(measuredTemperatureF), fromDefault);
}

bool InternalTemperatureClass::dualPointCalibrationC (
  float actualTemperature1C, float measuredTemperature1C,
  float actualTemperature2C, float measuredTemperature2C, bool fromDefault) {

  float deltaActual = actualTemperature2C - actualTemperature1C;
  float deltaMeasured = measuredTemperature2C - measuredTemperature1C;
  float newSlope;
  bool returnValue = false;

  float originalSlope = slope;
  float originalVTemp25 = vTemp25;

  if (fromDefault) {
    originalSlope = DEFAULT_SLOPE;
    originalVTemp25 = DEFAULT_VTEMP25;
  }

  // adjust slope first, then the offset
  newSlope = originalSlope * deltaMeasured / deltaActual;

  if (setSlope (newSlope)) {

    // offset at 25 degrees C

    // Original: measured voltage = originalVTemp25 - (measuredTemperature1C - 25) * originalSlope
    // New     : measured voltage = newVTemp25      - (actualTemperature1C   - 25) * newSlope
    //
    // Since measured voltage is the same:
    // newVTemp25 - (actualTemperature1C - 25) * newSlope = originalVTemp25 - (measuredTemperature1C - 25) * originalSlope
    //
    // Rearranging:
    // newVTemp25 = originalVTemp25 - (measuredTemperature1C - 25) * originalSlope + (actualTemperature1C - 25) * newSlope

    float newVTemp25 = originalVTemp25 - (measuredTemperature1C - 25) * originalSlope + (actualTemperature1C - 25) * newSlope;

    returnValue = setVTemp25 (newVTemp25);
  }

  return returnValue;
}

bool InternalTemperatureClass::dualPointCalibrationF (
  float actualTemperature1F, float measuredTemperature1F,
  float actualTemperature2F, float measuredTemperature2F, bool fromDefault) {

  return dualPointCalibrationC (toCelsius(actualTemperature1F), toCelsius(measuredTemperature1F),
                                toCelsius(actualTemperature2F), toCelsius(measuredTemperature2F), fromDefault);
}


bool InternalTemperatureClass::setVTemp25 (float volts)
{
  // perform a range check (0-5 volts)
  if (volts < 0.0 || volts > 5.0) {
    return false;
  }
  vTemp25 = volts;
  return true;
}

bool InternalTemperatureClass::setSlope (float voltsPerDegreeC)
{
  // perform a range check (factor of 10 around default value)
  if (voltsPerDegreeC < (DEFAULT_SLOPE / 10.0) || voltsPerDegreeC > (DEFAULT_SLOPE * 10.0)) {
    return false;
  }
  slope = voltsPerDegreeC;
  return true;
}

float InternalTemperatureClass::getVTemp25 () {
  return vTemp25;
}

float InternalTemperatureClass::getSlope () {
  return slope;
}

// Unique ID can be used to set calibration values by serial number
int InternalTemperatureClass::getUniqueID () {
  return SIM_UIDL;
}

#else   // Teensy 4

//
// Teensy 4.0 uses tempMon, so this is just a thin wrapper around it for backward compatibility
//

#define MIN_COUNT 0
#define MAX_COUNT 0xFFF

// Constructor
InternalTemperatureClass::InternalTemperatureClass()
{
}

bool InternalTemperatureClass::begin (int temperature_settings_type) {
   return true;
}

float InternalTemperatureClass::readTemperatureC () {
  return tempmonGetTemp();
}

float InternalTemperatureClass::readTemperatureF () {
  // convert celsius to fahrenheit
  return toFahrenheit(readTemperatureC());
}

float InternalTemperatureClass::readUncalibratedTemperatureC () {
  // calibration is internal
  return readTemperatureC();
}

float InternalTemperatureClass::readUncalibratedTemperatureF () {
  // calibration is internal
  return readTemperatureF();
}

int InternalTemperatureClass::attachInterruptCelsius (float triggerTemperature, temperatureAlarmType whichTemperature) {

  uint32_t tempCodeVal = celsiusToCount (triggerTemperature);

  // limit to max value
  if (tempCodeVal > MAX_COUNT) {
    tempCodeVal = MAX_COUNT;
  }

  // stop while updating to prevent interrupts
  tempmon_Stop ();
  
  if (whichTemperature == HighTemperature) {
    // clear previous value
    TEMPMON_TEMPSENSE0_CLR = TEMPMON_CTRL0_ALARM_VALUE (MAX_COUNT);

    // set new value
    TEMPMON_TEMPSENSE0_SET = TEMPMON_CTRL0_ALARM_VALUE (tempCodeVal);
  }
  else {  // LowTemperature
    // clear previous value
    TEMPMON_TEMPSENSE2_CLR = TEMPMON_CTRL2_LOW_ALARM_VALUE (MAX_COUNT);

    // set new value
    TEMPMON_TEMPSENSE2_SET = TEMPMON_CTRL2_LOW_ALARM_VALUE (tempCodeVal);    
  }

  attachInterruptVector(IRQ_TEMPERATURE, &InternalTemperatureClass::alarmISR);

  NVIC_CLEAR_PENDING(IRQ_TEMPERATURE);
  NVIC_ENABLE_IRQ(IRQ_TEMPERATURE);

  // start converting again
  tempmon_Start ();

  return 0;
}

int InternalTemperatureClass::detachHighTempInterrupt () {

  // if High Temp is only alarm
  if (alarm == HighTempAlarm) {
    alarm = NoAlarm;
    TEMPMON_TEMPSENSE0_CLR = TEMPMON_CTRL0_ALARM_VALUE (MAX_COUNT);   // prevent more interrupts
    NVIC_DISABLE_IRQ(IRQ_TEMPERATURE);
  }
  else if (alarm == BothAlarms) {
    alarm = LowTempAlarm;
    TEMPMON_TEMPSENSE0_CLR = TEMPMON_CTRL0_ALARM_VALUE (MAX_COUNT);   // prevent more interrupts
  }

  return 0;
}

int InternalTemperatureClass::detachLowTempInterrupt () {

  // if Low Temp is only alarm
  if (alarm == LowTempAlarm) {
    alarm = NoAlarm;
    TEMPMON_TEMPSENSE2_SET = TEMPMON_CTRL2_LOW_ALARM_VALUE (MAX_COUNT);   // prevent more interrupts
    NVIC_DISABLE_IRQ(IRQ_TEMPERATURE);
  }
  else if (alarm == BothAlarms) {
    alarm = HighTempAlarm;
    TEMPMON_TEMPSENSE2_SET = TEMPMON_CTRL2_LOW_ALARM_VALUE (MAX_COUNT);   // prevent more interrupts
  }

  return 0;
}

// convert celsius temperature to a raw count value
uint32_t InternalTemperatureClass::celsiusToCount (float temperatureCelsius) {

  uint32_t tempCodeVal;

  // read calibration data
  uint32_t calibrationData = HW_OCOTP_ANA1;
  uint32_t s_hotTemp = (uint32_t)(calibrationData & 0xFFU) >> 0x00U;
  uint32_t s_hotCount = (uint32_t)(calibrationData & 0xFFF00U) >> 0X08U;
  uint32_t roomCount = (uint32_t)(calibrationData & 0xFFF00000U) >> 0x14U;
  float    s_hot_ROOM = s_hotTemp - 25.0f;
  uint32_t s_roomC_hotC = roomCount - s_hotCount;

  // calculate count from temperature and calibration data
  tempCodeVal = (uint32_t)(s_hotCount + (s_hotTemp - temperatureCelsius) * s_roomC_hotC / s_hot_ROOM);

  return tempCodeVal;  
}

void InternalTemperatureClass::getRegisterCounts (int *currentCount, int *highTempCount, int *lowTempCount) {
  *currentCount = (int)(TEMPMON_TEMPSENSE0 & 0xFFF00U) >> 8U;
  *highTempCount = (int)(TEMPMON_TEMPSENSE0 & 0xFFF00000U) >> 20U;
  *lowTempCount = (int)(TEMPMON_TEMPSENSE2 & 0xFFFU) >> 0U;
}

//
//  Calibration functions - Teensy 4 comes calibrated from the factory, so does nothing
//

bool InternalTemperatureClass::singlePointCalibrationC (
  float actualTemperatureC, float measuredTemperatureC, bool fromDefault) {
   return true;
}

bool InternalTemperatureClass::singlePointCalibrationF (
  float actualTemperatureF, float measuredTemperatureF, bool fromDefault) {
   return true;
}

bool InternalTemperatureClass::dualPointCalibrationC (
  float actualTemperature1C, float measuredTemperature1C,
  float actualTemperature2C, float measuredTemperature2C, bool fromDefault) {
   return true;
}

bool InternalTemperatureClass::dualPointCalibrationF (
  float actualTemperature1F, float measuredTemperature1F,
  float actualTemperature2F, float measuredTemperature2F, bool fromDefault) {
   return true;
}

bool InternalTemperatureClass::setVTemp25 (float volts) {
  return true;
}

bool InternalTemperatureClass::setSlope (float voltsPerDegreeC) {
  return true;
}

float InternalTemperatureClass::getVTemp25 () {
  return 0.0;
}

float InternalTemperatureClass::getSlope () {
  return 0.0;
}

// Unique ID can be used to set calibration values by serial number
int InternalTemperatureClass::getUniqueID () {
  return HW_OCOTP_MAC0;
}

#endif  // if Teensy 4.0 else

//
// Common to Teemsy 3 and Teensy 4
//

void InternalTemperatureClass::alarmISR () {

  if (alarm == HighTempAlarm) {
    highTempISR (); 

    // prevent future interrupts
    detachHighTempInterrupt ();
  }
  else if (alarm == LowTempAlarm) {
    lowTempISR (); 

    // prevent future interrupts
    detachLowTempInterrupt ();
  }
  else if (alarm == BothAlarms) {

    // figure out which one it was by closest temperature
    int currentCount = 0;
    int highTempCount = 0;
    int lowTempCount = 0;

    getRegisterCounts (&currentCount, &highTempCount, &lowTempCount);

    // if closer to high temp than low temp
    if (abs (currentCount - highTempCount) < abs (currentCount - lowTempCount)) {
      highTempISR (); 

      // prevent future interrupts
      detachHighTempInterrupt ();
    }
    else {
      lowTempISR (); 

      // prevent future interrupts
      detachLowTempInterrupt ();
    }
  }
}

int InternalTemperatureClass::attachHighTempInterruptCelsius (float triggerTemperature, void (*function)(void)) {

  highTempISR = function;

  if (alarm == NoAlarm) {
    alarm = HighTempAlarm;
  }
  else if (alarm == LowTempAlarm) {
    alarm = BothAlarms;
  }

  return attachInterruptCelsius (triggerTemperature, HighTemperature);
}

int InternalTemperatureClass::attachHighTempInterruptFahrenheit (float triggerTemperature, void (*function)(void)) {
  return attachHighTempInterruptCelsius (toCelsius (triggerTemperature), function);
}

int InternalTemperatureClass::attachLowTempInterruptCelsius (float triggerTemperature, void (*function)(void)) {

  lowTempISR = function;

  if (alarm == NoAlarm) {
    alarm = LowTempAlarm;
  }
  else if (alarm == HighTempAlarm) {
    alarm = BothAlarms;
  }

  return attachInterruptCelsius (triggerTemperature, LowTemperature);
}

int InternalTemperatureClass::attachLowTempInterruptFahrenheit (float triggerTemperature, void (*function)(void)) {
  return attachLowTempInterruptCelsius (toCelsius (triggerTemperature), function);
}


// Utilities

float InternalTemperatureClass::toCelsius (float temperatureFahrenheit) {
  // convert fahrenheit to celsius 
  return (temperatureFahrenheit - 32) * 5.0 / 9.0;
}

float InternalTemperatureClass::toFahrenheit (float temperatureCelsius) {
  // convert celsius to fahrenheit
  return temperatureCelsius * 9.0 / 5.0 + 32;
}
