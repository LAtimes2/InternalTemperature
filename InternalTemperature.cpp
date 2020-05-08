/* InternalTemperature - read internal temperature of ARM processor
 * Copyright (C) 2020 LAtimes2
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

#include "InternalTemperature.h"
#include "Arduino.h"

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
  #define TEMPERATURE_PIN 70
  #define VREF_PIN 71
#else
  #define TEMPERATURE_PIN 38
  #define VREF_PIN 39
#endif

// Constructor
InternalTemperature::InternalTemperature()
  : slope (DEFAULT_SLOPE)
  , vTemp25 (DEFAULT_VTEMP25)
{
}

bool InternalTemperature::begin (int temperature_settings_type) {

  // TeensyLC needs the Bandgap voltage for measurements. All Teensy's need
  // the Bandgap turned on in low power mode.
  // So always turn it on. If it is already on, it doesn't hurt anything.
  enableBandgap ();

  if (temperature_settings_type == TEMPERATURE_MAX_ACCURACY)
  {
    // set to internal 1.2V reference
    analogReference(INTERNAL);

    // set to maximum number of bits
    analogReadResolution(16);

    // set to maximum averaging
    analogReadAveraging(32);
  }

  return true;
}

void InternalTemperature::enableBandgap (void) {

  // Bandgap enable
  PMC_REGSC |= PMC_REGSC_BGEN | PMC_REGSC_BGBE;

  // Delay for regulator to start up
  delay (2);
}

float InternalTemperature::readRawTemperatureVoltage () {
  return readRawVoltage (TEMPERATURE_PIN);
}

float InternalTemperature::readRawVoltage (int signalNumber) {

#if defined(__MKL26Z64__)   // Teensy LC 
  // uses Bandgap voltage since it doesn't have a reference voltage
  const float vRef = 1.0;
#else
  const float vRef = 1.195;
#endif

  int analogValue;
  int analogVref;
  float volts;
  float floatVref;

  int previousADC0_CFG1 = ADC0_CFG1;
  int previousADC0_CFG2 = ADC0_CFG2;

  //
  // set to max sample time (for best accuracy). This is
  // especially needed when using external Vref.
  //

  // enable long sample times
  ADC0_CFG1 |= ADC_CFG1_ADLSMP;

  // 00 = 32 clocks
  ADC0_CFG2 &= ~ADC_CFG2_ADLSTS(3);

  //
  // read temperature
  //
  analogValue = analogRead(signalNumber);

  // read Vref
  analogVref = analogRead(VREF_PIN);

/*
  Serial.print(" 0x");
  Serial.print(analogVref, HEX);
  Serial.print(" ");

  Serial.print(" 0x");
  Serial.print(analogValue, HEX);
  Serial.print(" ");
*/
  floatVref = analogVref;

  // compute scaling for Least Significant Bit
  float voltsPerBit = vRef / floatVref;

  // convert analog temperature reading to volts
  volts = analogValue * voltsPerBit;

/*
  Serial.print(", ");
  Serial.print(volts, 3);
  Serial.print(" V ");
*/
  // restore previous values
  ADC0_CFG1 = previousADC0_CFG1;
  ADC0_CFG2 = previousADC0_CFG2;

  return volts;
}

float InternalTemperature::convertTemperatureC (float volts, float vTemp25, float slope) {

  float temperatureCelsius;

  // convert voltage to temperature using equation from CPU Reference Manual
  temperatureCelsius = 25 - ((volts - vTemp25) / slope);

  return temperatureCelsius;
}

float InternalTemperature::convertTemperatureC (float volts) {
  return convertTemperatureC (volts, vTemp25, slope);
}

float InternalTemperature::convertUncalibratedTemperatureC (float volts) {
  return convertTemperatureC (volts, DEFAULT_VTEMP25, DEFAULT_SLOPE);
}

float InternalTemperature::readTemperatureC () {
  return convertTemperatureC (readRawTemperatureVoltage ());
}

float InternalTemperature::readTemperatureF () {
  // convert celsius to fahrenheit
  return toFahrenheit(readTemperatureC());
}

float InternalTemperature::readUncalibratedTemperatureC () {
  return convertUncalibratedTemperatureC (readRawTemperatureVoltage ());
}

float InternalTemperature::readUncalibratedTemperatureF () {
  // convert celsius to fahrenheit
  return toFahrenheit(readUncalibratedTemperatureC());
}

//
//  Calibration functions
//

bool InternalTemperature::singlePointCalibrationC (
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

bool InternalTemperature::singlePointCalibrationF (
  float actualTemperatureF, float measuredTemperatureF, bool fromDefault) {

  return singlePointCalibrationC (toCelsius(actualTemperatureF), toCelsius(measuredTemperatureF), fromDefault);
}

bool InternalTemperature::dualPointCalibrationC (
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

bool InternalTemperature::dualPointCalibrationF (
  float actualTemperature1F, float measuredTemperature1F,
  float actualTemperature2F, float measuredTemperature2F, bool fromDefault) {

  return dualPointCalibrationC (toCelsius(actualTemperature1F), toCelsius(measuredTemperature1F),
                                toCelsius(actualTemperature2F), toCelsius(measuredTemperature2F), fromDefault);
}


bool InternalTemperature::setVTemp25 (float volts)
{
  // perform a range check (0-5 volts)
  if (volts < 0.0 || volts > 5.0) {
    return false;
  }
  vTemp25 = volts;
  return true;
}

bool InternalTemperature::setSlope (float voltsPerDegreeC)
{
  // perform a range check (factor of 10 around default value)
  if (voltsPerDegreeC < (DEFAULT_SLOPE / 10.0) || voltsPerDegreeC > (DEFAULT_SLOPE * 10.0)) {
    return false;
  }
  slope = voltsPerDegreeC;
  return true;
}

float InternalTemperature::getVTemp25 () {
  return vTemp25;
}

float InternalTemperature::getSlope () {
  return slope;
}

// Unique ID can be used to set calibration values by serial number
int InternalTemperature::getUniqueID () {
  return SIM_UIDL;
}

#else   // Teensy 4

//
// Teensy 4.0 uses tempMon, so this is just a thin wrapper around it for backward compatibility
//

// Constructor
InternalTemperature::InternalTemperature()
{
}

bool InternalTemperature::begin (int temperature_settings_type) {
   return true;
}

float InternalTemperature::readTemperatureC () {
  return tempmonGetTemp();
}

float InternalTemperature::readTemperatureF () {
  // convert celsius to fahrenheit
  return toFahrenheit(readTemperatureC());
}

//
//  Calibration functions - Teensy 4 comes calibrated from the factory, so does nothing
//

bool InternalTemperature::singlePointCalibrationC (
  float actualTemperatureC, float measuredTemperatureC, bool fromDefault) {
   return true;
}

bool InternalTemperature::singlePointCalibrationF (
  float actualTemperatureF, float measuredTemperatureF, bool fromDefault) {
   return true;
}

bool InternalTemperature::dualPointCalibrationC (
  float actualTemperature1C, float measuredTemperature1C,
  float actualTemperature2C, float measuredTemperature2C, bool fromDefault) {
   return true;
}

bool InternalTemperature::dualPointCalibrationF (
  float actualTemperature1F, float measuredTemperature1F,
  float actualTemperature2F, float measuredTemperature2F, bool fromDefault) {
   return true;
}


bool InternalTemperature::setVTemp25 (float volts) {
  return true;
}

bool InternalTemperature::setSlope (float voltsPerDegreeC) {
  return true;
}

float InternalTemperature::getVTemp25 () {
  return 0.0;
}

float InternalTemperature::getSlope () {
  return 0.0;
}

// Unique ID can be used to set calibration values by serial number
int InternalTemperature::getUniqueID () {
  // TODO: use HW_OCOTP_CFG0?
  return 0;
}

#endif  // if Teensy 4.0 else

// Utilities

float InternalTemperature::toCelsius (float temperatureFahrenheit) {
  // convert fahrenheit to celsius 
  return (temperatureFahrenheit - 32) * 5.0 / 9.0;
}

float InternalTemperature::toFahrenheit (float temperatureCelsius) {
  // convert celsius to fahrenheit
  return temperatureCelsius * 9.0 / 5.0 + 32;
}
