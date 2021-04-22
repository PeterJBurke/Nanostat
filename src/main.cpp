bool userpause = false;              // pauses for user to press input on serial between each point in npv
bool print_output_to_serial = false; // pauses for user to press input on serial between each point in npv

//Standard Arduino Libraries
#include <Wire.h>
#include <WiFi.h>
#include "Arduino.h"
#include <ESPAsyncWebServer.h>
#include <ESPmDNS.h>
#include <SPIFFS.h>
#include <AsyncJson.h>
//#include <AsyncJson.h>
#include "wifi_credentials.h"
//Custom Internal Libraries
#include "LMP91000.h"

// Arduino serial interface
#define SerialDebugger Serial

// Microcontroller specific info (pinouts, Vcc, etc)
const uint16_t opVolt = 3300;                 //3300 mV
const uint8_t adcBits = 12;                   // ESP32 ADC is 12 bits
const uint16_t dacResolution = pow(2, 8) - 1; // ESP32 DAC is 8 bits, that is 12.9 mV for 3300 mV reference...
//LMP91000 control pins
const uint8_t dac = 25; // DAC pin for Vref to drive LMP91000. Pin 25 on BurkeLab ESP32Stat Rev 3.5
const uint8_t MENB = 5; // LMP91000 enable pin. Hard wired to ground in BurkeLab ESP32Stat Rev 3.5
//analog input pins to read voltages
const uint8_t LMP_C1 = 27; // Note C1, C2 not wired to microcontroller in BurkeLab ESP32Stat Rev 3.5
const uint8_t LMP_C2 = 39; // Note C1, C2 not wired in microcontroller in BurkeLab ESP32Stat Rev 3.5
const uint8_t LMP = 35;    // ADC pin for Vout reading. Pin 35 on BurkeLab ESP32Stat Rev 3.5
// calibration of ESP32 ADC (see calibration routine)
float a_coeff = -146.63; // hard code coeffs but they can be updated with calibration routine
float b_coeff = 7.64;    // hard code coeffs but they can be updated with calibration routine
// For BurkeLab ESP32Stat Rev 3.5, LED "blinky" pin.
int LEDPIN = 26;

// Global mode control (sweep type)
enum Sweep_Mode_Type
{
  dormant,  // not sweeping
  NPV,      // normal pulsed voltametry
  CV,       // cyclic voltametry
  SQV,      // square wave voltametry
  CA,       // chronoamperometry
  IV,       // IV curve
  CAL,      // calibrate
  DCBIAS,   // DC bias at a fixed point
  MISC_MODE // miscellanous
};
Sweep_Mode_Type Sweep_Mode = dormant;

// Sweep parameters (to be set by user through HTML form posts but defaults on initialization)
// (Initialized to their default values)
// General sweep parameters:
int sweep_param_lmpGain = 7;       //  (index) gain setting for LMP91000
bool sweep_param_setToZero = true; //   Boolean determining whether the bias potential of

// CV sweep parameters:
// runCV(sweep_param_lmpGain, sweep_param_cycles_CV, sweep_param_startV_CV,
// sweep_param_endV_CV, sweep_param_vertex1_CV, sweep_param_vertex2_CV, sweep_param_stepV_CV,
// sweep_param_rate_CV, sweep_param_setToZero);
int sweep_param_cycles_CV = 3;     //  (#)  number of times to run the scan
int sweep_param_startV_CV = 0;     //   (mV)  voltage to start the scan
int sweep_param_endV_CV = 0;       //     (mV)  voltage to stop the scan
int sweep_param_vertex1_CV = 100;  //  (mV)  edge of the scan
int sweep_param_vertex2_CV = -100; // (mV)  edge of the scan
int sweep_param_stepV_CV = 5;      // (mV)      how much to increment the voltage by
int sweep_param_rate_CV = 100;     // (mV/sec)       scanning rate

// NPV sweep parameters:
// runNPV(sweep_param_lmpGain, sweep_param_startV_NPV, sweep_param_endV_NPV,
//            sweep_param_pulseAmp_NPV, sweep_param_pulseAmp_NPV, sweep_param_period_NPV,
//            sweep_param_quietTime_NPV, uint8_t range, sweep_param_setToZero)
int sweep_param_startV_NPV = -200; //   (mV)  voltage to start the scan
int sweep_param_endV_NPV = 500;    //     (mV)  voltage to stop the scan
int sweep_param_pulseAmp_NPV = 1;
int sweep_param_width_NPV = 50;
int sweep_param_period_NPV = 200;
int sweep_param_quietTime_NPV = 1000;

// SWV sweep parameters:
// runSWV(sweep_param_lmpGain, sweep_param_startV_SWV, sweep_param_endV_SWV,
//            sweep_param_pulseAmp_SWV, sweep_param_stepV_SWV, sweep_param_freq_SWV, sweep_param_setToZero)
int sweep_param_startV_SWV = -200; //   (mV)  voltage to start the scan
int sweep_param_endV_SWV = 500;    //     (mV)  voltage to stop the scan
int sweep_param_pulseAmp_SWV = 25;
int sweep_param_stepV_SWV = 5; // (mV)      how much to increment the voltage by
int sweep_param_freq_SWV = 10;

// CA sweep parameters:
// runAmp(sweep_param_lmpGain, sweep_param_pre_stepV_CA, sweep_param_quietTime_CA,
//            sweep_param_V1_CA, sweep_param_t1_CA, sweep_param_V2_CA, sweep_param_t2_CA,
//            sweep_param_samples_CA, uint8_t range, sweep_param_setToZero)
int sweep_param_pre_stepV_CA = 50;
int sweep_param_quietTime_CA = 2000;
int sweep_param_V1_CA = 100;
int sweep_param_t1_CA = 2000;
int sweep_param_V2_CA = 50;
int sweep_param_t2_CA = 200;
int sweep_param_samples_CA = 100;

// Noise test sweep parameters:
// testNoiseAtABiasPoint(sweep_param_biasV_noisetest, sweep_param_numPoints_noisetest,
//                           sweep_param_delayTime_ms_noisetest)
int sweep_param_biasV_noisetest = -100;
int sweep_param_numPoints_noisetest = 100;
int sweep_param_delayTime_ms_noisetest = 50;

// IV sweep parameters:
// testIV(sweep_param_startV_IV, sweep_param_endV_IV, sweep_param_numPoints_IV,
//            sweep_param_delayTime_ms_IV)
int sweep_param_startV_IV = -200; //   (mV)  voltage to start the scan
int sweep_param_endV_IV = 500;    //     (mV)  voltage to stop the scan
int sweep_param_numPoints_IV = 701;
int sweep_param_delayTime_ms_IV = 50;

// Cal sweep parameters:
// calibrateDACandADCs(sweep_param_delayTime_ms_CAL)
int sweep_param_delayTime_ms_CAL = 50;

// Arrays of IV curves etc:
const uint16_t arr_samples = 2500; //use 1000 for EIS, can use 2500 for other experiments
uint16_t arr_cur_index = 0;
int16_t volts[arr_samples] = {0};                   // single sweep IV curve "V"
float amps[arr_samples] = {0};                      // single sweep IV curve "I"
int number_of_valid_points_in_volts_amps_array = 0; // rest of them are all zeros...
unsigned long input_time[arr_samples] = {0};
unsigned long output_time[arr_samples] = {0};
float v1_array[arr_samples] = {0};
float v2_array[arr_samples] = {0};
const float v_tolerance = 0.008;     //0.0075 works every other technique with 1mV step except CV which needs minimum 2mV step
unsigned long lastTime = 0;          // global variable, last time xyz was called in  ms....
uint16_t dacVout = 1500;             // desired output of the DAC in mV
float adc_avg = 0;                   // used in noise test subroutine, better to make it local eventually...
float adc_std_dev = 0;               // used in noise test subroutine, better to make it local eventually...
int num_adc_readings_to_average = 1; // when reading ADC, how many points to average....

// LMP91000 global status settings:
// Parameters the user can set on LMP91000:
// TIA gain (keep max at 350k feedback resistor)
// Rload (keep at 10 ohms)
// Ref source int/ext (keep ext)
// Int_Z zero 50 20 67% (keep at 50%)
// Bias_Sign (plus/minus)
// Bias 1%-24%
// FET_Short (keep off)
// Mode (keep at 3-lead)
uint8_t LMPgain = 6; // Feedback resistor of TIA.
//void LMP91000::setGain(uint8_t gain) const
//@param            gain: the gain to be set to
//param - value - gain resistor
//0 - 000 - External resistor
//1 - 001 - 2.75 kOhm
//2 - 010 - 3.5 kOhm
//3 - 011 - 7 kOhm
//4 - 100 - 14 kOhm
//5 - 101 - 35 kOhm
//6 - 110 - 120 kOhm
//7 - 111 - 350 kOhm
uint8_t bias_setting = 0; // determines percentage of VREF applied to CE opamp, from 1% to 24%
// from LMP91000.h:
//const double TIA_BIAS[] = {0, 0.01, 0.02, 0.04, 0.06, 0.08, 0.1, 0.12, 0.14,
//    0.16, 0.18, 0.2, 0.22, 0.24};
float RFB = 2200000; // feedback resistor if using external feedback resistor

// create pStat object to control LMP91000
LMP91000 pStat = LMP91000();

// create webserver object for website:
AsyncWebServer server(80);
// example code to control LED from browser used here from
// https://www.youtube.com/watch?v=aNDfsHQ5Gts&list=PL4cUxeGkcC9jx2TTZk3IGWKSbtugYdrlu&index=2
const char *PARAM_MESSAGE = "message"; // message server receives from client

// Legacy from LMP91000 examples in library:
bool saveQueues = false;

//******************* END VARIABLE DECLARATIONS**************************8

void blinkLED(int pin, int blinkFrequency_Hz, int duration_ms)
// blinks LED for duration ms using frequency of blinkFrequency
{
  int delayPeriod_ms = (1e3) / (2 * blinkFrequency_Hz);
  int t = 0;
  while (t < duration_ms)
  {
    digitalWrite(pin, HIGH);
    delay(delayPeriod_ms);
    digitalWrite(pin, LOW);
    delay(delayPeriod_ms);
    t += 2 * delayPeriod_ms;
  }
}

void pulseLED_on_off(int pin, int on_duration_ms)
// blinks LED on for on_duration_ms
{
  digitalWrite(pin, HIGH);
  delay(on_duration_ms);
  digitalWrite(pin, LOW);
}

inline void setLMPBias(int16_t voltage)
{
  //Sets the LMP91000's bias to positive or negative
  signed char sign = (float)voltage / abs(voltage);

  if (sign < 0)
    pStat.setNegBias();
  else if (sign > 0)
    pStat.setPosBias();
  else
  {
  } //do nothing
}

void setOutputsToZero()
{
  //Sets the electrochemical cell to 0V bias.
  //  analogWrite(dac,0); // SAMD21
  dacWrite(dac, 0); // ESP32

  pStat.setBias(0);
}

void initLMP(uint8_t lmpGain)
{
  //Initializes the LMP91000 to the appropriate settings
  //for operating MiniStat.
  pStat.disableFET();      // Don't short WE and RE.
  pStat.setGain(lmpGain);  // Set feedback resistor of LMP gain.
  pStat.setRLoad(0);       // Low resistance (10 ohms) between WE and TIA input.
  pStat.setExtRefSource(); // Cell voltage determined by external pin.
  pStat.setIntZ(1);        // TIA uses internal resistor for gain setting.
  pStat.setThreeLead();    // 3 lead potentiostat configuration.
  pStat.setBias(0);        // Zero voltage on cell, DAC.
  pStat.setPosBias();      // Positive bias.

  setOutputsToZero(); // Zero voltage on cell, DAC.
}

inline uint16_t convertDACVoutToDACVal(uint16_t dacVout)
{
  //dacVout       voltage output to set the DAC to
  //
  //Determines the correct value to write to the digital-to-analog
  //converter (DAC) given the desired voltage, the resolution of the DAC

  // dacResolution=2^8-1 ESP32 = 255
  // opVolt =3300 mV
  // DACVal integer 0 to 255 scaled to dacVout/opVolt
  // so resolution is 13 mV!!!

  return dacVout * ((float)dacResolution / opVolt);
}

inline float analog_read_avg(int num_points, int pin_num)
{
  // Reads adc at pin_num for num_points  and returns average.
  float analogRead_result = 0;
  // SerialDebugger.println("");
  // SerialDebugger.println("=======================");

  for (int16_t j = 0; j < num_points; j += 1)
  {
    analogRead_result += analogRead(pin_num);
    //SerialDebugger.println(" x x x x x ");
    // SerialDebugger.print("j=");
    // SerialDebugger.print(F("\t"));
    // SerialDebugger.print(j);
    // SerialDebugger.print(F("\t"));
    // SerialDebugger.print("analogRead_result=");
    // SerialDebugger.print(F("\t"));
    // SerialDebugger.println(analogRead_result);
    // SerialDebugger.println(" x x x x x ");
  }
  //  SerialDebugger.println("=======================");

  return analogRead_result / num_points;
}

inline void setVoltage(int16_t voltage)
{
  // Sets the DAC voltage, LMP91000 bias percentage, and LMP91000 bias sign, to get the desired cell voltage.
  // "voltage" is the desired cell voltage (RE minus WE)
  // Assumes DAC is perfect.

  // This function iterates bias_setting, dacVout until it finds a solution within v_tolerance of voltage (the desired cell voltage)
  // End result is bias_setting (integer), dacVout (in mV) which goes to LMP91000 and ESP32
  // setV (the cell voltage the user will actually get)
  // is given by dacVout * TIA_BIAS[bias_setting]
  // TIA_BIAS[bias_setting] varies from 1% to 22% depending on setting; note initially set to 0%...

  // global variables used in this function:
  // v_tolerance: how close to desired cell voltage user can tolerate since it is digitized
  // dacVout: desired output of the DAC in mV
  // bias_setting: determines percentage of VREF applied to CE opamp, from 1% to 24%

  const uint16_t minDACVoltage = 1520; // Minimum DAC voltage that can be set;
                                       //the LMP91000 accepts a minium value of 1.5V, adding the
  //additional 20 mV for the sake of a bit of a buffer

  dacVout = minDACVoltage; // global variable, initialized to a convenient value in this method, will be changed in iteration
  bias_setting = 0;        // global variable, initialized to a convenient value in this method, will be changed in iteration

  //if (abs(voltage) < 15)
  //{
  // voltage = 15 * (voltage / abs(voltage));
  //voltage cannot be set to less than 15mV because the LMP91000
  //accepts a minium of 1.5V at its VREF pin and has 1% as its
  //lowest bias option 1.5V*1% = 15mV
  //}

  int16_t setV = dacVout * TIA_BIAS[bias_setting]; // "setV" is the exact cell voltage we will get, try to get it close to "voltage"
  voltage = abs(voltage);                          // (sign handled elsewhere in the code...)

  if (abs(voltage) < 15)
  {
    // make executive decision
    if (abs(voltage) > 7.5)
    {
      //    make it 15 mV
      voltage = 15 * (voltage / abs(voltage));
    }
    else if (abs(voltage) <= 7.5)
    {
      // make it zero
      bias_setting = 0;
      setV = 0;
      dacVout = 1500;
    }
  }

  if (abs(voltage) >= 15)
  {
    // iterate
    while (setV > voltage * (1 + v_tolerance) || setV < voltage * (1 - v_tolerance)) // iterate to find
    // bias_setting (integer), dacVout (in mV) which goes to LMP91000 and ESP32
    // so that setV = dacVout * TIA_BIAS[bias_setting] is within v_tolerance of voltage (desired by user)
    {
      if (bias_setting == 0)
        bias_setting = 1;

      dacVout = voltage / TIA_BIAS[bias_setting];

      if (dacVout > opVolt)
      {
        bias_setting++;
        dacVout = 1500;

        if (bias_setting > NUM_TIA_BIAS)
          bias_setting = 0;
      }

      setV = dacVout * TIA_BIAS[bias_setting];
    }
  }
  pStat.setBias(bias_setting); // sets percentage voltage divider in LMP910000
  // dacwrite is ESP32 version of analogWrite
  dacWrite(dac, convertDACVoutToDACVal(dacVout)); // sets ESP32 DAC bits
  //SerialDebugger.println("dacVout=");
  //SerialDebugger.println(dacVout);
  //SerialDebugger.println("convertDACVoutToDACVal(dacVout)=");
  //SerialDebugger.println(convertDACVoutToDACVal(dacVout));
  //SerialDebugger.println("3300*convertDACVoutToDACVal(dacVout)/255=");
  //SerialDebugger.println(3300 * convertDACVoutToDACVal(dacVout) / 255);

  // analogWrite(dac,convertDACVoutToDACVal(dacVout));

  //  SerialDebugger.print(dacVout*.5);
  //  SerialDebugger.print(F("\t"));
  //  SerialDebugger.print(dacVout);
  //  SerialDebugger.print(F("\t"));

  //  SerialDebugger.print(TIA_BIAS[bias_setting]);
  //  SerialDebugger.print(F("\t"));
}

inline float biasAndSample(int16_t voltage, uint32_t rate)
{
  //@param        voltage: Set the bias of the electrochemical cell
  //@param        rate:    How much to time (in ms) should we wait to sample
  //                       current after biasing the cell. This parameter
  //                       sets the scan rate or the excitation frequency
  //                       based on which electrochemical technique
  //                       we're running.
  //
  //Sets the bias of the electrochemical cell then samples the resulting current.

  // Global variables assumed:
  // lastTime: last time in system ms this function was called (for timing reference)
  // num_adc_readings_to_average: self explanatory
  // float a = a_coeff; float b = b_coeff; // global calibration coefficients for ADC to voltage correction

  // Global variables used in this function implicit through setVoltage:
  // v_tolerance: how close to desired cell voltage user can tolerate since it is digitized
  // dacVout: desired output of the DAC in mV
  // bias_setting: determines percentage of VREF applied to CE opamp, from 1% to 24%

  setLMPBias(voltage);         // Sets the LMP91000's bias to positive or negative // voltage is cell voltage
  setVoltage(voltage);         // Sets the DAC voltage, LMP91000 bias percentage, and LMP91000 bias sign, to get the desired cell "voltage".
  pulseLED_on_off(LEDPIN, 10); // signify start of a new data point

  //delay sampling to set scan rate
  while (millis() - lastTime < rate)
    ;

  // Read output voltage of the transimpedance amplifier
  int adc_bits; // will be output voltage of TIA amplifier, "VOUT" on LMP91000 diagram, also C2
  // hard wired in BurkeLab ESP32Stat Rev 3.5 to LMP i.e ESP32 pin 32 (ADC1_CH7)
  //adc_bits = analogRead(LMP); // read a single point
  adc_bits = analog_read_avg(num_adc_readings_to_average, LMP); // read a number of points and average them...
  // Now convert adc_bits to actual voltage on Vout.
  // Without calibratoin, we would assume that virtual ground (WE = C1) is 50% of Vref.
  // Without calibratoin, we would assume that C2-C1=I_WE * RTIA, and C2 is VOUT is ADC reading.
  // However, C1 is not 50% of Vref.
  // Calibration takes care of this by reading C2 as a function of ADC set voltage (assuming perfect ADC) and then
  // correcting C2 using Vout_calibrated = (adcbits/4095)*3.3V corrected by slope/offset measured during calibration, as follows:
  // First, user opens WE so no current flows, then runs the xyz function, which sets LMP cell voltage to Zero and varies Vref i.e. Vdac.
  // That function measures Vadc as a function of Vdac and fits this formula (defining a,b):
  // adc(bits) = a + b * dac = a + b * dacvoltage * (255/3.3); here dac voltage is assumed correct
  // Correction from ADC to Vout then given by:
  // v1 = (3.3 / 255.0) * (1 / (2.0 * b)) * (float)adc_bits - (a / (2.0 * b)) * (3.3 / 255.0); // LMP is wired to Vout of the LMP91000

  //  Calibrate coefficients (make a local copy of the global ones):
  float a = a_coeff; // a is local copy of global a_coeff
  float b = b_coeff; // b is local copy of global a_coeff
  // temporary hard code cal coefficients for noise testing xxx
  // a = -121.34;
  // b = 7.67;

  float v1;
  v1 = (3.3 / 255.0) * (1 / (2.0 * b)) * (float)adc_bits - (a / (2.0 * b)) * (3.3 / 255.0); // LMP is wired to Vout of the LMP91000
  v1 = v1 * 1000;
  float v2 = dacVout * .5; //the zero of the internal transimpedance amplifier
  // V2 is not measured in  BurkeLab ESP32Stat Rev 3.5 and assumed to be half dacVout, calibration helps this see above
  float current = 0;

  //the current is determined by the zero of the transimpedance amplifier
  //from the output of the transimpedance amplifier, then dividing
  //by the feedback resistor
  //current = (V_OUT - V_IN-) / RFB
  //v1 and v2 are in milliVolts
  if (LMPgain == 0)                                    // using external feedback resistor, not (yet) suppored with BurkeLab ESP32Stat Rev 3.5
    current = (((v1 - v2) / 1000) / RFB) * pow(10, 9); //scales to nA
  else
    current = (((v1 - v2) / 1000) / TIA_GAIN[LMPgain - 1]) * pow(10, 6); //scales to uA

  if (print_output_to_serial)
  {
    SerialDebugger.print(int(millis())); // current time in ms
    SerialDebugger.print(F(","));        // comma
    //SerialDebugger.print(F("\t"));                          // tab
    SerialDebugger.print(voltage); // desired cell voltage
    SerialDebugger.print(F(","));  // comma
    //SerialDebugger.print(F("\t"));                          // tab
    SerialDebugger.print(dacVout * TIA_BIAS[bias_setting]); // SET cell voltage
    // setV = dacVout * TIA_BIAS[bias_setting]
    SerialDebugger.print(F(",")); // comma
    //SerialDebugger.print(F("\t"));                          // tab
    SerialDebugger.print(TIA_BIAS[bias_setting], 2); // scale divider
    SerialDebugger.print(F(","));                    // comma
    //SerialDebugger.print(F("\t"));                          // tab
    SerialDebugger.print(dacVout, DEC); // dacVout a global variable, calculated in setVoltage(voltage);
    SerialDebugger.print(F(","));       // comma
    //SerialDebugger.print(F("\t"));                          // tab
    SerialDebugger.print(adc_bits, 0); // adc_bits
    SerialDebugger.print(F(","));      // comma
    //SerialDebugger.print(F("\t"));                          // tab
    SerialDebugger.print(v1, 1);  // Vout
    SerialDebugger.print(F(",")); // comma
    //SerialDebugger.print(F("\t"));                          // tab
    SerialDebugger.print(v2, 0);  // C1, should be the zero also....
    SerialDebugger.print(F(",")); // comma
    //SerialDebugger.print(F("\t"));                          // tab
    SerialDebugger.print(current, 4);
    SerialDebugger.print(F(",")); // comma
    //SerialDebugger.print(F("\t"));                          // tab
  }

  //update timestamp for the next measurement
  lastTime = millis();

  return current;
}

void writeVoltsCurrentArraytoFile()
{
  // File file = SPIFFS.open("/file.txt", "w");
  // if (!file)
  // {
  //   Serial.println("Error opening file for writing");
  //   return;
  // }
  // int bytesWritten = file.print("TEST SPIFFS");

  // if (bytesWritten > 0)
  // {
  //   Serial.println("File was written");
  //   Serial.println(bytesWritten);
  // }
  // else
  // {
  //   Serial.println("File write failed");
  // }
  // file.close();

  File file = SPIFFS.open("/data.txt", FILE_WRITE);

  if (!file)
  {
    Serial.println("There was an error opening the file for writing");
    return;
  }
  if (file.println("Hello world ESP32 Spiffs file!"))
  {
    Serial.println("File was written");
  }
  else
  {
    Serial.println("File write failed");
  }
  file.println("Line 2!");
  file.println("Line 3!");
  Serial.println("Writing file. Date in millis() is:");
  Serial.println(millis());
  Serial.println("number_of_valid_points_in_volts_amps_array=");
  Serial.println(number_of_valid_points_in_volts_amps_array);

  file.print("Index");
  file.print(F("\t")); // tab
  file.print("Current_Amps");
  file.print(F("\t")); // tab
  file.println("Voltage_V");

  //Wrie Arrays
  for (uint16_t i = 0; i < number_of_valid_points_in_volts_amps_array; i++)
  {
    file.print(i);
    file.print(F("\t")); // tab
    file.print(amps[i], DEC);
    file.print(F("\t")); // tab
    file.println(volts[i] / 1e3, DEC);
  }
  file.close();
}

void writeVoltsCurrentTimeArraytoFile()
{
  // File file = SPIFFS.open("/file.txt", "w");
  // if (!file)
  // {
  //   Serial.println("Error opening file for writing");
  //   return;
  // }
  // int bytesWritten = file.print("TEST SPIFFS");

  // if (bytesWritten > 0)
  // {
  //   Serial.println("File was written");
  //   Serial.println(bytesWritten);
  // }
  // else
  // {
  //   Serial.println("File write failed");
  // }
  // file.close();

  File file = SPIFFS.open("/data.txt", FILE_WRITE);

  if (!file)
  {
    Serial.println("There was an error opening the file for writing");
    return;
  }
  if (file.println("BurkeLab Nanostat Rev 3.5 Sweep"))
  {
    Serial.println("File was written");
  }
  else
  {
    Serial.println("File write failed");
  }
  // file.println("Line 2!");
  // file.println("Line 3!");
  Serial.println("Writing file. Date in millis() is:");
  Serial.println(millis());
  Serial.println("number_of_valid_points_in_volts_amps_array=");
  Serial.println(number_of_valid_points_in_volts_amps_array);

  file.print("Index");
  file.print(F("\t")); // tab
  file.print("Time");
  file.print(F("\t")); // tab
  file.print("Current_Amps");
  file.print(F("\t")); // tab
  file.println("Voltage_V");

  //Write Arrays
  for (uint16_t i = 0; i < number_of_valid_points_in_volts_amps_array; i++)
  {
    file.print(i);
    file.print(F("\t")); // tab
    file.print(output_time[i]);
    file.print(F("\t")); // tab
    file.print(amps[i], DEC);
    file.print(F("\t")); // tab
    file.println(volts[i] / 1e3, DEC);
  }
  file.close();
}

void readFileAndPrintToSerial()
{
  File file2 = SPIFFS.open("/data.txt");

  if (!file2)
  {
    Serial.println("Failed to open file for reading");
    return;
  }

  Serial.println("File Content:");

  while (file2.available())
  {

    Serial.write(file2.read());
  }

  file2.close();
}

// void downloadfile()
// {

//   File todownload = SPIFFS.open("/data.txt", "r");
//   if (todownload)
//   {
//     server.sendHeader("Content-Type", "text/text");
//     server.sendHeader("Content-Disposition", "attachment; filename=" + filename);
//     server.sendHeader("Connection", "close");
//     server.streamFile(todownload, "application/octet-stream");
//     todownload.close();
//   }
//   else
//   {
//     Serial.println("Couldn't open file data.txt.")
//   }
// }
// }

void listDir(const char *dirname, uint8_t levels)
{
  // from https://github.com/espressif/arduino-esp32/blob/master/libraries/SPIFFS/examples/SPIFFS_Test/SPIFFS_Test.ino#L9
  // see also https://techtutorialsx.com/2019/02/24/esp32-arduino-listing-files-in-a-spiffs-file-system-specific-path/
  Serial.printf("Listing directory: %s\r\n", dirname);

  File root = SPIFFS.open(dirname);
  if (!root)
  {
    Serial.println("- failed to open directory");
    return;
  }
  if (!root.isDirectory())
  {
    Serial.println(" - not a directory");
    return;
  }

  File file = root.openNextFile();
  while (file)
  {
    // if(file.isDirectory()){
    //     Serial.print("  DIR : ");
    //     Serial.println(file.name());
    //     if(levels){
    //         listDir(SPIFFS, file.path(), levels -1);
    //     }
    // } else {
    //     Serial.print("  FILE: ");
    //     Serial.print(file.name());
    //     Serial.print("\tSIZE: ");
    //     Serial.println(file.size());
    // }

    Serial.print("  FILE: ");
    Serial.print(file.name());
    Serial.print("\tSIZE: ");
    Serial.println(file.size());

    file = root.openNextFile();
  }
}

inline void saveVoltammogram(float voltage, float current, bool debug)
{
  //@param        voltage: voltage or time depending on type of experiment
  //                       voltage for voltammetry, time for time for
  //                       time evolution experiments like chronoamperometry
  //@param        current: current from the electrochemical cell
  //@param        debug:   flag for determining whether or not to print to
  //                       serial monitor
  //
  //Save voltammogram data to corresponding arrays.
  // if (saveQueues && arr_cur_index < arr_samples)
  // {
  //   volts[arr_cur_index] = (int16_t)voltage;
  //   amps[arr_cur_index] = current;
  //   arr_cur_index++;
  //   number_of_valid_points_in_volts_amps_array += 1;
  // }
  volts[arr_cur_index] = (int16_t)voltage;
  amps[arr_cur_index] = current;
  arr_cur_index++;
  number_of_valid_points_in_volts_amps_array++;

  if (debug)
  {
    SerialDebugger.print(voltage);
    //SerialDebugger.print(F("\t"));
    SerialDebugger.print(F(","));
    SerialDebugger.print(current, 5);
    SerialDebugger.print(F(","));
    SerialDebugger.print(micros());
  }
}
void testIV(int16_t startV, int16_t endV, int16_t numPoints,
            uint32_t delayTime_ms)
{

  //@param      startV:       voltage to start the scan
  //@param      endV:         voltage to stop the scan
  //@param      numPoints:    number of points in the scan
  //@param      delayTime_ms:  settle time after set point
  //
  // Measures IV curve, dumps output to serial.
  float i_forward = 0;
  uint32_t voltage_step;

  //Reset Arrays
  for (uint16_t i = 0; i < arr_samples; i++)
    volts[i] = 0;
  for (uint16_t i = 0; i < arr_samples; i++)
    amps[i] = 0;

  arr_cur_index = 0;
  number_of_valid_points_in_volts_amps_array = 0;

  voltage_step = (endV - startV) / numPoints;
  if (voltage_step == 0)
  {
    voltage_step = 1; // 1 mV minimum
  }

  SerialDebugger.println(startV);
  SerialDebugger.println(endV);
  SerialDebugger.println(numPoints);
  SerialDebugger.println(voltage_step);

  //xxxxxxxxxxxxxxxxxxxxxxxxxx

  initLMP(LMPgain);
  setLMPBias(startV);
  setVoltage(startV);

  //xxxxxxxxxxxxxxxxxxxxxxxxxx
  if (print_output_to_serial)
  {
    SerialDebugger.println("RUNNING TEST IV:");
    SerialDebugger.println(a_coeff);
    SerialDebugger.println(b_coeff);
    SerialDebugger.println(startV);
    SerialDebugger.println(endV);
    SerialDebugger.println(numPoints);
    SerialDebugger.println(delayTime_ms);
    SerialDebugger.println(voltage_step);

    SerialDebugger.println("Column header meanings:");
    SerialDebugger.println("BIAS:");
    SerialDebugger.println("T = time in ms");
    SerialDebugger.println("Vc = desired cell voltage in mV (internal variable name = voltage)");
    SerialDebugger.println("Vset = set cell voltage in mV (internal variable name = vset = dacVout * TIA_BIAS[bias_setting])");
    SerialDebugger.println("div = percentage scale (internal variable name = TIA_BIAS[bias_setting])");
    SerialDebugger.println("Vdac = ESP32 dac voltage in mV (internal variable name = dacVout)");
    SerialDebugger.println("adcbits = analogread(LMP)");
    SerialDebugger.println("Vout = TIA output in mV (internal variable name = v1 = pStat.getVoltage(analogRead(LMP), opVolt, adcBits);");
    SerialDebugger.println("Vc1 = internal zero in mV also C1 (internal variable name = v2 = dacVout * .5)");
    SerialDebugger.println("i_f = forward current in uA (internal variable name = current = (((v1 - v2) / 1000) / TIA_GAIN[LMPgain - 1]) * pow(10, 6); //scales to uA)");

    SerialDebugger.println("xyz = xyz in xyz (internal variable name = xyz)");
    SerialDebugger.println("xyz = xyz in xyz (internal variable name = v)");

    //    SerialDebugger.println("T\tVc\tVset\tdiv\tVdac\tadcbits\tVout\tVc1\ti_f");
    SerialDebugger.println("T,Vc,Vset,div,Vdac,adcbits,Vout,Vc1,i_f");
  }

  for (int16_t this_voltage = startV; this_voltage <= endV; this_voltage += voltage_step)
  {
    // SerialDebugger.println(this_voltage);
    // SerialDebugger.println(delayTime_ms);
    i_forward = biasAndSample(this_voltage, delayTime_ms);

    saveVoltammogram(this_voltage, i_forward, false);
    // saveVoltammogram(this_voltage, i_forward, true);
    //i_forward = biasAndSample(startV, delayTime_ms);
    if (print_output_to_serial)
    {
      SerialDebugger.println("EOL");
    }
    if (userpause) //will hold the code here until a character is sent over the SerialDebugger port
    {
      while (!SerialDebugger.available())
        ;
      SerialDebugger.read();
    }
  }
  setOutputsToZero();
}

void testNoiseAtABiasPoint(int16_t biasV, int16_t numPoints,
                           uint32_t delayTime_ms)
{

  // sets call voltage to biasV using  setVoltage(biasV) only at beginning
  // then "reads" ADC numPoints times using analog_read_avg(num_readings_to_average, LMP); where each reading is an average
  // gives std_dev
  // internally iterates averages per reading to get idea how it effects noise...
  // dumps output to serial

  //Reset Arrays
  for (uint16_t i = 0; i < arr_samples; i++)
    volts[i] = 0;
  for (uint16_t i = 0; i < arr_samples; i++)
    output_time[i] = 0;
  for (uint16_t i = 0; i < arr_samples; i++)
    amps[i] = 0;

  number_of_valid_points_in_volts_amps_array = 0;
  arr_cur_index = 0;

  float adc_bits_array[numPoints];
  float adc_bits_array_sum = 0;
  float adc_bits_array_avg = 0;
  float adc_bits_array_std_dev = 0;
  float adc_bits_minus_avg_squared_sum = 0;
  int num_readings_to_average = 1;

  float num_points_to_average[7] = {1, 5, 10, 50, 100, 500, 1000};

  //xxxxxxxxxxxxxxxxxxxxxxxxxx
  initLMP(LMPgain);
  setLMPBias(biasV);
  setVoltage(biasV);
  //xxxxxxxxxxxxxxxxxxxxxxxxxx

  SerialDebugger.print("num_rdgs");
  SerialDebugger.print(F("\t"));

  SerialDebugger.print("adc_avg");
  SerialDebugger.print(F("\t"));

  SerialDebugger.println("adc_std_dev");

  //  for (int16_t i = 1; i < 1000; i += 1)
  for (int16_t i = 0; i < 7; i += 1)
  {
    adc_bits_array_sum = 0;
    adc_bits_array_avg = 0;
    adc_bits_array_std_dev = 0;
    adc_bits_minus_avg_squared_sum = 0;
    num_readings_to_average = num_points_to_average[i];
    //num_readings_to_average = i;
    delay(delayTime_ms);
    for (int16_t j = 0; j < numPoints; j += 1) // read the adc data
    {
      pulseLED_on_off(LEDPIN, 10);
      adc_bits_array[j] = analog_read_avg(num_readings_to_average, LMP);
      //pulseLED_on_off(LEDPIN, 10);
      // Now populate Volts array etc:
      volts[arr_cur_index] = biasV;
      output_time[arr_cur_index] = millis();
      amps[arr_cur_index] = adc_bits_array[j];
      arr_cur_index ++;
      number_of_valid_points_in_volts_amps_array++;
    }
    for (int16_t j = 0; j < numPoints; j += 1) // calculate the average
    {
      adc_bits_array_sum += adc_bits_array[j];
    }
    adc_bits_array_avg = adc_bits_array_sum / numPoints;
    for (int16_t j = 0; j < numPoints; j += 1) // calculate the std deviation
    {
      adc_bits_minus_avg_squared_sum += (adc_bits_array_avg - adc_bits_array[j]) * (adc_bits_array_avg - adc_bits_array[j]);
    }
    adc_bits_array_std_dev = sqrt(adc_bits_minus_avg_squared_sum / numPoints);

    SerialDebugger.print(num_readings_to_average);
    SerialDebugger.print(F("\t"));

    SerialDebugger.print(adc_bits_array_avg);
    SerialDebugger.print(F("\t"));

    SerialDebugger.println(adc_bits_array_std_dev);
  }
}

void testDACs(uint32_t delayTime_ms)
{

  // steps ESP32 DAC from 0 to 255 digital so you can check it on a multimeter

  //xxxxxxxxxxxxxxxxxxxxxxxxxx

  initLMP(LMPgain);
  setLMPBias(100);
  setVoltage(100);

  //xxxxxxxxxxxxxxxxxxxxxxxxxx
  SerialDebugger.println("RUNNING TEST DAC:");

  SerialDebugger.println("i\tdacgoal");

  float dacgoal;
  for (int16_t j = 0; j <= 255; j += 1)
  {
    dacWrite(dac, j); // sets ESP32 DAC bits
    dacgoal = 3300.0 * j / 255;
    SerialDebugger.print(j); // current time in ms
    SerialDebugger.print(F("\t"));
    SerialDebugger.println(dacgoal); // desired cell voltage
    if (userpause)                   //will hold the code here until a character is sent over the SerialDebugger port
    {
      while (!SerialDebugger.available())
        ;
      SerialDebugger.read();
    }
  }
  setOutputsToZero();
}

void testDACandADCs(uint32_t delayTime_ms)
{

  // steps ESP32 DAC from 0 to 255 digital so you can check it on a multimeter
  // measures with ADC
  // prints to serial
  float adc_voltage;

  //xxxxxxxxxxxxxxxxxxxxxxxxxx

  initLMP(LMPgain);
  setLMPBias(100);
  setVoltage(100);

  //xxxxxxxxxxxxxxxxxxxxxxxxxx
  SerialDebugger.println("RUNNING TEST DAC:");

  SerialDebugger.println("i\tdacgoal\tadc_voltage xyz");

  float dacgoal;
  for (int16_t j = 0; j <= 255; j += 1)
  {
    pulseLED_on_off(LEDPIN, 10); // signify start of a new data point
    // dacWrite(dac, j); // sets ESP32 DAC bits
    dacWrite(dac, j); // sets ESP32 DAC bits
    // dacgoal = 3300.0 * j / 255;
    dacgoal = 3300.0 * 136 / 255;
    delay(delayTime_ms);
    // read ADC....
    adc_voltage = analogRead(LMP);
    SerialDebugger.print(j); // current time in ms
    SerialDebugger.print(F("\t"));
    SerialDebugger.print(dacgoal); // desired cell voltage
    SerialDebugger.print(F("\t"));
    SerialDebugger.println(adc_voltage); // adc voltage

    if (userpause) //will hold the code here until a character is sent over the SerialDebugger port
    {
      while (!SerialDebugger.available())
        ;
      SerialDebugger.read();
    }
  }
  setOutputsToZero();
}

void calibrateDACandADCs(uint32_t delayTime_ms)
{

  // Sets LMP91000 gain to 0%, so at Vout, voltage is always 0.5 * Vref. (Assumes WE floating so no WE current!)
  // Stores results (slope/offset calibration points) in global variables    a_coeff, b_coeff
  // Note probably need to do it for each TIA gain setting, hard coded to 7 ie. RTIA=350kohm for now.
  // Vout=DAC, Vref=ADC
  // Run DAC from 1.5 to 3 volts (in digital units with 8 bits, 116 to 255 bit)
  // Read ADC over same range. Have ADC calibratoin for DAC range.
  // Will use to correct as follows:
  // Fit ADC/2 vs DAC and return call coefficients....
  // To run: 1) Disconnect WE from RE/CE; 2) Run calibrateDACandADCs; 3) Reconnect; 4) Run testIV (biasandsample uses coefficients)
  // Temporarily, click CV button on website for step 2 and NPV for step 4 (hardwired, later to fix UI for this...)

  float dacgoal;
  int this_voltage_adc;

  //xxxxxxxxxxxxxxxxxxxxxxxxxx
  pStat.disableFET();
  pStat.setGain(LMPgain); // TIA feedback resistor , global variable
  pStat.setRLoad(0);
  pStat.setExtRefSource();
  pStat.setIntZ(1);
  pStat.setThreeLead();
  pStat.setBias(0);
  pStat.setNegBias();
  pStat.setBias(0); // sets percentage voltage divider in LMP910000
                    // TIA_BIAS[bias_setting] varies from 1% to 22% depending on setting; note initially set to 0%...
                    // from LMP91000.h:
                    //const double TIA_BIAS[] = {0, 0.01, 0.02, 0.04, 0.06, 0.08, 0.1, 0.12, 0.14,
                    //    0.16, 0.18, 0.2, 0.22, 0.24};
                    //xxxxxxxxxxxxxxxxxxxxxxxxxx

  //xxxxxxxxxxxxxxxxxxxxxxxxxx
  SerialDebugger.println("RUNNING TEST ADC CAL LMP91000:");
  SerialDebugger.println("TIA_BIAS[0]=");
  SerialDebugger.println(TIA_BIAS[0]);

  int adc_int_from_dac_int[140]; // for dac 116 to 255, what adc reads
  // assuming cell is open, no current, ADC is reading vout which is also WE voltage if no current
  // WE voltage is supposed to be 0.5* Vref, and Vref is DAC

  float sumonx = 0;
  float sumony = 0;
  float sumonxy = 0;
  float sumonxsquared = 0;
  float sumonysquared = 0;
  float n = 0;
  float a = 0;
  float b = 0;

  for (int16_t j = 116; j <= 255; j += 1) // want to go from 1.5 to 3.3 V
  // 1.5 V is (1.5/3.3)*255 = 115.909 = 116 for 8 bit dac
  {
    dacWrite(dac, j); // sets ESP32 DAC bits
    if (j == 116)
    {
      delay(250);
    }
    // read analog voltage which should be half of dac voltage...
    this_voltage_adc = analogRead(LMP);
    adc_int_from_dac_int[j - 116] = this_voltage_adc;
    dacgoal = 3300.0 * j / 255;
    SerialDebugger.print(j);
    SerialDebugger.print(F("\t"));
    SerialDebugger.print(dacgoal);
    SerialDebugger.print(F("\t"));
    SerialDebugger.println(this_voltage_adc); // desired cell voltage
    if (userpause)                            //will hold the code here until a character is sent over the SerialDebugger port
    {
      while (!SerialDebugger.available())
        ;
      SerialDebugger.read();
    }
    // now fill in statistics:
    sumonx += j;
    sumony += this_voltage_adc;
    sumonxy += j * this_voltage_adc;
    sumonxsquared += j * j;
    sumonysquared += this_voltage_adc * this_voltage_adc;
    n += 1;
  }

  // now I want to fit a line to adc_int_from_dac_int[i] vs i....
  // https://www.statisticshowto.com/probability-and-statistics/regression-analysis/find-a-linear-regression-equation/
  // y=a+bx
  // y=adc
  // x=dac
  // sumonx=
  // sumony=
  // sumonxy=
  // sumonxsquared=
  // sumonysquared=
  // n=
  // a= ((sumony)*(sumonxsquared)-(sumonx)*(sumonxy))/(n*(sumonxsquared)-(sumonx)*(sumonx))
  // b= (n*(sumonxy) - (sumonx)*(sumony))/(n*(sumonxsquared)-(sumonx)*(sumonx))
  a = ((sumony) * (sumonxsquared) - (sumonx) * (sumonxy)) / (n * (sumonxsquared) - (sumonx) * (sumonx));
  b = (n * (sumonxy) - (sumonx) * (sumony)) / (n * (sumonxsquared) - (sumonx) * (sumonx));
  SerialDebugger.println("y=a + bx");
  SerialDebugger.println("y=dac");
  SerialDebugger.println("y=adc");
  SerialDebugger.print("a=");
  SerialDebugger.println(a);
  SerialDebugger.print("b=");
  SerialDebugger.println(b);
  // so adc = a + b * dac
  // dac = bit # (0-255)
  // adc = bit # (0-4095)
  // so for calculation of actual voltage from adc, we can assume the calibration (taking dac as perfect)
  // dacvoltage = (dac/255)*3.3
  // adcvoltage = 0.5*dacvoltage
  // adc = a + b * dac = a + b * dacvoltage * (255/3.3); here dac voltage is assumed correct
  // so inverting, we get dacvoltage = (adc-a)*(3.3/255)*(1/b)
  // dacvoltage = ((3.3/255)*(1/b)) * adc + ((-a/b)*(3.3/255))
  // adcvoltage = ((3.3/255)*(1/2*b)) * adc + ((-a/2*b)*(3.3/255))

  a_coeff = a;
  b_coeff = b;
}

void testNOISE(int num_points)
{
  // reads adc and gives noise (std deviation, average)
  // asumes adc pin is LMP (global variable)
  // note: LMP91000 not effected by this, so make sure it is properly initiallized if you are measuring its output
  // otherwise you can just disconnect it if circuit allows and put any external voltage into the ADC pin you want...
  int timer_total;
  int adc_array[num_points];
  float time_microS_of_read_array[num_points];
  float avg_of_adc_array = 0;
  float std_dev_of_adc_array = 0;

  float avg_of_adc_array_mV = 0;
  float std_dev_of_adc_array_mV = 0;

  float avg_of_time_microS_per_point = 0;
  float std_dev_time_microS_per_point = 0;
  float sum_of_adc_array = 0;
  float sum_of_time_microS_of_read_array = 0;
  float sum_of_adc_array_minus_avg_squared = 0;
  float sum_of_time_microS_of_read_array_minus_avg_squared = 0;
  int timer;
  float total_measurement_time_micros = 0;
  float total_function_time_micros = 0;

  timer_total = micros();

  for (int16_t j = 0; j <= num_points; j += 1)
  {
    timer = micros();
    adc_array[j] = analogRead(LMP);
    time_microS_of_read_array[j] = micros() - timer;
  }

  total_measurement_time_micros = micros() - timer_total;

  for (int16_t j = 0; j <= num_points; j += 1)
  {
    sum_of_adc_array += adc_array[j];
    sum_of_time_microS_of_read_array += time_microS_of_read_array[j];
    //    SerialDebugger.print("j=");
    //    SerialDebugger.print(F("\t"));
    //    SerialDebugger.print(j);
    //    SerialDebugger.print(F("\t"));
    //    SerialDebugger.print("time_microS_of_read_array[j]=");
    //    SerialDebugger.print(F("\t"));
    //    SerialDebugger.print(time_microS_of_read_array[j]);
    //    SerialDebugger.print(F("\t"));
    //    SerialDebugger.print("sum_of_time_microS_of_read_array=");
    //    SerialDebugger.print(F("\t"));
    //   SerialDebugger.println(sum_of_time_microS_of_read_array);
  }
  avg_of_adc_array = sum_of_adc_array / num_points;
  avg_of_time_microS_per_point = sum_of_time_microS_of_read_array / num_points;

  for (int16_t j = 0; j <= num_points; j += 1)
  {
    sum_of_adc_array_minus_avg_squared = (avg_of_adc_array - adc_array[j]) * (avg_of_adc_array - adc_array[j]);
    sum_of_time_microS_of_read_array_minus_avg_squared = (avg_of_time_microS_per_point - time_microS_of_read_array[j]) * (avg_of_time_microS_per_point - time_microS_of_read_array[j]);
  }
  std_dev_of_adc_array = sqrt(sum_of_adc_array_minus_avg_squared / num_points);
  std_dev_time_microS_per_point = sqrt(sum_of_time_microS_of_read_array_minus_avg_squared / num_points);

  avg_of_adc_array_mV = (3300.0 / 4095.0) * avg_of_adc_array;
  std_dev_of_adc_array_mV = (3300.0 / 4095.0) * std_dev_of_adc_array;

  total_function_time_micros = micros() - timer_total;

  SerialDebugger.println("****************************************");

  SerialDebugger.print("total_measurement_time_micros in milliseconds");
  SerialDebugger.print(F("\t"));
  SerialDebugger.println(total_measurement_time_micros / 1e3);

  SerialDebugger.print("total_function_time_micros in milliseconds");
  SerialDebugger.print(F("\t"));
  SerialDebugger.println(total_function_time_micros / 1e3);

  SerialDebugger.print("avg_of_adc_array_mV = ");
  SerialDebugger.print(F("\t"));
  SerialDebugger.println(avg_of_adc_array_mV);

  SerialDebugger.print("std_dev_of_adc_array_mV = ");
  SerialDebugger.print(F("\t"));
  SerialDebugger.println(std_dev_of_adc_array_mV);

  SerialDebugger.print("ADC average = ");
  SerialDebugger.print(F("\t"));
  SerialDebugger.println(avg_of_adc_array);

  SerialDebugger.print("std_dev_of_adc_array = ");
  SerialDebugger.print(F("\t"));
  SerialDebugger.println(std_dev_of_adc_array);

  SerialDebugger.print("avg_of_time_microS_per_point = ");
  SerialDebugger.print(F("\t"));
  SerialDebugger.println(avg_of_time_microS_per_point);

  SerialDebugger.print("std_dev_time_microS_per_point = ");
  SerialDebugger.print(F("\t"));
  SerialDebugger.println(std_dev_time_microS_per_point);

  SerialDebugger.println("****************************************");
}

int read_ADC_and_report_time(uint32_t delayTime_ms)
{
  // tests time to read and adc point from esp32, note internal driver settings not clear so this is beta verion of function
  // esp32 low level adc drivers are confusing (!)
  /*
  analogReadResolution(12);             // Sets the sample bits and read resolution, default is 12-bit (0 - 4095), range is 9 - 12 bits
  analogSetWidth(12);                   // Sets the sample bits and read resolution, default is 12-bit (0 - 4095), range is 9 - 12 bits
                                        //  9-bit gives an ADC range of 0-511
                                        // 10-bit gives an ADC range of 0-1023
                                        // 11-bit gives an ADC range of 0-2047
                                        // 12-bit gives an ADC range of 0-4095
  analogSetCycles(8);                   // Set number of cycles per sample, default is 8 and provides an optimal result, range is 1 - 255
  analogSetSamples(1);                  // Set number of samples in the range, default is 1, it has an effect on sensitivity has been multiplied
  analogSetClockDiv(1);                 // Set the divider for the ADC clock, default is 1, range is 1 - 255
  analogSetAttenuation(ADC_11db);       // Sets the input attenuation for ALL ADC inputs, default is ADC_11db, range is ADC_0db, ADC_2_5db, ADC_6db, ADC_11db
  analogSetPinAttenuation(VP,ADC_11db); // Sets the input attenuation, default is ADC_11db, range is ADC_0db, ADC_2_5db, ADC_6db, ADC_11db
                                        // ADC_0db provides no attenuation so IN/OUT = 1 / 1 an input of 3 volts remains at 3 volts before ADC measurement
                                        // ADC_2_5db provides an attenuation so that IN/OUT = 1 / 1.34 an input of 3 volts is reduced to 2.238 volts before ADC measurement
                                        // ADC_6db provides an attenuation so that IN/OUT = 1 / 2 an input of 3 volts is reduced to 1.500 volts before ADC measurement
                                        // ADC_11db provides an attenuation so that IN/OUT = 1 / 3.6 an input of 3 volts is reduced to 0.833 volts before ADC measurement
  adcAttachPin(VP);                     // Attach a pin to ADC (also clears any other analog mode that could be on), returns TRUE/FALSE result 
  adcStart(VP);                         // Starts an ADC conversion on attached pin's bus
  adcBusy(VP);                          // Check if conversion on the pin's ADC bus is currently running, returns TRUE/FALSE result 
  adcEnd(VP);                           // Get the result of the conversion (will wait if it have not finished), returns 16-bit integer result
  */
  analogSetClockDiv(255); // 1338mS

  int analogRead_result;
  int timer = micros();
  analogRead_result = analogRead(LMP);
  Serial.print(analogRead_result);
  Serial.print("  ");
  Serial.println(micros() - timer);
  return analogRead_result;

  // See: https://github.com/espressif/arduino-esp32/blob/master/cores/esp32/esp32-hal-adc.h
}

void testLMP91000(uint32_t delayTime_ms, uint8_t bias_setting_local)
{

  // PJB 4/12/2021
  // The purpose of this subroutine is to set the LMP91000 settings over the I2C
  // so that the user can manually apply an input (control)  voltage and
  // manually read the output voltages in a test breadboad.
  // It assumes the user has access to all LMP91000 pins on a breakout board of some kind, and
  // that the ESP32 controls the LMP910000 over the I2C interface.
  // For example, the ESP32 could be an ESP32 Pico Kit board, or any ESP32 board, with I2C
  // wired manually to the LMP91000.
  // P Burke made a breakout board of LMP91000 chips.

  // Parameters the user can set on LMP91000:
  // TIA gain (keep max at 350k feedback resistor)
  // Rload (keep at 10 ohms)
  // Ref source int/ext (keep ext)
  // Int_Z zero 50 20 67% (keep at 50%)
  // Bias_Sign (plus/minus)
  // Bias 1%-24%
  // FET_Short (keep off)
  // Mode (keep at 3-lead)

  bool loop_dac = false;
  float dacgoal;
  //xxxxxxxxxxxxxxxxxxxxxxxxxx
  pStat.disableFET();
  pStat.setGain(7); // TIA feedback resistor 350k
  pStat.setRLoad(0);
  pStat.setExtRefSource();
  pStat.setIntZ(1);
  pStat.setThreeLead();
  pStat.setBias(0);
  pStat.setNegBias();
  pStat.setBias(bias_setting_local); // sets percentage voltage divider in LMP910000
  // TIA_BIAS[bias_setting] varies from 1% to 22% depending on setting; note initially set to 0%...
  // from LMP91000.h:
  //const double TIA_BIAS[] = {0, 0.01, 0.02, 0.04, 0.06, 0.08, 0.1, 0.12, 0.14,
  //    0.16, 0.18, 0.2, 0.22, 0.24};
  //xxxxxxxxxxxxxxxxxxxxxxxxxx

  SerialDebugger.println("RUNNING TEST LMP91000:");
  SerialDebugger.println("TIA_BIAS[bias_setting_local]=");
  SerialDebugger.println(TIA_BIAS[bias_setting_local]);

  for (int16_t i = 0; i <= 13; i += 1) //loop over percentage of Vref applied to cell
  {
    pStat.setBias(i);
    SerialDebugger.println("****************************************");
    SerialDebugger.println("TIA_BIAS[i]=");
    SerialDebugger.println(TIA_BIAS[i]);
    if (userpause) //will hold the code here until a character is sent over the SerialDebugger port
    {
      while (!SerialDebugger.available())
        ;
      SerialDebugger.read();
    }
    if (loop_dac)
    {
      for (int16_t j = 0; j <= 255; j += 1)
      {
        dacWrite(dac, j); // sets ESP32 DAC bits
        dacgoal = 3300.0 * j / 255;
        SerialDebugger.print(j); // current time in ms
        SerialDebugger.print(F("\t"));
        SerialDebugger.println(dacgoal); // desired cell voltage
        if (userpause)                   //will hold the code here until a character is sent over the SerialDebugger port
        {
          while (!SerialDebugger.available())
            ;
          SerialDebugger.read();
        }
      }
    }
  }
}

void runNPVForward(int16_t startV, int16_t endV, int8_t pulseAmp,
                   uint32_t pulse_width, uint32_t off_time)
{

  //@param      startV:       voltage to start the scan
  //@param      endV:         voltage to stop the scan
  //@param      pulseAmp:     amplitude of square wave
  //@param      pulse_width:  the pulse-width of the excitation voltage
  //@param      off_time:
  //
  //Runs NPV in the forward (oxidation) direction. The bias potential
  //is swept from a more negative voltage to a more positive voltage.
  float i_forward = 0;
  float i_backward = 0;

  for (int16_t j = startV; j <= endV; j += pulseAmp)
  {
    i_forward = biasAndSample(j, pulse_width);
    if (userpause) //will hold the code here until a character is sent over the SerialDebugger port
    {
      while (!SerialDebugger.available())
        ;
      SerialDebugger.read();
    }

    i_backward = biasAndSample(startV, off_time);      // xxx comment out to do only forward bias
    saveVoltammogram(j, i_forward - i_backward, true); // this will print voltage, current
    SerialDebugger.println();
    if (userpause) //will hold the code here until a character is sent over the SerialDebugger port
    {
      while (!SerialDebugger.available())
        ;
      SerialDebugger.read();
    }
  }
}

void runNPVBackward(int16_t startV, int16_t endV, int8_t pulseAmp,
                    uint32_t pulse_width, uint32_t off_time)
{
  //@param      startV:       voltage to start the scan
  //@param      endV:         voltage to stop the scan
  //@param      pulseAmp:     amplitude of square wave
  //@param      pulse_width:  the pulse-width of the excitation voltage
  //@param      off_time:
  //
  //Runs NPV in the reverse (reduction) direction. The bias potential
  //is swept from a more positivie voltage to a more negative voltage.

  float i_forward = 0;
  float i_backward = 0;

  for (int16_t j = startV; j >= endV; j -= pulseAmp)
  {
    i_forward = biasAndSample(j, pulse_width);
    //will hold the code here until a character is sent over the SerialDebugger port
    //this ensures the experiment will only run when initiated
    while (!SerialDebugger.available())
      ;
    SerialDebugger.read();
    i_backward = biasAndSample(startV, off_time);
    //will hold the code here until a character is sent over the SerialDebugger port
    //this ensures the experiment will only run when initiated
    while (!SerialDebugger.available())
      ;
    SerialDebugger.read();
    saveVoltammogram(j, i_forward - i_backward, true); // this will print voltage, currnt
    SerialDebugger.println();
  }
}

void runNPV(uint8_t lmpGain, int16_t startV, int16_t endV,
            int8_t pulseAmp, uint32_t pulse_width, uint32_t pulse_period,
            uint32_t quietTime, uint8_t range, bool setToZero)
{
  //@param      lmpGain:      gain setting for LMP91000
  //@param      startV:       voltage to start the scan
  //@param      endV:         voltage to stop the scan
  //@param      pulseAmp:     amplitude of square wave
  //@param      pulse_period: the  of the excitation voltage
  //@param      pulse_width:  the pulse-width of the excitation voltage
  //@param      quietTime:    initial wait time before the first pulse
  //@param      range:        the expected range of the measured current
  //@param      setToZero:    Boolean determining whether the bias potential of
  //                          the electrochemical cell should be set to 0 at the
  //                          end of the experiment.
  //
  //Runs the electrochemical technique, Normal Pulse Voltammetry. In this
  //technique the electrochemical cell is biased at increasing superimposed
  //voltages. The current is sampled at the end of each step potential. The
  //potential is returned to the startV at the end of each pulse period.
  //https://www.basinc.com/manuals/EC_epsilon/techniques/Pulse/pulse#normal
  //Reset Arrays
  for (uint16_t i = 0; i < arr_samples; i++)
    volts[i] = 0;
  for (uint16_t i = 0; i < arr_samples; i++)
    amps[i] = 0;

  arr_cur_index = 0;
  number_of_valid_points_in_volts_amps_array = 0;

  if (pulse_width > pulse_period)
  {
    uint32_t temp = pulse_width;
    pulse_width = pulse_period;
    pulse_period = temp;
  }

  if (print_output_to_serial)
  {

    //Print column headers
    //  String current = "";
    //  if(range == 12) current = "Current(pA)";
    //  else if(range == 9) current = "Current(nA)";
    //  else if(range == 6) current = "Current(uA)";
    //  else if(range == 3) current = "Current(mA)";
    //  else current = "SOME ERROR";

    // TIA_BIAS[bias_setting]
    // dacVout*TIA_BIAS[bias_setting] is the exact voltage we will get

    SerialDebugger.println("Column header meanings:");
    SerialDebugger.println("FORWARD BIAS:");
    SerialDebugger.println("T = time in ms");
    SerialDebugger.println("Vc = desired cell voltage in mV (internal variable name = voltage)");
    SerialDebugger.println("Vset = set cell voltage in mV (internal variable name = vset = dacVout * TIA_BIAS[bias_setting])");
    SerialDebugger.println("div = percentage scale (internal variable name = TIA_BIAS[bias_setting])");
    SerialDebugger.println("Vdac = ESP32 dac voltage in mV (internal variable name = dacVout)");
    SerialDebugger.println("adc_bits = TIA output in adc_bits;");
    SerialDebugger.println("(V1) Vout = TIA output in mV (internal variable name = v1 = pStat.getVoltage(analogRead(LMP), opVolt, adcBits);");
    SerialDebugger.println("(V2) Vc1 = internal zero in mV also C1 (internal variable name = v2 = dacVout * .5)");
    SerialDebugger.println("i_f = forward current in uA (internal variable name = current = (((v1 - v2) / 1000) / TIA_GAIN[LMPgain - 1]) * pow(10, 6); //scales to uA)");

    SerialDebugger.println("REVERSE BIAS:");
    SerialDebugger.println("T = time in ms");
    SerialDebugger.println("Vc = desired cell voltage in mV (internal variable name = voltage)");
    SerialDebugger.println("Vset = set cell voltage in mV (internal variable name = vset = dacVout * TIA_BIAS[bias_setting])");
    SerialDebugger.println("div = percentage scale (internal variable name = TIA_BIAS[bias_setting])");
    SerialDebugger.println("Vdac = ESP32 dac voltage in mV (internal variable name = dacVout)");
    SerialDebugger.println("Vout = TIA output in mV (internal variable name = v1 = pStat.getVoltage(analogRead(LMP), opVolt, adcBits);");
    SerialDebugger.println("Vc1 = internal zero in mV also C1 (internal variable name = v2 = dacVout * .5)");
    SerialDebugger.println("i_R = reverse current in uA (internal variable name = current = (((v1 - v2) / 1000) / TIA_GAIN[LMPgain - 1]) * pow(10, 6); //scales to uA)");

    SerialDebugger.println("RESPONSE:");
    SerialDebugger.println("Vc = desired cell voltage in mV (internal variable name = voltage FORWARD, should be FORWARD-REVERSE...)");
    SerialDebugger.println("I = i_f - i_R in uA (internal variable name = i_forward - i_backward)");
    SerialDebugger.println("xyz = xyz in xyz (internal variable name = xyz)");
    SerialDebugger.println("xyz = xyz in xyz (internal variable name = v)");

    SerialDebugger.println("T,Vc,Vset,div,Vdac,adcbits,Vout,Vc1,i_f,T,Vc,Vset,div,Vdac,adcbits,Vout,Vc1,i_R,V,I,T");

    //    SerialDebugger.println("T\tVc\tVset\tdiv\tVdac\tVout\tVc1\ti_f\tT\tVc\tVset\tdiv\tVdac\tVout\tVc1\ti_R\tVc\tI");
    //  SerialDebugger.println("T\tVc\tVdac\tVout\tVc1\ti_f\tT\tVc\tZ\tVout\ti_R\tLMP\tI");
    //  SerialDebugger.println("T\tVc\tVout\tVc1\ti_f\tT\tVc\tZ\tVout\ti_R\tLMP\tI");
    //  SerialDebugger.println("T\t\tVc\tVout\tVc1\ti_f\tT\tVc\tZ\tVout\ti_R\tLMP\tI");
    //  SerialDebugger.println("T(ms)\tVcell(mV)\tZero(mV)\tLMP\ti_f\tT(ms)\tVcell(mV)\tZero(mV)\tLMP\ti_R\tV(mV)\tI");
    //  SerialDebugger.println(F("Time(ms),Cell set voltage (mV),Zero(mV),dacVout,TIA_BIAS[bias_setting],LMP i.e. Vout (mV),VC1(mV),i_forward,Time(ms),Cell set voltage (mV),Zero(mV),dacVout,TIA_BIAS[bias_setting],LMP i.e. Vout (mV),VC1(mV),i_backward,Voltage(mV),Current"));
    //  SerialDebugger.println(F("T(ms),Vcell(mV),Zero(mV),dacVout,TIA_BIAS,Vout(mV),VC1(mV),i_f,Time(ms),Cell set voltage (mV),Zero(mV),dacVout,TIA_BIAS[bias_setting],LMP i.e. Vout (mV),VC1(mV),i_backward,Voltage(mV),Current"));
  }

  //Reset Arrays
  for (uint16_t i = 0; i < arr_samples; i++)
    volts[i] = 0;
  for (uint16_t i = 0; i < arr_samples; i++)
    amps[i] = 0;

  initLMP(lmpGain);
  pulseAmp = abs(pulseAmp);
  uint32_t off_time = pulse_period - pulse_width;

  saveQueues = true;

  setLMPBias(startV); // -200 mV to start
  setVoltage(startV); // -200 mV to start
  // SerialDebugger.println();

  unsigned long time1 = millis();
  while (millis() - time1 < quietTime)
    ;

  if (startV < endV)
    runNPVForward(startV, endV, pulseAmp, pulse_width, off_time);
  else
    runNPVBackward(startV, endV, pulseAmp, pulse_width, off_time);

  arr_cur_index = 0;
  if (setToZero)
    setOutputsToZero();
}

//@param      cycles:     number of times to run the scan
//@param      startV:     voltage to start the scan
//@param      endV:       voltage to stop the scan
//@param      vertex1:    edge of the scan
//                        If vertex1 is greater than starting voltage, we start
//                        the experiment by running CV in the forward
//                        (oxidation) direction.
//@param      vertex2:    edge of the scan
//                        If vertex2 is greater than starting voltage, we start
//                        the experiment by running CV in the reverse
//                        (reduction) direction.
//@param      stepV:      how much to increment the voltage by
//@param      rate:       scanning rate
//                        in the case of CV, scanning rate is in mV/s
//Runs CV in the forward (oxidation) direction first
void runCVForward(uint8_t cycles, int16_t startV, int16_t endV,
                  int16_t vertex1, int16_t vertex2, int16_t stepV, uint16_t rate)
{
  int16_t j = startV;
  float i_cv = 0;
  number_of_valid_points_in_volts_amps_array = 0;

  for (uint8_t i = 0; i < cycles; i++)
  {
    if (i == cycles - 2)
      saveQueues = true;
    else
      saveQueues = false;

    //j starts at startV
    for (j; j <= vertex1; j += stepV)
    {
      i_cv = biasAndSample(j, rate);
      SerialDebugger.print(i + 1);
      SerialDebugger.print(F(","));
      saveVoltammogram(j, i_cv, true);
      SerialDebugger.println();
    }
    j -= 2 * stepV; //increment j twice to avoid biasing at the vertex twice

    //j starts right below the first vertex
    for (j; j >= vertex2; j -= stepV)
    {
      i_cv = biasAndSample(j, rate);
      SerialDebugger.print(i + 1);
      SerialDebugger.print(F(","));
      saveVoltammogram(j, i_cv, true);
      SerialDebugger.println();
    }
    j += 2 * stepV; //increment j twice to avoid biasing at the vertex twice

    //j starts right above the second vertex
    for (j; j <= endV; j += stepV)
    {
      i_cv = biasAndSample(j, rate);
      SerialDebugger.print(i + 1);
      SerialDebugger.print(F(","));
      saveVoltammogram(j, i_cv, true);
      SerialDebugger.println();
    }
    j -= 2 * stepV; //increment j twice to avoid biasing at the vertex twice
  }
}

//@param      cycles:     number of times to run the scan
//@param      startV:     voltage to start the scan
//@param      endV:       voltage to stop the scan
//@param      vertex1:    edge of the scan
//                        If vertex1 is greater than starting voltage, we start
//                        the experiment by running CV in the forward
//                        (oxidation) direction.
//@param      vertex2:    edge of the scan
//                        If vertex2 is greater than starting voltage, we start
//                        the experiment by running CV in the reverse
//                        (reduction) direction.
//@param      stepV:      how much to increment the voltage by
//@param      rate:       scanning rate
//                        in the case of CV, scanning rate is in mV/s
//Runs CV in the reverse (reduction) direction first
void runCVBackward(uint8_t cycles, int16_t startV, int16_t endV,
                   int16_t vertex1, int16_t vertex2, int16_t stepV, uint16_t rate)
{
  int16_t j = startV;
  float i_cv = 0;

  for (uint8_t i = 0; i < cycles; i++)
  {
    if (i == cycles - 2)
      saveQueues = true;
    else
      saveQueues = false;

    //j starts at startV
    for (j; j >= vertex1; j -= stepV)
    {
      i_cv = biasAndSample(j, rate);
      SerialDebugger.print(i + 1);
      SerialDebugger.print(F(","));
      saveVoltammogram(j, i_cv, true);
      SerialDebugger.println();
    }
    j += 2 * stepV; //increment j twice to avoid biasing at the vertex twice

    //j starts right above vertex1
    for (j; j <= vertex2; j += stepV)
    {
      i_cv = biasAndSample(j, rate);
      SerialDebugger.print(i + 1);
      SerialDebugger.print(F(","));
      saveVoltammogram(j, i_cv, true);
      SerialDebugger.println();
    }
    j -= 2 * stepV; //increment j twice to avoid biasing at the vertex twice

    //j starts right below vertex2
    for (j; j >= endV; j -= stepV)
    {
      i_cv = biasAndSample(j, rate);
      SerialDebugger.print(i + 1);
      SerialDebugger.print(F(","));
      saveVoltammogram(j, i_cv, true);
      SerialDebugger.println();
    }
    j += 2 * stepV; //increment j twice to avoid biasing at the vertex twice
  }
}

//void runCV()
//
//@param      lmpGain:    gain setting for LMP91000
//@param      cycles:     number of times to run the scan
//@param      startV:     voltage to start the scan
//@param      endV:       voltage to stop the scan
//@param      vertex1:    edge of the scan
//                        If vertex1 is greater than starting voltage, we start
//                        the experiment by running CV in the forward
//                        (oxidation) direction.
//@param      vertex2:    edge of the scan
//                        If vertex2 is greater than starting voltage, we start
//                        the experiment by running CV in the reverse
//                        (reduction) direction.
//@param      stepV:      how much to increment the voltage by
//@param      rate:       scanning rate
//                        in the case of CV, scanning rate is in mV/s
//@param      setToZero:  Boolean determining whether the bias potential of
//                        the electrochemical cell should be set to 0 at the
//                        end of the experiment.
//
//Runs the electrochemical technique, Cyclic Voltammetry. In this
//technique the electrochemical cell is biased at a series of
//voltages and the current at each subsequent voltage is measured.
void runCV(uint8_t lmpGain, uint8_t cycles, int16_t startV,
           int16_t endV, int16_t vertex1, int16_t vertex2,
           int16_t stepV, uint16_t rate, bool setToZero)
{
  SerialDebugger.println(F("Time(ms),Zero,LMP,Current,Cycle,Voltage,Current"));

  initLMP(lmpGain);
  //the method automatically handles if stepV needs to be positive or negative
  //no need for the user to specify one or the other
  //this step deals with that in case the user doesn't know
  stepV = abs(stepV);
  //figures out the delay needed to achieve a given scan rate
  //delay is dependent on desired rate and number of steps taken
  //more steps = smaller delay since we'll need to go by a bit faster
  //to sample at more steps vs. less steps, but at the same rate
  rate = (1000.0 * stepV) / rate;
  // e.g. rate = 100 mV/s step = 5 mV
  // so 100/5 steps per second or 5/100 seconds between steps=50 msec
  // so new "rate" is delay in ms between points (assuming read is instantaneous)

  //Reset Arrays
  for (uint16_t i = 0; i < arr_samples; i++)
    volts[i] = 0;
  for (uint16_t i = 0; i < arr_samples; i++)
    amps[i] = 0;

  lastTime = millis();
  if (vertex1 > startV)
    runCVForward(cycles, startV, endV, vertex1, vertex2, stepV, rate);
  else
    runCVBackward(cycles, startV, endV, vertex1, vertex2, stepV, rate);

  //sets the bias of the electrochemical cell to 0V
  if (setToZero)
    setOutputsToZero();

  arr_cur_index = 0;
}

//void runSWVForward
//
//@param      startV:     voltage to start the scan
//@param      endV:       voltage to stop the scan
//@param      pulseAmp:   amplitude of square wave
//@param      stepV:      how much to increment the voltage by
//@param      freq:       square wave frequency
//
//Runs SWV in the forward (oxidation) direction. The bias potential is
//swept from a more negative voltage to a more positive voltage.
void runSWVForward(int16_t startV, int16_t endV, int16_t pulseAmp,
                   int16_t stepV, double freq)
{
  float i_forward = 0;
  float i_backward = 0;

  for (int16_t j = startV; j <= endV; j += stepV)
  {
    //positive pulse
    i_forward = biasAndSample(j + pulseAmp, freq);

    //negative pulse
    i_backward = biasAndSample(j - pulseAmp, freq);

    saveVoltammogram(j, i_forward - i_backward, true);
    SerialDebugger.println();
  }
}

//void runSWVBackward
//
//@param      startV:     voltage to start the scan
//@param      endV:       voltage to stop the scan
//@param      pulseAmp:   amplitude of square wave
//@param      stepV:      how much to increment the voltage by
//@param      freq:       square wave frequency
//
//Runs SWV in the backward (reduction) direction. The bias potential
//is swept from a more positivie voltage to a more negative voltage.
void runSWVBackward(int16_t startV, int16_t endV, int16_t pulseAmp,
                    int16_t stepV, double freq)
{
  float i_forward = 0;
  float i_backward = 0;

  for (int16_t j = startV; j >= endV; j -= stepV)
  {
    //positive pulse
    i_forward = biasAndSample(j + pulseAmp, freq);

    //negative pulse
    i_backward = biasAndSample(j - pulseAmp, freq);

    saveVoltammogram(j, i_forward - i_backward, true);
    SerialDebugger.println();
  }
}

//void runSWV()
//
//@param      lmpGain:    gain setting for LMP91000
//@param      startV:     voltage to start the scan
//@param      endV:       voltage to stop the scan
//@param      pulseAmp:   amplitude of square wave
//@param      stepV:      how much to increment the voltage by
//@param      freq:       square wave frequency
//@param      setToZero:  Boolean determining whether the bias potential of
//                        the electrochemical cell should be set to 0 at the
//                        end of the experiment.
//
//Runs the electrochemical technique, Square Wave Voltammetry. In this
//technique the electrochemical cell is biased at a series of
//voltages with a superimposed squar wave on top of the bias voltage.
//The current is sampled on the forward and the reverse pulse.
//https://www.basinc.com/manuals/EC_epsilon/Techniques/Pulse/pulse#square
//
void runSWV(uint8_t lmpGain, int16_t startV, int16_t endV,
            int16_t pulseAmp, int16_t stepV, double freq, bool setToZero)
{
  SerialDebugger.println(F("Time(ms),Zero,LMP,Time(ms),Zero,LMP,Voltage,Current"));

  initLMP(lmpGain);
  stepV = abs(stepV);
  pulseAmp = abs(pulseAmp);
  freq = (uint16_t)(1000.0 / (2 * freq)); //converts frequency to milliseconds

  //Reset Arrays
  for (uint16_t i = 0; i < arr_samples; i++)
    volts[i] = 0;
  for (uint16_t i = 0; i < arr_samples; i++)
    amps[i] = 0;

  arr_cur_index = 0;
  number_of_valid_points_in_volts_amps_array = 0;

  //testing shows that time is usually off by 1 ms or so, therefore
  //we subtract 1 ms from the calculated rate to compensate
  freq -= 1;

  saveQueues = true;
  if (print_output_to_serial)
  {

    //Print column headers
    //  String current = "";
    //  if(range == 12) current = "Current(pA)";
    //  else if(range == 9) current = "Current(nA)";
    //  else if(range == 6) current = "Current(uA)";
    //  else if(range == 3) current = "Current(mA)";
    //  else current = "SOME ERROR";

    // TIA_BIAS[bias_setting]
    // dacVout*TIA_BIAS[bias_setting] is the exact voltage we will get

    SerialDebugger.println("Column header meanings:");
    SerialDebugger.println("FORWARD BIAS:");
    SerialDebugger.println("T = time in ms");
    SerialDebugger.println("Vc = desired cell voltage in mV (internal variable name = voltage)");
    SerialDebugger.println("Vset = set cell voltage in mV (internal variable name = vset = dacVout * TIA_BIAS[bias_setting])");
    SerialDebugger.println("div = percentage scale (internal variable name = TIA_BIAS[bias_setting])");
    SerialDebugger.println("Vdac = ESP32 dac voltage in mV (internal variable name = dacVout)");
    SerialDebugger.println("adc_bits = TIA output in adc_bits;");
    SerialDebugger.println("(V1) Vout = TIA output in mV (internal variable name = v1 = pStat.getVoltage(analogRead(LMP), opVolt, adcBits);");
    SerialDebugger.println("(V2) Vc1 = internal zero in mV also C1 (internal variable name = v2 = dacVout * .5)");
    SerialDebugger.println("i_f = forward current in uA (internal variable name = current = (((v1 - v2) / 1000) / TIA_GAIN[LMPgain - 1]) * pow(10, 6); //scales to uA)");

    SerialDebugger.println("REVERSE BIAS:");
    SerialDebugger.println("T = time in ms");
    SerialDebugger.println("Vc = desired cell voltage in mV (internal variable name = voltage)");
    SerialDebugger.println("Vset = set cell voltage in mV (internal variable name = vset = dacVout * TIA_BIAS[bias_setting])");
    SerialDebugger.println("div = percentage scale (internal variable name = TIA_BIAS[bias_setting])");
    SerialDebugger.println("Vdac = ESP32 dac voltage in mV (internal variable name = dacVout)");
    SerialDebugger.println("Vout = TIA output in mV (internal variable name = v1 = pStat.getVoltage(analogRead(LMP), opVolt, adcBits);");
    SerialDebugger.println("Vc1 = internal zero in mV also C1 (internal variable name = v2 = dacVout * .5)");
    SerialDebugger.println("i_R = reverse current in uA (internal variable name = current = (((v1 - v2) / 1000) / TIA_GAIN[LMPgain - 1]) * pow(10, 6); //scales to uA)");

    SerialDebugger.println("RESPONSE:");
    SerialDebugger.println("Vc = desired cell voltage in mV (internal variable name = voltage FORWARD, should be FORWARD-REVERSE...)");
    SerialDebugger.println("I = i_f - i_R in uA (internal variable name = i_forward - i_backward)");
    SerialDebugger.println("xyz = xyz in xyz (internal variable name = xyz)");
    SerialDebugger.println("xyz = xyz in xyz (internal variable name = v)");

    SerialDebugger.println("T,Vc,Vset,div,Vdac,adcbits,Vout,Vc1,i_f,T,Vc,Vset,div,Vdac,adcbits,Vout,Vc1,i_R,V,I,T");

    //    SerialDebugger.println("T\tVc\tVset\tdiv\tVdac\tVout\tVc1\ti_f\tT\tVc\tVset\tdiv\tVdac\tVout\tVc1\ti_R\tVc\tI");
    //  SerialDebugger.println("T\tVc\tVdac\tVout\tVc1\ti_f\tT\tVc\tZ\tVout\ti_R\tLMP\tI");
    //  SerialDebugger.println("T\tVc\tVout\tVc1\ti_f\tT\tVc\tZ\tVout\ti_R\tLMP\tI");
    //  SerialDebugger.println("T\t\tVc\tVout\tVc1\ti_f\tT\tVc\tZ\tVout\ti_R\tLMP\tI");
    //  SerialDebugger.println("T(ms)\tVcell(mV)\tZero(mV)\tLMP\ti_f\tT(ms)\tVcell(mV)\tZero(mV)\tLMP\ti_R\tV(mV)\tI");
    //  SerialDebugger.println(F("Time(ms),Cell set voltage (mV),Zero(mV),dacVout,TIA_BIAS[bias_setting],LMP i.e. Vout (mV),VC1(mV),i_forward,Time(ms),Cell set voltage (mV),Zero(mV),dacVout,TIA_BIAS[bias_setting],LMP i.e. Vout (mV),VC1(mV),i_backward,Voltage(mV),Current"));
    //  SerialDebugger.println(F("T(ms),Vcell(mV),Zero(mV),dacVout,TIA_BIAS,Vout(mV),VC1(mV),i_f,Time(ms),Cell set voltage (mV),Zero(mV),dacVout,TIA_BIAS[bias_setting],LMP i.e. Vout (mV),VC1(mV),i_backward,Voltage(mV),Current"));
  }

  //Reset Arrays
  for (uint16_t i = 0; i < arr_samples; i++)
    volts[i] = 0;
  for (uint16_t i = 0; i < arr_samples; i++)
    amps[i] = 0;

  if (startV < endV)
    runSWVForward(startV, endV, pulseAmp, stepV, freq);
  else
    runSWVBackward(startV, endV, pulseAmp, stepV, freq);

  arr_cur_index = 0;
  if (setToZero)
    setOutputsToZero();
}

void runDPVForward(int16_t startV, int16_t endV, int8_t pulseAmp,
                   int8_t stepV, uint32_t pulse_width, uint32_t off_time)
{
  float i_forward = 0;
  float i_backward = 0;

  for (int16_t j = startV; j <= endV; j += stepV)
  {
    //baseline
    i_backward = biasAndSample(j, off_time);

    //pulse
    i_forward = biasAndSample(j + pulseAmp, pulse_width);

    saveVoltammogram(j, i_forward - i_backward, true);
    SerialDebugger.println();
  }
}

void runDPVBackward(int16_t startV, int16_t endV, int8_t pulseAmp,
                    int8_t stepV, uint32_t pulse_width, uint32_t off_time)
{
  float i_forward = 0;
  float i_backward = 0;

  for (int16_t j = startV; j >= endV; j -= stepV)
  {
    //baseline
    i_backward = biasAndSample(j, off_time);

    //pulse
    i_forward = biasAndSample(j + pulseAmp, pulse_width);

    saveVoltammogram(j, i_forward - i_backward, true);
    SerialDebugger.println();
  }
}

void runDPV(uint8_t lmpGain, int16_t startV, int16_t endV,
            int8_t pulseAmp, int8_t stepV, uint32_t pulse_period,
            uint32_t pulse_width, bool setToZero)
{
  //SerialDebugger.println("Time(ms),Zero(mV),LMP,Voltage(mV)," + current);
  SerialDebugger.println(F("Time(ms),Zero(mV),LMP,Voltage(mV),Current"));

  initLMP(lmpGain);
  stepV = abs(stepV);
  pulseAmp = abs(pulseAmp);
  uint32_t off_time = pulse_period - pulse_width;

  saveQueues = true;

  //Reset Arrays
  for (uint16_t i = 0; i < arr_samples; i++)
    volts[i] = 0;
  for (uint16_t i = 0; i < arr_samples; i++)
    amps[i] = 0;

  if (startV < endV)
    runDPVForward(startV, endV, pulseAmp, stepV, pulse_width, off_time);
  else
    runDPVBackward(startV, endV, pulseAmp, stepV, pulse_width, off_time);

  arr_cur_index = 0;
  if (setToZero)
    setOutputsToZero();
}

//void runAmp()
//
//@param      lmpGain:    gain setting for LMP91000
//@param      pre_stepV:  voltage to start the scan
//@param      quietTime:  initial wait time before the first pulse
//
//@param      v1:         the first step potential
//@param      t1:         how long we hold the cell at the first potential
//@param      v2:         the second step potential
//@param      t2:         how long we hold the cell at the second potential
//@param      samples:    how many times we sample the current at each potential
//@param      range:      the expected range of the measured current
//                          range = 12 is picoamperes
//                          range = 9 is nanoamperes
//                          range = 6 is microamperes
//                          range = 3 is milliamperes
//@param      setToZero:  Boolean determining whether the bias potential of
//                        the electrochemical cell should be set to 0 at the
//                        end of the experiment.
//
//Runs Amperometry technique. In Amperometry, the voltage is biased
//and held at one or maybe two potentials and the current is sampled
//while the current is held steady.
//https://www.basinc.com/manuals/EC_epsilon/Techniques/ChronoI/ca
//
void runAmp(uint8_t lmpGain, int16_t pre_stepV, uint32_t quietTime,
            int16_t v1, uint32_t t1, int16_t v2, uint32_t t2,
            uint16_t samples, uint8_t range, bool setToZero)
{
  //Print column headers
  //  String current = "";
  //  if (range == 12)
  //    current = "Current(pA)";
  //  else if (range == 9)
  //    current = "Current(nA)";
  //  else if (range == 6)
  //    current = "Current(uA)";
  //  else if (range == 3)
  //    current = "Current(mA)";
  //  else
  //    current = "SOME ERROR";

  //Reset Arrays
  for (uint16_t i = 0; i < arr_samples; i++)
    volts[i] = 0;
  for (uint16_t i = 0; i < arr_samples; i++)
    amps[i] = 0;

  arr_cur_index = 0;
  number_of_valid_points_in_volts_amps_array = 0;

  if (samples > arr_samples / 3)
    samples = arr_samples / 3;
  initLMP(lmpGain);

  int16_t voltageArray[3] = {pre_stepV, v1, v2};
  uint32_t timeArray[3] = {quietTime, t1, t2};

  //Reset Arrays
  for (uint16_t i = 0; i < arr_samples; i++)
    volts[i] = 0;
  for (uint16_t i = 0; i < arr_samples; i++)
    output_time[i] = 0;
  for (uint16_t i = 0; i < arr_samples; i++)
    amps[i] = 0;

  //i = 0 is pre-step voltage
  //i = 1 is first step potential
  //i = 2 is second step potential
  for (uint8_t i = 0; i < 3; i++)
  {
    //the time between samples is determined by the
    //number of samples inputted by the user
    uint32_t fs = (double)timeArray[i] / samples;
    unsigned long startTime = millis();

    //Print column headers
    //SerialDebugger.println("Voltage(mV),Time(ms)," + current);
    SerialDebugger.println("Voltage(mV),Time(ms), Current(microA)");

    //set bias potential
    setLMPBias(voltageArray[i]);
    setVoltage(voltageArray[i]);
    //SerialDebugger.println();

    while (millis() - startTime < timeArray[i])
    {
      //  Calibrate coefficients (make a local copy of the global ones):
      float a = a_coeff; // a is local copy of global a_coeff
      float b = b_coeff; // b is local copy of global a_coeff
      // Read output voltage of the transimpedance amplifier
      int adc_bits; // will be output voltage of TIA amplifier, "VOUT" on LMP91000 diagram, also C2
      // hard wired in BurkeLab ESP32Stat Rev 3.5 to LMP i.e ESP32 pin 32 (ADC1_CH7)
      //adc_bits = analogRead(LMP); // read a single point
      pulseLED_on_off(LEDPIN, 10); // signify start of a new data point
      number_of_valid_points_in_volts_amps_array++;
      adc_bits = analog_read_avg(num_adc_readings_to_average, LMP); // read a number of points and average them...
      float v1;
      v1 = (3.3 / 255.0) * (1 / (2.0 * b)) * (float)adc_bits - (a / (2.0 * b)) * (3.3 / 255.0); // LMP is wired to Vout of the LMP91000
      v1 = v1 * 1000;
      float v2 = dacVout * .5; //the zero of the internal transimpedance amplifier
      // V2 is not measured in  BurkeLab ESP32Stat Rev 3.5 and assumed to be half dacVout, calibration helps this see above
      float current = 0;

      //the current is determined by the zero of the transimpedance amplifier
      //from the output of the transimpedance amplifier, then dividing
      //by the feedback resistor
      //current = (V_OUT - V_IN-) / RFB
      //v1 and v2 are in milliVolts
      if (LMPgain == 0)                                    // using external feedback resistor, not (yet) suppored with BurkeLab ESP32Stat Rev 3.5
        current = (((v1 - v2) / 1000) / RFB) * pow(10, 9); //scales to nA
      else
        current = (((v1 - v2) / 1000) / TIA_GAIN[LMPgain - 1]) * pow(10, 6); //scales to uA
      //Sample and save data
      volts[arr_cur_index] = voltageArray[i];
      output_time[arr_cur_index] = millis();
      amps[arr_cur_index] = current;

      //Print data
      SerialDebugger.print(volts[arr_cur_index]);
      SerialDebugger.print(F(","));
      SerialDebugger.print(output_time[arr_cur_index]);
      SerialDebugger.print(F(","));
      SerialDebugger.print(amps[arr_cur_index]);
      SerialDebugger.println();

      arr_cur_index++;
      delay(fs - 1); //the -1 is for adjusting for a slight offset
    }

    SerialDebugger.println();
    SerialDebugger.println();
  }

  arr_cur_index = 0;
  if (setToZero)
    setOutputsToZero();
}

void runNPVandPrintToSerial()
// runNPVandPrintToSerial
{

  //##############################NORMAL PULSE VOLTAMMETRY##############################
  LMPgain = 7;
  runNPV(LMPgain, -200, 500, 10, 50, 200, 500, 6, true);
  // Run Normal Pulse Voltammetry with gain 350000, -200 to + 500 mV, 10 mV step, 50 microsecond width,
  //  200 microsecond period, 500 microsecond quiet time, range micro amps) // micro or milli???

  SerialDebugger.println(F("Voltage,Current")); // the final array
  for (uint16_t i = 0; i < arr_samples; i++)
  {
    SerialDebugger.print(volts[i]);
    SerialDebugger.print(F("\t"));
    SerialDebugger.println(amps[i]);
  }
}

void runCVandPrintToSerial()
// runCVandPrintToSerial
{
  //##############################CYCLIC VOLTAMMETRY##############################

  //  //void runCV(uint8_t lmpGain, uint8_t cycles, int16_t startV,
  //  //           int16_t endV, int16_t vertex1, int16_t vertex2,
  //  //           int16_t stepV, uint16_t rate, bool setToZero)
  //
  // runCV(LMPgain, 1, -100, 100, -100, 100, 1, 50, true);

  SerialDebugger.println(F("Voltage,Current")); // the final array
  for (uint16_t i = 0; i < arr_samples; i++)
  {
    SerialDebugger.print(volts[i]);
    SerialDebugger.print(F("\t"));
    SerialDebugger.println(amps[i]);
  }
}

void runSWVForwardandPrintToSerial()
// runSWVandPrintToSerial (forward)
{
  //  ##############################SQUARE WAVE VOLTAMMETRY (Forward -- Oxidation)##############################

  //  runSWV(LMPgain, -400, 500, 50, 1, 31.25, true);

  SerialDebugger.println(F("Voltage,Current"));
  for (uint16_t i = 0; i < arr_samples; i++)
  {
    SerialDebugger.print(volts[i]);
    SerialDebugger.print(F(","));
    SerialDebugger.println(amps[i]);
  }
}

void runSWVReverseandPrintToSerial()
// runCAandPrintToSerial (reverse)
{

  //  ##############################SQUARE WAVE VOLTAMMETRY (Reverse -- Reduction)##############################

  // runSWV(LMPgain, -30, -500, 50, 66, 62.5, true);
  SerialDebugger.println(F("Voltage,Current"));
  for (uint16_t i = 0; i < arr_samples; i++)
  {
    SerialDebugger.print(volts[i]);
    SerialDebugger.print(F(","));
    SerialDebugger.println(amps[i]);
  }
}
void runCAandPrintToSerial()
// LEGACY FROM TESTING DO NOT USE
// runCAandPrintToSerial
{

  //  ##############################CHRONOAMPEROMETRY##############################

  // runAmp(LMPgain, 174, 40000, -200, 40000, 200, 40000, 400, 6, true);
  for (uint16_t i = 0; i < arr_samples; i++)
  {
    SerialDebugger.print(volts[i]);
    SerialDebugger.print(F(","));
    SerialDebugger.println(amps[i]);
  }
  SerialDebugger.println("Quiet time till next Amp run");
}

void set_sweep_parameters_from_form_input(String form_id, String form_value)
{

  // else if(form_id=="xyz"){
  //   xyz=form_value.toInt();
  // }

  if (form_id == "sweep_param_lmpGain")
  {
    sweep_param_lmpGain = form_value.toInt();
  }

  else if (form_id == "sweep_param_startV_CV")
  {
    sweep_param_startV_CV = form_value.toInt();
  }

  else if (form_id == "sweep_param_endV_CV")
  {
    sweep_param_endV_CV = form_value.toInt();
  }

  else if (form_id == "sweep_param_vertex1_CV")
  {
    sweep_param_vertex1_CV = form_value.toInt();
  }

  else if (form_id == "sweep_param_vertex2_CV")
  {
    sweep_param_vertex2_CV = form_value.toInt();
  }

  else if (form_id == "sweep_param_stepV_CV")
  {
    sweep_param_stepV_CV = form_value.toInt();
  }

  else if (form_id == "sweep_param_rate_CV")
  {
    sweep_param_rate_CV = form_value.toInt();
  }

  else if (form_id == "sweep_param_startV_NPV")
  {
    sweep_param_startV_NPV = form_value.toInt();
  }

  else if (form_id == "sweep_param_endV_NPV")
  {
    sweep_param_endV_NPV = form_value.toInt();
  }

  else if (form_id == "sweep_param_pulseAmp_NPV")
  {
    sweep_param_pulseAmp_NPV = form_value.toInt();
  }

  else if (form_id == "sweep_param_width_NPV")
  {
    sweep_param_width_NPV = form_value.toInt();
  }

  else if (form_id == "sweep_param_period_NPV")
  {
    sweep_param_period_NPV = form_value.toInt();
  }

  else if (form_id == "sweep_param_quietTime_NPV")
  {
    sweep_param_quietTime_NPV = form_value.toInt();
  }

  else if (form_id == "sweep_param_startV_SWV")
  {
    sweep_param_startV_SWV = form_value.toInt();
  }

  else if (form_id == "sweep_param_endV_SWV")
  {
    sweep_param_endV_SWV = form_value.toInt();
  }

  else if (form_id == "sweep_param_pulseAmp_SWV")
  {
    sweep_param_pulseAmp_SWV = form_value.toInt();
  }

  else if (form_id == "sweep_param_stepV_SWV")
  {
    sweep_param_stepV_SWV = form_value.toInt();
  }

  else if (form_id == "sweep_param_freq_SWV")
  {
    sweep_param_freq_SWV = form_value.toInt();
  }

  else if (form_id == "sweep_param_pre_stepV_CA")
  {
    sweep_param_pre_stepV_CA = form_value.toInt();
  }

  else if (form_id == "sweep_param_quietTime_CA")
  {
    sweep_param_quietTime_CA = form_value.toInt();
  }

  else if (form_id == "sweep_param_V1_CA")
  {
    sweep_param_V1_CA = form_value.toInt();
  }

  else if (form_id == "sweep_param_t1_CA")
  {
    sweep_param_t1_CA = form_value.toInt();
  }

  else if (form_id == "sweep_param_V2_CA")
  {
    sweep_param_V2_CA = form_value.toInt();
  }

  else if (form_id == "sweep_param_t2_CA")
  {
    sweep_param_t2_CA = form_value.toInt();
  }

  else if (form_id == "sweep_param_samples_CA")
  {
    sweep_param_samples_CA = form_value.toInt();
  }

  else if (form_id == "sweep_param_biasV_noisetest")
  {
    sweep_param_biasV_noisetest = form_value.toInt();
  }

  else if (form_id == "sweep_param_delayTime_ms_noisetest")
  {
    sweep_param_delayTime_ms_noisetest = form_value.toInt();
  }

  else if (form_id == "sweep_param_startV_IV")
  {
    sweep_param_startV_IV = form_value.toInt();
  }

  else if (form_id == "sweep_param_endV_IV")
  {
    sweep_param_endV_IV = form_value.toInt();
  }

  else if (form_id == "sweep_param_numPoints_IV")
  {
    sweep_param_numPoints_IV = form_value.toInt();
  }

  else if (form_id == "sweep_param_delayTime_ms_IV")
  {
    sweep_param_delayTime_ms_IV = form_value.toInt();
  }

  else if (form_id == "sweep_param_delayTime_ms_CAL")
  {
    sweep_param_delayTime_ms_CAL = form_value.toInt();
  }

  // else if(form_id=="xyz"){
  //   xyz=form_value.toInt();
  // }

  else if (form_id == "sweep_param_cycles_CV")
  {
    sweep_param_cycles_CV = form_value.toInt();
  }

  else if (form_id == "sweep_param_setToZero")
  {
    if (form_value == "true")
    {
      sweep_param_setToZero = true;
    }
    else if (form_value != "true")
    {
      sweep_param_setToZero = false;
    }
  }
}

void configureserver()
// configures server
{
  // Need to tell server to accept packets from any source with any header via http methods GET, PUT:
  DefaultHeaders::Instance().addHeader("Access-Control-Allow-Origin", "*");
  DefaultHeaders::Instance().addHeader("Access-Control-Allow-Methods", "GET, PUT");
  DefaultHeaders::Instance().addHeader("Access-Control-Allow-Headers", "*");

  // // Button #xyz
  // server.addHandler(new AsyncCallbackJsonWebHandler("/buttonxyzpressed", [](AsyncWebServerRequest *requestxyz, JsonVariant &jsonxyz) {
  //   const JsonObject &jsonObjxyz = jsonxyz.as<JsonObject>();
  //   if (jsonObjxyz["on"])
  //   {
  //     Serial.println("Button xyz pressed.");
  //     // digitalWrite(LEDPIN, HIGH);
  //     Sweep_Mode = CV;
  //   }
  //   requestxyz->send(200, "OK");
  // }));

  // Button #1
  server.addHandler(new AsyncCallbackJsonWebHandler("/button1pressed", [](AsyncWebServerRequest *request1, JsonVariant &json1) {
    const JsonObject &jsonObj1 = json1.as<JsonObject>();
    if (jsonObj1["on"])
    {
      Serial.println("Button 1 pressed. Running CV sweep.");
      // digitalWrite(LEDPIN, HIGH);
      Sweep_Mode = CV;
    }
    request1->send(200, "OK");
  }));
  // Button #2
  server.addHandler(new AsyncCallbackJsonWebHandler("/button2pressed", [](AsyncWebServerRequest *request2, JsonVariant &json2) {
    const JsonObject &jsonObj2 = json2.as<JsonObject>();
    if (jsonObj2["on"])
    {
      Serial.println("Button 2 pressed. Running NPV sweep.");
      // digitalWrite(LEDPIN, HIGH);
      Sweep_Mode = NPV;
    }
    request2->send(200, "OK");
  }));
  // Button #3
  server.addHandler(new AsyncCallbackJsonWebHandler("/button3pressed", [](AsyncWebServerRequest *request3, JsonVariant &json3) {
    const JsonObject &jsonObj3 = json3.as<JsonObject>();
    if (jsonObj3["on"])
    {
      Serial.println("Button 3 pressed. Running SQV sweep.");
      // digitalWrite(LEDPIN, HIGH);
      Sweep_Mode = SQV;
    }
    request3->send(200, "OK");
  }));
  // Button #4
  server.addHandler(new AsyncCallbackJsonWebHandler("/button4pressed", [](AsyncWebServerRequest *request4, JsonVariant &json4) {
    const JsonObject &jsonObj4 = json4.as<JsonObject>();
    if (jsonObj4["on"])
    {
      Serial.println("Button 4 pressed. Running CA sweep.");
      // digitalWrite(LEDPIN, HIGH);
      Sweep_Mode = CA;
    }
    request4->send(200, "OK");
  }));
  // Button #5
  server.addHandler(new AsyncCallbackJsonWebHandler("/button5pressed", [](AsyncWebServerRequest *request5, JsonVariant &json5) {
    const JsonObject &jsonObj5 = json5.as<JsonObject>();
    if (jsonObj5["on"])
    {
      Serial.println("Button 5 pressed. Running DC sweep.");
      // digitalWrite(LEDPIN, HIGH);
      Sweep_Mode = DCBIAS;
    }
    request5->send(200, "OK");
  }));
  // Button #6
  server.addHandler(new AsyncCallbackJsonWebHandler("/button6pressed", [](AsyncWebServerRequest *request6, JsonVariant &json6) {
    const JsonObject &jsonObj6 = json6.as<JsonObject>();
    if (jsonObj6["on"])
    {
      Serial.println("Button 6 pressed. Running IV sweep.");
      // digitalWrite(LEDPIN, HIGH);
      Sweep_Mode = IV;
    }
    request6->send(200, "OK");
  }));

  // Button #7
  server.addHandler(new AsyncCallbackJsonWebHandler("/button7pressed", [](AsyncWebServerRequest *request7, JsonVariant &json7) {
    const JsonObject &jsonObj7 = json7.as<JsonObject>();
    if (jsonObj7["on"])
    {
      Serial.println("Button 7 pressed. Running CAL sweep.");
      // digitalWrite(LEDPIN, HIGH);
      Sweep_Mode = CAL;
    }
    request7->send(200, "OK");
  }));
  // Button #8
  server.addHandler(new AsyncCallbackJsonWebHandler("/button8pressed", [](AsyncWebServerRequest *request8, JsonVariant &json8) {
    const JsonObject &jsonObj8 = json8.as<JsonObject>();
    if (jsonObj8["on"])
    {
      Serial.println("Button 8 pressed. Running MISC_MODE sweep.");
      // digitalWrite(LEDPIN, HIGH);
      Sweep_Mode = MISC_MODE;
    }
    request8->send(200, "OK");
  }));
  // Button #9
  server.addHandler(new AsyncCallbackJsonWebHandler("/button9pressed", [](AsyncWebServerRequest *request9, JsonVariant &json9) {
    const JsonObject &jsonObj9 = json9.as<JsonObject>();
    if (jsonObj9["on"])
    {
      Serial.println("Button 9 pressed.");
      // digitalWrite(LEDPIN, HIGH);
      Sweep_Mode = dormant;
    }
    request9->send(200, "OK");
  }));
  // Button #10
  server.addHandler(new AsyncCallbackJsonWebHandler("/button10pressed", [](AsyncWebServerRequest *request10, JsonVariant &json10) {
    const JsonObject &jsonObj10 = json10.as<JsonObject>();
    if (jsonObj10["on"])
    {
      Serial.println("Button 10 pressed.");
      // digitalWrite(LEDPIN, HIGH);
      Sweep_Mode = dormant;
    }
    request10->send(200, "OK");
  }));

  server.serveStatic("/", SPIFFS, "/").setDefaultFile("index.html");

  server.on("/download", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(SPIFFS, "/data.txt", "text/plain", true);
  });

  server.onNotFound([](AsyncWebServerRequest *request) {
    if (request->method() == HTTP_OPTIONS)
    {
      request->send(200); // options request typically sent by client at beginning to make sure server can handle request
    }
    else
    {
      Serial.println("Not found");
      request->send(404, "Not found");
    }
  });

  // Send a POST request to <IP>/actionpage with a form field message set to <message>
  server.on("/actionpage.html", HTTP_POST, [](AsyncWebServerRequest *request) {
    String message;
    Serial.println("server.on bla bla bla called");

    //**********************************************

    // List all parameters int params = request->params();
    int params = request->params();
    for (int i = 0; i < params; i++)
    {
      AsyncWebParameter *p = request->getParam(i);
      if (p->isPost())
      {
        Serial.print(i);
        Serial.print(F("\t"));
        Serial.print(p->name().c_str());
        Serial.print(F("\t"));
        Serial.println(p->value().c_str());
        //SerialDebugger.print(F("\t"))

        //Serial.println(i,'/T',p->name().c_str(),'/T',p->value().c_str());
        // Serial.println(i,'/T',p->name().c_str(),'/T',p->value().c_str());
        //Serial.println(i,'/T',p->name().c_str(),'/T',p->value().c_str());
        //Serial.printf("POST[%s]: %s\n", p->name().c_str(), p->value().c_str());
        set_sweep_parameters_from_form_input(p->name().c_str(), p->value().c_str());
      }
    }

    //**********************************************

    if (request->hasParam(PARAM_MESSAGE, true))
    {
      message = request->getParam(PARAM_MESSAGE, true)->value();
      Serial.println(message);
    }
    else
    {
      message = "No message sent";
    }
    // request->send(200, "text/HTML", "Hello, POST: " + message);
    request->send(200, "text/HTML", "Sweep data saved. Click <a href=\"/index.html\">here</a> to return to main page.");
    // request->send(200, "text/URL", "www.google.com");
    // request->send(200, "text/URL", "<meta http-equiv=\"Refresh\" content=\"0; URL=https://google.com/\">");
    // <meta http-equiv="Refresh" content="0; URL=https://example.com/">
  });

  server.begin();
}

// void configuresweepparamssever(){
// // configures server to parse html post messages that allow the user to set the sweep parameters
// // Sweep parameters (to be set by user through HTML form posts but defaults on initialization)
// // int sweep_param_lmpGain=7; //  (index) gain setting for LMP91000
// // int sweep_param_cycles=3;  //  (#)  number of times to run the scan
// // int sweep_param_startV=0; //   (mV)  voltage to start the scan
// // int sweep_param_endV=0; //     (mV)  voltage to stop the scan
// // int sweep_param_vertex1=100;//  (mV)  edge of the scan
// // int sweep_param_vertex2=-100; // (mV)  edge of the scan
// // int sweep_param_stepV=5; // (mV)      how much to increment the voltage by
// // int sweep_param_rate=100; // (mV/sec)       scanning rate
// // bool sweep_param_setToZero=true;//   Boolean determining whether the bias potential of

// }

void setup()
{
  pinMode(LEDPIN, OUTPUT);

  Wire.begin(); // uses default microcontroller pins
  SerialDebugger.begin(115200);
  while (!SerialDebugger)
    ;

  analogReadResolution(12);
  //  analogWriteResolution(10); // ESP32 only does 8 bits.
  // initADC(false); // Will not work for ESP32 boards.

  //enable the potentiostat
  pStat.setMENB(MENB);
  delay(50);
  pStat.standby();
  delay(50);
  initLMP(0);                 // Initializes the LMP91000 to the appropriate settings
  blinkLED(LEDPIN, 15, 1000); // blink LED to show setup is complete, also give settle time to LMP91000
  SerialDebugger.println(F("Setup complete."));
  SerialDebugger.print(F(" "));

  //############################# WEBSERVER & WIFI #####################################

  WiFi.mode(WIFI_STA);
  WiFi.begin(SSID, PASSWORD);
  while (WiFi.waitForConnectResult() != WL_CONNECTED)
  {
    Serial.println("Connected Failed! Rebooting...");
    delay(1000);
    ESP.restart();
  }

  Serial.println("Connected!");
  Serial.println("The local IP address is:");
  //print the local IP address
  Serial.println(WiFi.localIP());
  if (!SPIFFS.begin(true))
  {
    Serial.println("An Error has occurred while mounting SPIFFS");
    return;
  }
  MDNS.begin("ESP32Stat");

  configureserver();
  //####################################################################################
  File root = SPIFFS.open("/");
  File file = root.openNextFile();
}

void loop()
{

  //will hold the code here until a character is sent over the SerialDebugger port
  //this ensures the experiment will only run when initiated

  //  SerialDebugger.println(F("Press enter to begin a sweep."));
  //  while (!SerialDebugger.available())
  //    ;
  //  SerialDebugger.read();

  delay(10); // 10 ms delay

  if (Sweep_Mode == NPV)
  {

    // testNoiseAtABiasPoint(-100, 100, 50);

    // testNOISE(100);
    // testDACs(50);
    // testDACandADCs(50);
    //testLMP91000(50, 1);
    // runNPV(6, -200, 500, 10, 50, 200, 1000, 1, true);

    runNPV(sweep_param_lmpGain, sweep_param_startV_NPV, sweep_param_endV_NPV,
           sweep_param_pulseAmp_NPV, sweep_param_pulseAmp_NPV, sweep_param_period_NPV,
           sweep_param_quietTime_NPV, 1, sweep_param_setToZero);

    writeVoltsCurrentArraytoFile();
    Sweep_Mode = dormant;
  }
  else if (Sweep_Mode == CV)
  {

    // testLMP91000(50, 1);

    // runCV(LMPgain, 2, 0, 0, 450, -200, 5, 1000, true);

    runCV(sweep_param_lmpGain, sweep_param_cycles_CV, sweep_param_startV_CV,
          sweep_param_endV_CV, sweep_param_vertex1_CV, sweep_param_vertex2_CV, sweep_param_stepV_CV,
          sweep_param_rate_CV, sweep_param_setToZero);
    writeVoltsCurrentArraytoFile();
    Sweep_Mode = dormant;
  }
  else if (Sweep_Mode == SQV)
  {
    // runSWV(LMPgain, -200, 500, 25, 5, 10, true);

    runSWV(sweep_param_lmpGain, sweep_param_startV_SWV, sweep_param_endV_SWV,
           sweep_param_pulseAmp_SWV, sweep_param_stepV_SWV, sweep_param_freq_SWV, sweep_param_setToZero);
    writeVoltsCurrentArraytoFile();
    Sweep_Mode = dormant;
  }
  else if (Sweep_Mode == CA)
  {
    // runAmp(LMPgain, 50, 2000, 100, 2000, 50, 200, 100, 1, true);

    runAmp(sweep_param_lmpGain, sweep_param_pre_stepV_CA, sweep_param_quietTime_CA,
           sweep_param_V1_CA, sweep_param_t1_CA, sweep_param_V2_CA, sweep_param_t2_CA,
           sweep_param_samples_CA, 1, sweep_param_setToZero);
    writeVoltsCurrentTimeArraytoFile();
    Sweep_Mode = dormant;
  }
  else if (Sweep_Mode == DCBIAS)
  {
    // testNoiseAtABiasPoint(-100, 100, 50); //  bias at -100 mV on cell. Read 100 readings and calculate std dev and avg of adc for different reading avgs.
    testNoiseAtABiasPoint(sweep_param_biasV_noisetest, sweep_param_numPoints_noisetest,
                          sweep_param_delayTime_ms_noisetest);
    writeVoltsCurrentTimeArraytoFile();
    // testNOISE(100);
    // testDACs(50);
    // testDACandADCs(50);
    // testLMP91000(50, 1);
    Sweep_Mode = dormant;
  }
  else if (Sweep_Mode == IV)
  {
    //testIV(-200, 500, 701, 50);

    testIV(sweep_param_startV_IV, sweep_param_endV_IV, sweep_param_numPoints_IV,
           sweep_param_delayTime_ms_IV);
    writeVoltsCurrentArraytoFile();
    Sweep_Mode = dormant;
  }
  else if (Sweep_Mode == CAL)
  {
    // calibrateDACandADCs(50);
    // calibrateDACandADCs(sweep_param_delayTime_ms_CAL);

    Sweep_Mode = dormant;
  }
  else if (Sweep_Mode == MISC_MODE)
  {
    // testNoiseAtABiasPoint(-100, 100, 50);
    listDir("/", 3);
    writeVoltsCurrentArraytoFile();
    sleep(1);
    readFileAndPrintToSerial();
    Sweep_Mode = dormant;
  }
  else
  {
    delay(10);
  }
}
