//https://github.com/PeterJBurke/Nanostat

bool userpause = false;             // pauses for user to press input on serial between each point in sweep
bool print_output_to_serial = true; // prints verbose output to serial

//Libraries
#include <Wire.h>
#include <WiFi.h>
#include "Arduino.h"
#include <ESPAsyncWebServer.h>
#include <ESPmDNS.h>
#include <SPIFFS.h>
#include <AsyncJson.h>
#include "wifi_credentials.h"
#include "LMP91000.h"
#include "WebSocketsServer.h"
#include "DNSServer.h"
// #include <OTA.h>
#include <Update.h>

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
  CTLPANEL, // Control panel type interface
  MISC_MODE // miscellanous
};
Sweep_Mode_Type Sweep_Mode = dormant;

// Sweep parameters (to be set by user through HTML form posts but defaults on initialization)
// (Initialized to their default values)
// General sweep parameters:
int sweep_param_lmpGain = 7;       //  (index) gain setting for LMP91000
bool sweep_param_setToZero = true; //   Boolean

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
int sweep_param_endV_NPV = 200;    //     (mV)  voltage to stop the scan
int sweep_param_pulseAmp_NPV = 20;
int sweep_param_width_NPV = 50;
int sweep_param_period_NPV = 200;
int sweep_param_quietTime_NPV = 1000;

// SWV sweep parameters:
// runSWV(sweep_param_lmpGain, sweep_param_startV_SWV, sweep_param_endV_SWV,
//            sweep_param_pulseAmp_SWV, sweep_param_stepV_SWV, sweep_param_freq_SWV, sweep_param_setToZero)
int sweep_param_startV_SWV = -200; //   (mV)  voltage to start the scan
int sweep_param_endV_SWV = 200;    //     (mV)  voltage to stop the scan
int sweep_param_pulseAmp_SWV = 20;
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
int sweep_param_t2_CA = 2000;
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
int sweep_param_endV_IV = 200;    //     (mV)  voltage to stop the scan
int sweep_param_numPoints_IV = 701;
int sweep_param_delayTime_ms_IV = 50;

// Cal sweep parameters:
// calibrateDACandADCs(sweep_param_delayTime_ms_CAL)
int sweep_param_delayTime_ms_CAL = 50;

// Arrays of IV curves etc:
const uint16_t arr_samples = 5000; //use 1000 for EIS, can use 2500 for other experiments (10k does not fit in DRAM)
uint16_t arr_cur_index = 0;
int16_t volts[arr_samples] = {0}; // single sweep IV curve "V"
float amps[arr_samples] = {0};    // single sweep IV curve "I"
int32_t time_Voltammaogram[arr_samples] = {0};
int number_of_valid_points_in_volts_amps_array = 0; // rest of them are all zeros...

// JSON string to sent over websockets to browswer client:
// char Voltammogram_JSON_cstr[31000];
// char Voltammogram_JSON_cstr[31000];
//char Voltammogram_JSON_cstr2[31000];
int16_t volts_temp = 0;
float amps_temp = 0;
float v1_temp = 0;
float v2_temp = 0;
float analog_read_avg_bits_temp = 0;
String temp_json_string = "";

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
uint8_t LMPgainGLOBAL = 7; // Feedback resistor of TIA.
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
float RFB = 1e6; // feedback resistor if using external feedback resistor

// create pStat object to control LMP91000
LMP91000 pStat = LMP91000();

// create webserver object for website:
AsyncWebServer server(80); //

// Websockets:
// Tutorial: https://www.youtube.com/watch?v=ZbX-l1Dl4N4&list=PL4sSjlE6rMIlvrllrtOVSBW8WhhMC_oI-&index=8
// Tutorial: https://www.youtube.com/watch?v=mkXsmCgvy0k
// Code tutorial: https://shawnhymel.com/1882/how-to-create-a-web-server-with-websockets-using-an-esp32-in-arduino/
// Github: https://github.com/Links2004/arduinoWebSockets
WebSocketsServer m_websocketserver = WebSocketsServer(81);
String m_websocketserver_text_to_send = "";
String m_websocketserver_text_to_send_2 = "";
int m_time_sent_websocketserver_text = millis();
int m_microsbefore_websocketsendcalled = micros();
int last_time_loop_called = millis();
int last_time_sent_websocket_server = millis();
float m_websocket_send_rate = 1.0; // Hz, how often to send a test point to websocket...
bool m_send_websocket_test_data_in_loop = false;

//WifiTool object
int WAIT_FOR_WIFI_TIME_OUT = 6000;
const char *PARAM_MESSAGE = "message"; // message server receives from client
std::unique_ptr<DNSServer> dnsServer;
std::unique_ptr<AsyncWebServer> m_wifitools_server;
const byte DNS_PORT = 53;
bool restartSystem = false;

// Control panel mode settings: (Control panel mode is browser page like a front panel of instrument)
uint8_t LMPgain_control_panel = 6; // Feedback resistor of TIA.
int num_adc_readings_to_average_control_panel = 1;
int sweep_param_delayTime_ms_control_panel = 50;
int cell_voltage_control_panel = 100;

//******************* END VARIABLE DECLARATIONS**************************8

boolean connectAttempt(String ssid, String password)
{
  boolean isWiFiConnected = false;
  // set mode
  WiFi.mode(WIFI_STA);
  // if no SSID is passed we attempt last connected wifi (from WIFI object)
  if (ssid == "")
  {
    if (WiFi.status() != WL_CONNECTED)
    {
      WiFi.begin();
    }
  }
  else
  {
    int ssidSize = ssid.length() + 1;
    int passwordSize = password.length() + 1;
    char ssidArray[ssidSize];
    char passwordArray[passwordSize];
    ssid.toCharArray(ssidArray, ssidSize);
    password.toCharArray(passwordArray, passwordSize);
    WiFi.begin(ssidArray, passwordArray);
  } //end if

  if (ssid == "")
  {
    Serial.print(F("Connecting Wifi default SSID ..."));
  }
  else if (ssid != "")
  {
    Serial.print(F("Connecting Wifi SSID = "));
    Serial.print(ssid);
    Serial.print(F(" ..."));
  }
  unsigned long now = millis();
  while (WiFi.status() != WL_CONNECTED && millis() < now + WAIT_FOR_WIFI_TIME_OUT)
  {
    Serial.print(".");
    delay(250);
  }
  // Serial.print(F("\nStatus:"));
  // Serial.println(WiFi.status());
  if (WiFi.status() == WL_CONNECTED)
  {
    Serial.println(F("\nWiFi connected"));
    Serial.println(F("IP address: "));
    Serial.println(WiFi.localIP());
    Serial.print(F("ssid: "));
    Serial.println(WiFi.SSID());
    //Serial.print(F("password: "));
    //Serial.println(WiFi.psk());
    isWiFiConnected = true;
  }

  return isWiFiConnected;
}

void sendTimeOverWebsocketJSON() // sends current time as JSON object to websocket
{
  String json = "{\"value\":";
  json += String(millis() / 1e3, 3);
  json += "}";
  m_websocketserver.broadcastTXT(json.c_str(), json.length());
}

void sendValueOverWebsocketJSON(int value_to_send_over_websocket) // sends integer as JSON object to websocket
{
  String json = "{\"value\":";
  json += String(value_to_send_over_websocket);
  json += "}";
  m_websocketserver.broadcastTXT(json.c_str(), json.length());
}

void sendStringOverWebsocket(String string_to_send_over_websocket)
{
  m_websocketserver.broadcastTXT(string_to_send_over_websocket.c_str(), string_to_send_over_websocket.length());
}

void send_is_sweeping_status_over_websocket(bool is_sweeping)
{
  if (is_sweeping)
  {
    // send is sweeping
    temp_json_string = "{\"is_sweeping\":true}";
  };
  if (!is_sweeping)
  {
    // send is not sweeping
    temp_json_string = "{\"is_sweeping\":false}";
  };
  m_websocketserver.broadcastTXT(temp_json_string.c_str(), temp_json_string.length());
}

void send_expect_binary_data_over_websocket(bool expect_binary_data)
{
  if (expect_binary_data)
  {
    // send is sweeping
    temp_json_string = "{\"expect_binary_data\":true}";
  };
  if (!expect_binary_data)
  {
    // send is not sweeping
    temp_json_string = "{\"expect_binary_data\":false}";
  };
  m_websocketserver.broadcastTXT(temp_json_string.c_str(), temp_json_string.length());
}

void sendVoltammogramWebsocketJSON()
{
  // old version used String, had memory probs for large arrays
  // psuedo code:
  // 1) Convert voltammagram to JSON...
  // Voltammagram JSON format will be like this:
  // { "Current" : [3,2,6,...], "Voltage": [8,6,7,....], "Time": [3,4,5,...]}
  // 2) Send JSON over websocket...

  Serial.println("sendVoltammogramWebsocketJSON called");
  Serial.print("number_of_valid_points_in_volts_amps_array=");
  Serial.println(number_of_valid_points_in_volts_amps_array);

  String current_array_string = "[";
  String voltage_array_string = "[";
  String time_array_string = "[";
  for (uint16_t i = 0; i < number_of_valid_points_in_volts_amps_array; i++)
  {

    current_array_string += amps[i];
    if (i != (number_of_valid_points_in_volts_amps_array - 1))
    {
      current_array_string += ",";
    }

    voltage_array_string += volts[i];
    if (i != (number_of_valid_points_in_volts_amps_array - 1))
    {
      voltage_array_string += ",";
    }

    time_array_string += (time_Voltammaogram[i] - time_Voltammaogram[0]); // normalize to start of sweep time
    if (i != (number_of_valid_points_in_volts_amps_array - 1))
    {
      time_array_string += ",";
    }
  }
  current_array_string += "]";
  voltage_array_string += "]";
  time_array_string += "]";

  String Voltammogram_JSON = "";
  Voltammogram_JSON += "{\"Current\":";
  Voltammogram_JSON += current_array_string;
  Voltammogram_JSON += ",\"Voltage\":";
  Voltammogram_JSON += voltage_array_string;
  Voltammogram_JSON += ",\"Time\":";
  Voltammogram_JSON += time_array_string;
  Voltammogram_JSON += "}";

  if (print_output_to_serial)
  {
    Serial.println("####################################");
    Serial.println("Just created Voltammogram_JSON string.");
    Serial.println("Ingredients of string:");
    Serial.println("current_array_string:");
    Serial.println(current_array_string);
    Serial.println("voltage_array_string:");
    Serial.println(voltage_array_string);
    Serial.println("time_array_string:");
    Serial.println(time_array_string);

    Serial.println("####################################");
    Serial.println("Beginning sending Voltammogram_JSON over websocket.");
    if (print_output_to_serial)
    {
      Serial.println(Voltammogram_JSON);
    }
    Serial.print("Heap free memory (in bytes)= ");
    Serial.println(ESP.getFreeHeap());
  }

  m_websocketserver.broadcastTXT(Voltammogram_JSON.c_str(), Voltammogram_JSON.length());

  if (print_output_to_serial)
  {
    Serial.println("Finished sending Voltammogram_JSON over websocket.");
    Serial.println("####################################");
  }
}

void sendVoltammogramWebsocketBIN()
{
  // send data over websocket binary
  // psuedo code:
  // 1) Convert voltammagram to JSON...
  // Voltammagram JSON format will be like this:
  // { "Current" : [3,2,6,...], "Voltage": [8,6,7,....], "Time": [3,4,5,...]}
  // 2) Send JSON over websocket...
  // Version 2 tries to use char[] instead of String for memory management (String can't hold all the data)

  // Refernce arrays we are going to use to write:
  // Arrays of IV curves etc:
  // const uint16_t arr_samples = 5000; //use 1000 for EIS, can use 2500 for other experiments (10k does not fit in DRAM)
  // uint16_t arr_cur_index = 0;
  // int16_t volts[arr_samples] = {0}; // single sweep IV curve "V"
  // float amps[arr_samples] = {0};    // single sweep IV curve "I"
  // int32_t time_Voltammaogram[arr_samples] = {0};
  // int number_of_valid_points_in_volts_amps_array = 0; // rest of them are all zeros...

  Serial.println("sendVoltammogramWebsocketBIN called");
  Serial.print("Heap free memory (in bytes)= ");
  Serial.println(ESP.getFreeHeap());
  Serial.println("Creating binary array to send to browswer");
  Serial.print("number_of_valid_points_in_volts_amps_array=");
  Serial.println(number_of_valid_points_in_volts_amps_array);

  // reset time to zero for 1st point: (unless it's already been done!)
  if (time_Voltammaogram[0] != 0)
  {
    int32_t temp_time, temp_start_time;
    temp_start_time = time_Voltammaogram[0];
    for (uint16_t i = 0; i < number_of_valid_points_in_volts_amps_array; i++)
    {
      temp_time = time_Voltammaogram[i];
      time_Voltammaogram[i] = temp_time - temp_start_time;
    }
  }

  bool m_headerToPayload = false;

  //  *********BEGIN Websocket sample code for sending binary array of floats*****************
  // int m_num_in_test_array = 10;
  // float m_test_float_array[m_num_in_test_array];
  // for (uint8_t i = 0; i < m_num_in_test_array; i += 1)
  // {
  //   m_test_float_array[i] = 3.1415 + i;
  //   Serial.print("m_test_float_array[i]=");
  //   Serial.println(m_test_float_array[i]);
  // }
  // size_t m_test_float_array_buffer_len = m_num_in_test_array * sizeof(float); // 4 byte
  // Serial.print("m_test_float_array_buffer_len=");
  // Serial.println(m_test_float_array_buffer_len);
  // m_websocketserver.broadcastBIN((uint8_t *)m_test_float_array, m_test_float_array_buffer_len, m_headerToPayload);
  //  *********END Websocket sample code for sending binary array of floats*****************

  //  *********BEGIN Websocket  code for sending  arrays*****************
  int m_amps_array_buffer_len = sizeof(float) * number_of_valid_points_in_volts_amps_array;
  int m_volts_array_buffer_len = sizeof(int16_t) * number_of_valid_points_in_volts_amps_array;
  int m_time_array_buffer_len = sizeof(int32_t) * number_of_valid_points_in_volts_amps_array;
  Serial.print("m_amps_array_buffer_len=");
  Serial.println(m_amps_array_buffer_len);
  Serial.print("m_volts_array_buffer_len=");
  Serial.println(m_volts_array_buffer_len);
  Serial.print("m_time_array_buffer_len=");
  Serial.println(m_time_array_buffer_len);

  // Tell browswer to get ready for 3 new binary array messages:
  send_expect_binary_data_over_websocket(true);
  // Arrays are aleady in memory, no need to use more memory, just send them directly:
  m_websocketserver.broadcastBIN((uint8_t *)amps, m_amps_array_buffer_len, m_headerToPayload);
  m_websocketserver.broadcastBIN((uint8_t *)volts, m_volts_array_buffer_len, m_headerToPayload);
  m_websocketserver.broadcastBIN((uint8_t *)time_Voltammaogram, m_time_array_buffer_len, m_headerToPayload);

  //  *********END Websocket  code for sending  arrays*****************

  if (print_output_to_serial)
  {
    Serial.println("Finished sending sendVoltammogramWebsocketBIN over websocket.");
    Serial.println("####################################");
  }
}

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
}

void setOutputsToZero()
{
  //Sets the electrochemical cell to 0V bias.
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
  setOutputsToZero();      // Zero voltage on cell, DAC.
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
  // Reads adc at pin_num for num_points and returns average.
  float analogRead_result = 0;

  for (int16_t j = 0; j < num_points; j += 1)
  {
    analogRead_result += analogRead(pin_num);
  }

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
}

inline float biasAndSample(int16_t voltage, uint32_t rate)
{
  //Sets the bias of the electrochemical cell then samples the resulting current.
  //@param        voltage: Set the bias of the electrochemical cell
  //@param        rate:    How much to time (in ms) should we wait to sample
  //                       current after biasing the cell. This parameter
  //                       sets the scan rate or the excitation frequency
  //                       based on which electrochemical technique
  //                       we're running.

  // Global variables assumed:
  // lastTime: last time in system ms this function was called (for timing reference)
  // num_adc_readings_to_average: self explanatory
  // float a = a_coeff; float b = b_coeff; // global calibration coefficients for ADC to voltage correction

  // Global variables used in this function implicit through setVoltage:
  // v_tolerance: how close to desired cell voltage user can tolerate since it is digitized
  // dacVout: desired output of the DAC in mV
  // bias_setting: determines percentage of VREF applied to CE opamp, from 1% to 24%

  setLMPBias(voltage); // Sets the LMP91000's bias to positive or negative // voltage is cell voltage
  setVoltage(voltage); // Sets the DAC voltage, LMP91000 bias percentage, and LMP91000 bias sign, to get the desired cell "voltage".

  if (rate > 10) // we can afford the time to pulse the LED for 10 ms, just waiting anyways
  {
    pulseLED_on_off(LEDPIN, 10); // signify start of a new data point
  }
  else if (rate < 10) // we cannot afford the time to pulse the LED for 10 ms, so pulse 1 ms only
  {
    pulseLED_on_off(LEDPIN, 1); // signify start of a new data point
  }

  //delay sampling to set scan rate
  while (millis() - lastTime < rate) // wait then sample
    ;

  // Read output voltage of the transimpedance amplifier
  int adc_bits; // will be output voltage of TIA amplifier, "VOUT" on LMP91000 diagram, also C2
  // hard wired in BurkeLab ESP32Stat Rev 3.5 to LMP i.e ESP32 pin 32 (ADC1_CH7)
  adc_bits = analog_read_avg(num_adc_readings_to_average, LMP); // read a number of points and average them...

  // Now convert adc_bits to actual voltage on Vout.
  // Without calibration, we would assume that virtual ground (WE = C1) is 50% of Vref.
  // Without calibration, we would assume that C2-C1=I_WE * RTIA, and C2 is VOUT is ADC reading.
  // However, C1 is not 50% of Vref.
  // Calibration takes care of this by reading C2 as a function of ADC set voltage (assuming perfect ADC) and then
  // correcting C2 using Vout_calibrated = (adcbits/4095)*3.3V corrected by slope/offset measured during calibration, as follows:
  // First, user opens WE so no current flows, then runs the cal function, which sets LMP cell voltage to Zero and varies Vref i.e. Vdac.
  // That function measures Vadc as a function of Vdac and fits this formula (defining a,b):
  // adc(bits) = a + b * dac = a + b * dacvoltage * (255/3.3); here dac voltage is assumed correct
  // Correction from ADC to Vout then given by (in volts):
  // v1 = (3.3 / 255.0) * (1 / (2.0 * b)) * (float)adc_bits - (a / (2.0 * b)) * (3.3 / 255.0);

  //  Calibrate coefficients (make a local copy of the global ones):
  float a = a_coeff; // a is local copy of global a_coeff
  float b = b_coeff; // b is local copy of global a_coeff

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
  if (LMPgainGLOBAL == 0)                              // using external feedback resistor, hard coded to 1e6 for now with BurkeLab ESP32Stat Rev 3.5
    current = (((v1 - v2) / 1000) / RFB) * pow(10, 9); //scales to nA
  else
    current = (((v1 - v2) / 1000) / TIA_GAIN[LMPgainGLOBAL - 1]) * pow(10, 6); //scales to uA

  if (print_output_to_serial)
  {
    // Serial.print("******** BIAS AND SAMPLE START********************)");
    Serial.print(int(millis())); // current time in ms
    // Serial.print(F(","));        // comma
    Serial.print(F("\t")); // tab
    Serial.print(voltage); // desired cell voltage
    // Serial.print(F(","));  // comma
    Serial.print(F("\t"));                          // tab
    Serial.print(dacVout * TIA_BIAS[bias_setting]); // SET cell voltage
    // setV = dacVout * TIA_BIAS[bias_setting]
    // Serial.print(F(",")); // comma
    Serial.print(F("\t"));                   // tab
    Serial.print(TIA_BIAS[bias_setting], 2); // scale divider
    // Serial.print(F(","));                    // comma
    Serial.print(F("\t"));      // tab
    Serial.print(dacVout, DEC); // dacVout a global variable, calculated in setVoltage(voltage);
    // Serial.print(F(","));       // comma
    Serial.print(F("\t"));     // tab
    Serial.print(adc_bits, 0); // adc_bits
    // Serial.print(F(","));      // comma
    Serial.print(F("\t")); // tab
    Serial.print(v1, 1);   // Vout
    // Serial.print(F(",")); // comma
    Serial.print(F("\t")); // tab
    Serial.print(v2, 0);   // C1, should be the zero also....
    // Serial.print(F(",")); // comma
    Serial.print(F("\t")); // tab
    Serial.print(current, 4);
    // Serial.print(F(",")); // comma
    Serial.print(F("\t")); // tab
    // Serial.print("******** BIAS AND SAMPLE END********************)");
  }

  //update timestamp for the next measurement
  lastTime = millis();
  return current;
}

void writeVoltsCurrentArraytoFile()
{
  File file = SPIFFS.open("/data.txt", FILE_WRITE);

  if (!file)
  {
    Serial.println("There was an error opening the file for writing");
    return;
  }
  if (file.println("BurkeLab Nanostat Rev 3.5 Sweep"))
  {
    Serial.println("Header of file was written");
  }
  else
  {
    Serial.println("File write failed");
  }
  Serial.println("Writing file.");
  Serial.println("number_of_valid_points_in_volts_amps_array=");
  Serial.println(number_of_valid_points_in_volts_amps_array);

  file.print("Index");
  file.print(F("\t")); // tab
  file.print("Current_Amps");
  file.print(F("\t")); // tab
  file.print("Voltage_V");
  file.print(F("\t")); // tab
  file.println("Time_ms");

  //Wrie Arrays
  for (uint16_t i = 0; i < number_of_valid_points_in_volts_amps_array; i++)
  {
    file.print(i);
    file.print(F("\t")); // tab
    file.print(amps[i], DEC);
    file.print(F("\t")); // tab
    file.print(volts[i] / 1e3, DEC);
    file.print(F("\t"));                                         // tab
    file.println(time_Voltammaogram[i] - time_Voltammaogram[0]); // normalize to start of sweep
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

void writeCalFile()
{
  File m_cal_file_to_write_name = SPIFFS.open("/calibration.JSON", FILE_WRITE);

  if (!m_cal_file_to_write_name)
  {
    Serial.println("There was an error opening the file for writing");
    return;
  }

  String m_calibration_string_to_write_JSON = "";
  m_calibration_string_to_write_JSON += "{\"acoeff\":";
  m_calibration_string_to_write_JSON += a_coeff;
  m_calibration_string_to_write_JSON += ",\"bcoeff\":";
  m_calibration_string_to_write_JSON += b_coeff;
  m_calibration_string_to_write_JSON += "}";

  Serial.println("Writing this JSON string to cal file:");
  Serial.println(m_calibration_string_to_write_JSON);

  if (!m_cal_file_to_write_name.println(m_calibration_string_to_write_JSON))
  {
    Serial.println("File write failed");
  }
  m_cal_file_to_write_name.close();
}

void readCalFile()
{
  File m_cal_file_name = SPIFFS.open("/calibration.JSON");

  if (!m_cal_file_name)
  {
    Serial.println("Failed to open cal file file for reading");
    return;
  }

  if (!SPIFFS.exists("/calibration.JSON"))
  {
    Serial.println("calibration.JSON does not exist, creating one.");
    writeCalFile();
    return;
  }

  // Serial.println("Cal file opened.");
  // Serial.println("File Content:");

  String secretsJson;
  while (m_cal_file_name.available()) // read json from file
  {
    secretsJson += char(m_cal_file_name.read());
  } //end while

  // Serial.println("Parsing cal file: ");
  // Serial.println(secretsJson);
  StaticJsonDocument<1000> m_JSONdoc_from_payload;
  DeserializationError m_error = deserializeJson(m_JSONdoc_from_payload, secretsJson); // m_JSONdoc is now a json object
  if (m_error)
  {
    Serial.println("deserializeJson() failed with code ");
    Serial.println(m_error.c_str());
  }
  //JsonObject m_JsonObject_from_payload = m_JSONdoc_from_payload.as<JsonObject>();
  a_coeff = m_JSONdoc_from_payload["acoeff"];
  b_coeff = m_JSONdoc_from_payload["bcoeff"];
  Serial.println("Cal file loaded with these parameters:");
  Serial.print("a_coeff = ");
  Serial.print(a_coeff);
  Serial.print(F("\t")); // tab
  Serial.print("b_coeff = ");
  Serial.println(b_coeff);

  m_cal_file_name.close();
}

bool readSSIDPWDfile(String m_pwd_filename_to_read)
{
  File m_pwd_file_to_read = SPIFFS.open(m_pwd_filename_to_read);

  if (!m_pwd_file_to_read)
  {
    Serial.println("Failed to open PWD file file for reading");
    return false;
  }

  if (!SPIFFS.exists(m_pwd_filename_to_read))
  {
    Serial.print(m_pwd_filename_to_read);
    Serial.println("  does not exist.");
    return false;
  }

  String m_pwd_file_string;
  while (m_pwd_file_to_read.available()) // read json from file
  {
    m_pwd_file_string += char(m_pwd_file_to_read.read());
  } //end while
  // Serial.print("m_pwd_file_string = ");
  // Serial.println(m_pwd_file_string);
  m_pwd_file_to_read.close();

  //parse
  StaticJsonDocument<1000> m_JSONdoc_from_pwd_file;
  DeserializationError m_error = deserializeJson(m_JSONdoc_from_pwd_file, m_pwd_file_string); // m_JSONdoc is now a json object
  if (m_error)
  {
    Serial.println("deserializeJson() failed with code ");
    Serial.println(m_error.c_str());
  }
  // m_JSONdoc_from_pwd_file is the JSON object now we can use it.
  String m_SSID1_name = m_JSONdoc_from_pwd_file["SSID1"];
  String m_SSID2_name = m_JSONdoc_from_pwd_file["SSID2"];
  String m_SSID3_name = m_JSONdoc_from_pwd_file["SSID3"];
  String m_PWD1_name = m_JSONdoc_from_pwd_file["PWD1"];
  String m_PWD2_name = m_JSONdoc_from_pwd_file["PWD1"];
  String m_PWD3_name = m_JSONdoc_from_pwd_file["PWD1"];
  // Serial.print("m_SSID1_name = ");
  // Serial.print(m_SSID1_name);
  // Serial.print(F("\t")); // tab
  // Serial.print("m_PWD1_name = ");
  // Serial.print(F("\t")); // tab
  // Serial.print("m_SSID2_name = ");
  // Serial.print(m_SSID2_name);
  // Serial.print(F("\t")); // tab
  // Serial.print("m_PWD2_name = ");
  // Serial.print(m_PWD2_name);
  // Serial.print(F("\t")); // tab
  // Serial.print("m_SSID3_name = ");
  // Serial.print(m_SSID3_name);
  // Serial.print(F("\t")); // tab
  // Serial.print("m_PWD3_name = ");
  // Serial.println(m_PWD3_name);

  // Try connecting:
  //****************************8
  if (connectAttempt(m_SSID1_name, m_PWD1_name))
  {
    return true;
  }
  Serial.println("Failed to connect.");
  if (connectAttempt(m_SSID2_name, m_PWD2_name))
  {
    return true;
  }
  Serial.println("Failed to connect.");

  if (connectAttempt(m_SSID3_name, m_PWD3_name))
  {
    return true;
  }
  Serial.println("Failed to connect.");

  return false;
}

void setUpAPService()
{
  Serial.println(F("Starting Access Point server."));

  // DNSServer dnsServer;
  // dnsServer.reset(new DNSServer());
  WiFi.mode(WIFI_AP);
  // WiFi.softAPConfig(IPAddress(172, 217, 28, 1), IPAddress(172, 217, 28, 1), IPAddress(255, 255, 255, 0));
  WiFi.softAP("NanoStatAP");
  delay(500);

  /* Setup the DNS server redirecting all the domains to the apIP */
  // dnsServer->setErrorReplyCode(DNSReplyCode::NoError);
  // dnsServer->start(DNS_PORT, "*", IPAddress(172, 217, 28, 1));

  //Serial.println("dns server config done");
}

void process()
{
  ///DNS
  // dnsServer->processNextRequest();
  //yield
  yield();
  delay(10);
  // Reset flag/timer
  if (restartSystem)
  {
    if (restartSystem + 1000 < millis())
    {
      ESP.restart();
    } //end if
  }   //end if
}

void handleGetSavSecreteJson(AsyncWebServerRequest *request)
{
  String message;

  String m_SSID1_name;
  String m_SSID2_name;
  String m_SSID3_name;
  String m_PWD1_name;
  String m_PWD2_name;
  String m_PWD3_name;
  String m_temp_string;
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
      m_temp_string = p->name().c_str();
      if (m_temp_string == "ssid1")
      {
        m_SSID1_name = p->value().c_str();
      }
      else if (m_temp_string == "pass1")
      {
        m_PWD1_name = p->value().c_str();
      }
      else if (m_temp_string == "ssid2")
      {
        m_SSID2_name = p->value().c_str();
      }
      else if (m_temp_string == "pass2")
      {
        m_PWD2_name = p->value().c_str();
      }
      else if (m_temp_string == "ssid3")
      {
        m_SSID3_name = p->value().c_str();
      }
      else if (m_temp_string == "pass3")
      {
        m_PWD3_name = p->value().c_str();
      }
    }
  }
  if (request->hasParam(PARAM_MESSAGE, true))
  {
    message = request->getParam(PARAM_MESSAGE, true)->value();
    Serial.println(message);
  }
  else
  {
    message = "No message sent";
  }
  request->send(200, "text/HTML", "Credentials saved. Rebooting...");

  // {"SSID1":"myssid1xyz","PWD1":"mypwd1xyz",
  //     "SSID2":"myssid2xyz","PWD2":"mypwd2xyz",
  //     "SSID3":"myssid3xyz","PWD3":"mypwd3xyz"}

  String SSID_and_pwd_JSON = "";
  SSID_and_pwd_JSON += "{\"SSID1\":\"";
  SSID_and_pwd_JSON += m_SSID1_name;
  SSID_and_pwd_JSON += "\",\"PWD1\":\"";
  SSID_and_pwd_JSON += m_PWD1_name;

  SSID_and_pwd_JSON += "\",\"SSID2\":\"";
  SSID_and_pwd_JSON += m_SSID2_name;
  SSID_and_pwd_JSON += "\",\"PWD2\":\"";
  SSID_and_pwd_JSON += m_PWD2_name;

  SSID_and_pwd_JSON += "\",\"SSID3\":\"";
  SSID_and_pwd_JSON += m_SSID3_name;
  SSID_and_pwd_JSON += "\",\"PWD3\":\"";
  SSID_and_pwd_JSON += m_PWD3_name;

  SSID_and_pwd_JSON += "\"}";

  Serial.println("JSON string to write to file = ");
  Serial.println(SSID_and_pwd_JSON);

  Serial.print("m_SSID1_name = ");
  Serial.print(m_SSID1_name);
  Serial.print(F("\t")); // tab
  Serial.print("m_PWD1_name = ");
  Serial.print(m_PWD1_name);
  Serial.print(F("\t")); // tab
  Serial.print("m_SSID2_name = ");
  Serial.print(m_SSID2_name);
  Serial.print(F("\t")); // tab
  Serial.print("m_PWD2_name = ");
  Serial.print(m_PWD2_name);
  Serial.print(F("\t")); // tab
  Serial.print("m_SSID3_name = ");
  Serial.print(m_SSID3_name);
  Serial.print(F("\t")); // tab
  Serial.print("m_PWD3_name = ");
  Serial.println(m_PWD3_name);

  File m_ssid_pwd_file_to_write_name = SPIFFS.open("/credentials.JSON", FILE_WRITE);

  if (!m_ssid_pwd_file_to_write_name)
  {
    Serial.println("There was an error opening the pwd/ssid file for writing");
    return;
  }

  Serial.println("Writing this JSON string to pwd/ssid file:");
  Serial.println(SSID_and_pwd_JSON);

  if (!m_ssid_pwd_file_to_write_name.println(SSID_and_pwd_JSON))
  {
    Serial.println("File write failed");
  }
  m_ssid_pwd_file_to_write_name.close();

  request->send(200, "text/html", "<h1>Restarting .....</h1>");
  restartSystem = millis();
}

void handleGetSavSecreteJsonNoReboot(AsyncWebServerRequest *request)
{
  String message;

  String m_SSID1_name;
  String m_SSID2_name;
  String m_SSID3_name;
  String m_PWD1_name;
  String m_PWD2_name;
  String m_PWD3_name;
  String m_temp_string;
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
      m_temp_string = p->name().c_str();
      if (m_temp_string == "ssid1")
      {
        m_SSID1_name = p->value().c_str();
      }
      else if (m_temp_string == "pass1")
      {
        m_PWD1_name = p->value().c_str();
      }
      else if (m_temp_string == "ssid2")
      {
        m_SSID2_name = p->value().c_str();
      }
      else if (m_temp_string == "pass2")
      {
        m_PWD2_name = p->value().c_str();
      }
      else if (m_temp_string == "ssid3")
      {
        m_SSID3_name = p->value().c_str();
      }
      else if (m_temp_string == "pass3")
      {
        m_PWD3_name = p->value().c_str();
      }
    }
  }
  if (request->hasParam(PARAM_MESSAGE, true))
  {
    message = request->getParam(PARAM_MESSAGE, true)->value();
    Serial.println(message);
  }
  else
  {
    message = "No message sent";
  }
  // request->send(200, "text/HTML", "bla bla bla bla bla xyz xyz xyz xyz xyz ");

  // {"SSID1":"myssid1xyz","PWD1":"mypwd1xyz",
  //     "SSID2":"myssid2xyz","PWD2":"mypwd2xyz",
  //     "SSID3":"myssid3xyz","PWD3":"mypwd3xyz"}

  String SSID_and_pwd_JSON = "";
  SSID_and_pwd_JSON += "{\"SSID1\":\"";
  SSID_and_pwd_JSON += m_SSID1_name;
  SSID_and_pwd_JSON += "\",\"PWD1\":\"";
  SSID_and_pwd_JSON += m_PWD1_name;

  SSID_and_pwd_JSON += "\",\"SSID2\":\"";
  SSID_and_pwd_JSON += m_SSID2_name;
  SSID_and_pwd_JSON += "\",\"PWD2\":\"";
  SSID_and_pwd_JSON += m_PWD2_name;

  SSID_and_pwd_JSON += "\",\"SSID3\":\"";
  SSID_and_pwd_JSON += m_SSID3_name;
  SSID_and_pwd_JSON += "\",\"PWD3\":\"";
  SSID_and_pwd_JSON += m_PWD3_name;

  SSID_and_pwd_JSON += "\"}";

  Serial.println("JSON string to write to file = ");
  Serial.println(SSID_and_pwd_JSON);

  Serial.print("m_SSID1_name = ");
  Serial.print(m_SSID1_name);
  Serial.print(F("\t")); // tab
  Serial.print("m_PWD1_name = ");
  Serial.print(m_PWD1_name);
  Serial.print(F("\t")); // tab
  Serial.print("m_SSID2_name = ");
  Serial.print(m_SSID2_name);
  Serial.print(F("\t")); // tab
  Serial.print("m_PWD2_name = ");
  Serial.print(m_PWD2_name);
  Serial.print(F("\t")); // tab
  Serial.print("m_SSID3_name = ");
  Serial.print(m_SSID3_name);
  Serial.print(F("\t")); // tab
  Serial.print("m_PWD3_name = ");
  Serial.println(m_PWD3_name);

  File m_ssid_pwd_file_to_write_name = SPIFFS.open("/credentials.JSON", FILE_WRITE);

  if (!m_ssid_pwd_file_to_write_name)
  {
    Serial.println("There was an error opening the pwd/ssid file for writing");
    return;
  }

  Serial.println("Writing this JSON string to pwd/ssid file:");
  Serial.println(SSID_and_pwd_JSON);

  if (!m_ssid_pwd_file_to_write_name.println(SSID_and_pwd_JSON))
  {
    Serial.println("File write failed");
  }
  m_ssid_pwd_file_to_write_name.close();

  request->send(200, "text/html", "  <head> <meta http-equiv=\"refresh\" content=\"2; URL=wifi.html\" /> <meta name=\"viewport\" content=\"width=device-width, initial-scale=1.0\"> </head> <body> <h1> Credentials stored to flash on NanoStat. </h1>  </body>");
  restartSystem = millis();
}

void handleFileList(AsyncWebServerRequest *request)
{
  Serial.println("handle fle list");
  if (!request->hasParam("dir"))
  {
    request->send(500, "text/plain", "BAD ARGS");
    return;
  }

  AsyncWebParameter *p = request->getParam("dir");
  String path = p->value().c_str();
  Serial.println("handleFileList: " + path);
  String output = "[";

  File root = SPIFFS.open("/", "r");
  if (root.isDirectory())
  {
    Serial.println("here ??");
    File file = root.openNextFile();
    while (file)
    {
      if (output != "[")
      {
        output += ',';
      }
      output += "{\"type\":\"";
      output += (file.isDirectory()) ? "dir" : "file";
      output += "\",\"name\":\"";
      output += String(file.name()).substring(1);
      output += "\"}";
      file = root.openNextFile();
    }
  }

  path = String();
  output += "]";
  Serial.println("Sending file list to client.");
  // Serial.println(output);
  request->send(200, "application/json", output);
}

void handleUpload(AsyncWebServerRequest *request, String filename, String redirect, size_t index, uint8_t *data, size_t len, bool final)
{
  Serial.println("handleUpload called");
  Serial.println(filename);
  Serial.println(redirect);
  File fsUploadFile;
  if (!index)
  {
    if (!filename.startsWith("/"))
      filename = "/" + filename;
    Serial.println((String) "UploadStart: " + filename);
    fsUploadFile = SPIFFS.open(filename, "w"); // Open the file for writing in SPIFFS (create if it doesn't exist)
  }
  for (size_t i = 0; i < len; i++)
  {
    fsUploadFile.write(data[i]);
    // Serial.write(data[i]);
  }
  if (final)
  {
    Serial.println((String) "UploadEnd: " + filename);
    fsUploadFile.close();

    request->send(200, "text/HTML", "  <head> <meta http-equiv=\"refresh\" content=\"2; URL=files.html\" /> <meta name=\"viewport\" content=\"width=device-width, initial-scale=1.0\"> </head> <body> <h1> File uploaded! </h1> <p> Returning to file page. </p> </body>");

    // request->redirect(redirect);
  }
}

// handle the upload of the firmware
void handleFirmwareUpload(AsyncWebServerRequest *request, String filename, size_t index, uint8_t *data, size_t len, bool final)
{
  // handle upload and update
  if (!index)
  {
    Serial.printf("Update: %s\n", filename.c_str());
    if (!Update.begin(UPDATE_SIZE_UNKNOWN))
    { //start with max available size
      Update.printError(Serial);
    }
  }

  /* flashing firmware to ESP*/
  if (len)
  {
    Update.write(data, len);
  }

  if (final)
  {
    if (Update.end(true))
    { //true to set the size to the current progress
      Serial.printf("Update Success: %ub written\nRebooting...\n", index + len);
    }
    else
    {
      Update.printError(Serial);
    }
  }
  // alternative approach
  // https://github.com/me-no-dev/ESPAsyncWebServer/issues/542#issuecomment-508489206
}

// handle the upload of the firmware
void handleFilesystemUpload(AsyncWebServerRequest *request, String filename, size_t index, uint8_t *data, size_t len, bool final)
{
  // handle upload and update
  if (!index)
  {
    Serial.printf("Update: %s\n", filename.c_str());
    // if (!Update.begin(UPDATE_SIZE_UNKNOWN))
    if (!Update.begin(SPIFFS.totalBytes(), U_SPIFFS))
    { //start with max available size
      Update.printError(Serial);
    }
  }

  /* flashing firmware to ESP*/
  if (len)
  {
    Update.write(data, len);
  }

  if (final)
  {
    if (Update.end(true))
    { //true to set the size to the current progress
      Serial.printf("Update Success: %ub written\nRebooting...\n", index + len);
    }
    else
    {
      Update.printError(Serial);
    }
  }
  // alternative approach
  // https://github.com/me-no-dev/ESPAsyncWebServer/issues/542#issuecomment-508489206
}

void getWifiScanJson(AsyncWebServerRequest *request)
{
  String json = "{\"scan_result\":[";
  int n = WiFi.scanComplete();
  if (n == -2)
  {
    WiFi.scanNetworks(true);
  }
  else if (n)
  {
    for (int i = 0; i < n; ++i)
    {
      if (i)
        json += ",";
      json += "{";
      json += "\"RSSI\":" + String(WiFi.RSSI(i));
      json += ",\"SSID\":\"" + WiFi.SSID(i) + "\"";
      json += "}";
    }
    WiFi.scanDelete();
    if (WiFi.scanComplete() == -2)
    {
      WiFi.scanNetworks(true);
    }
  }
  json += "]}";
  request->send(200, "application/json", json);
  json = String();
}

void runWifiPortal()
{

  m_wifitools_server.reset(new AsyncWebServer(80));

  IPAddress myIP;
  myIP = WiFi.softAPIP();
  // myIP = WiFi.localIP();
  Serial.print(F("AP IP address: "));
  Serial.println(myIP);

  // Need to tell server to accept packets from any source with any header via http methods GET, PUT:
  DefaultHeaders::Instance().addHeader("Access-Control-Allow-Origin", "*");
  DefaultHeaders::Instance().addHeader("Access-Control-Allow-Methods", "GET, PUT");
  DefaultHeaders::Instance().addHeader("Access-Control-Allow-Headers", "*");

  m_wifitools_server->serveStatic("/", SPIFFS, "/").setDefaultFile("wifi_index.html");

  // m_wifitools_server->on("/saveSecret/", HTTP_ANY, [&, this](AsyncWebServerRequest *request) {
  //   handleGetSavSecreteJson(request);
  // });

  m_wifitools_server->on("/saveSecret", HTTP_POST, [](AsyncWebServerRequest *request)
                         { handleGetSavSecreteJson(request); });

  m_wifitools_server->on("/list", HTTP_ANY, [](AsyncWebServerRequest *request)
                         { handleFileList(request); });

  m_wifitools_server->on("/wifiScan.json", HTTP_GET, [](AsyncWebServerRequest *request)
                         { getWifiScanJson(request); });

  //xyzxyzxyzxyzxyzxyzxyzxyzxyzxyzxyzxyzxyzxyzxyzxyzxyzxyzxyzxyzxyzxyzxyzxyzxyzxyzxyzxyzxyzxyzxyzxyzxyzxyz

  // m_wifitools_server->on("/saveSecret/", HTTP_ANY, [&, this](AsyncWebServerRequest *request) {
  //   handleGetSavSecreteJson(request);
  // });

  // m_wifitools_server->on("/list", HTTP_ANY, [&, this](AsyncWebServerRequest *request) {
  //   handleFileList(request);
  // });

  // // spiff delete
  // m_wifitools_server->on("/edit", HTTP_DELETE, [&, this](AsyncWebServerRequest *request) {
  //   handleFileDelete(request);
  // });

  // // spiff upload
  // m_wifitools_server->on(
  //     "/edit", HTTP_POST, [&, this](AsyncWebServerRequest *request) {},
  //     [&, this](AsyncWebServerRequest *request, const String &filename, size_t index, uint8_t *data,
  //               size_t len, bool final) {
  //       handleUpload(request, filename, "/wifi_spiffs_admin.html", index, data, len, final);
  //     });

  // m_wifitools_server->on("/wifiScan.json", HTTP_GET, [&, this](AsyncWebServerRequest *request) {
  //   getWifiScanJson(request);
  // });

  // // Simple Firmware Update Form
  // m_wifitools_server->on("/update", HTTP_GET, [&, this](AsyncWebServerRequest *request) {
  //   request->send(SPIFFS, "/wifi_upload.html");
  // });
  // m_wifitools_server->on(
  //     "/update", HTTP_POST, [&, this](AsyncWebServerRequest *request) {
  //       uint8_t isSuccess = !Update.hasError();
  //       if (isSuccess)
  //         restartSystem = millis();
  //       AsyncWebServerResponse *response = request->beginResponse(200, "text/plain", isSuccess ? "OK" : "FAIL");
  //       response->addHeader("Connection", "close");
  //       request->send(response); },
  //     [&, this](AsyncWebServerRequest *request, String filename, size_t index, uint8_t *data, size_t len, bool final) {
  //       if (!index)
  //       {
  //         Serial.printf("Update Start: %s\n", filename.c_str());

  //         if (!Update.begin(UPDATE_SIZE_UNKNOWN))
  //         {
  //           Update.printError(Serial);
  //         }
  //       }
  //       if (!Update.hasError())
  //       {
  //         if (Update.write(data, len) != len)
  //         {
  //           Update.printError(Serial);
  //         }
  //       }
  //       if (final)
  //       {
  //         if (Update.end(true))
  //         {
  //           Serial.printf("Update Success: %uB\n", index + len);
  //         }
  //         else
  //         {
  //           Update.printError(Serial);
  //         }
  //       }
  //     });

  // m_wifitools_server->on("/restart", HTTP_GET, [&, this](AsyncWebServerRequest *request) {
  //   request->send(200, "text/html", "OK");
  //   restartSystem = millis();
  // });

  // m_wifitools_server->onNotFound([](AsyncWebServerRequest *request) {
  //   Serial.println("handle not found");
  //   request->send(404);
  // });

  // m_wifitools_server->addHandler(new CaptiveRequestHandler()).setFilter(ON_AP_FILTER); //only when requested from AP

  //xyzxyzxyzxyzxyzxyzxyzxyzxyzxyzxyzxyzxyzxyzxyzxyzxyzxyzxyzxyzxyzxyzxyzxyzxyzxyzxyzxyzxyzxyzxyzxyzxyzxyz

  Serial.println(F("HTTP server started"));
  m_wifitools_server->begin();
  if (!MDNS.begin("nanostat")) // see https://randomnerdtutorials.com/esp32-access-point-ap-web-server/
  {
    Serial.println("Error setting up MDNS responder !");
    while (1)
      ;
    {
      delay(1000);
    }
  }
  Serial.println("MDNS started.");
  // MDNS.begin("nanostat");
  while (1) // loop until user hits restart... Once credentials saved, won't end up here again unless wifi not connecting!
  {
    process();
  }
}

void runWifiPortal_after_connected_to_WIFI()
{
  // Don't run this after starting server or ESP32 will crash!!!
  server.on("/saveSecret/", HTTP_POST, [](AsyncWebServerRequest *request)
            { handleGetSavSecreteJsonNoReboot(request); });

  //xyzxyzxyzxyzxyzxyzxyzxyzxyzxyzxyzxyzxyzxyzxyzxyzxyzxyzxyzxyzxyzxyzxyzxyzxyzxyzxyzxyzxyzxyzxyzxyzxyzxyz

  // m_wifitools_server->on("/saveSecret/", HTTP_ANY, [&, this](AsyncWebServerRequest *request) {
  //   handleGetSavSecreteJson(request);
  // });

  // m_wifitools_server->on("/list", HTTP_ANY, [&, this](AsyncWebServerRequest *request) {
  //   handleFileList(request);
  // });

  // // spiff delete
  // m_wifitools_server->on("/edit", HTTP_DELETE, [&, this](AsyncWebServerRequest *request) {
  //   handleFileDelete(request);
  // });

  // // spiff upload
  // m_wifitools_server->on(
  //     "/edit", HTTP_POST, [&, this](AsyncWebServerRequest *request) {},
  //     [&, this](AsyncWebServerRequest *request, const String &filename, size_t index, uint8_t *data,
  //               size_t len, bool final) {
  //       handleUpload(request, filename, "/wifi_spiffs_admin.html", index, data, len, final);
  //     });

  // m_wifitools_server->on("/wifiScan.json", HTTP_GET, [&, this](AsyncWebServerRequest *request) {
  //   getWifiScanJson(request);
  // });

  // // Simple Firmware Update Form
  // m_wifitools_server->on("/update", HTTP_GET, [&, this](AsyncWebServerRequest *request) {
  //   request->send(SPIFFS, "/wifi_upload.html");
  // });
  // m_wifitools_server->on(
  //     "/update", HTTP_POST, [&, this](AsyncWebServerRequest *request) {
  //       uint8_t isSuccess = !Update.hasError();
  //       if (isSuccess)
  //         restartSystem = millis();
  //       AsyncWebServerResponse *response = request->beginResponse(200, "text/plain", isSuccess ? "OK" : "FAIL");
  //       response->addHeader("Connection", "close");
  //       request->send(response); },
  //     [&, this](AsyncWebServerRequest *request, String filename, size_t index, uint8_t *data, size_t len, bool final) {
  //       if (!index)
  //       {
  //         Serial.printf("Update Start: %s\n", filename.c_str());

  //         if (!Update.begin(UPDATE_SIZE_UNKNOWN))
  //         {
  //           Update.printError(Serial);
  //         }
  //       }
  //       if (!Update.hasError())
  //       {
  //         if (Update.write(data, len) != len)
  //         {
  //           Update.printError(Serial);
  //         }
  //       }
  //       if (final)
  //       {
  //         if (Update.end(true))
  //         {
  //           Serial.printf("Update Success: %uB\n", index + len);
  //         }
  //         else
  //         {
  //           Update.printError(Serial);
  //         }
  //       }
  //     });

  // m_wifitools_server->on("/restart", HTTP_GET, [&, this](AsyncWebServerRequest *request) {
  //   request->send(200, "text/html", "OK");
  //   restartSystem = millis();
  // });

  // m_wifitools_server->onNotFound([](AsyncWebServerRequest *request) {
  //   Serial.println("handle not found");
  //   request->send(404);
  // });

  // m_wifitools_server->addHandler(new CaptiveRequestHandler()).setFilter(ON_AP_FILTER); //only when requested from AP

  //xyzxyzxyzxyzxyzxyzxyzxyzxyzxyzxyzxyzxyzxyzxyzxyzxyzxyzxyzxyzxyzxyzxyzxyzxyzxyzxyzxyzxyzxyzxyzxyzxyzxyz

  Serial.println(F("HTTP server started"));
  m_wifitools_server->begin();
  if (!MDNS.begin("nanostat")) // see https://randomnerdtutorials.com/esp32-access-point-ap-web-server/
  {
    Serial.println("Error setting up MDNS responder !");
    while (1)
      ;
    {
      delay(1000);
    }
  }
  Serial.println("MDNS started.");
  // MDNS.begin("nanostat");
  while (1) // loop until user hits restart... Once credentials saved, won't end up here again unless wifi not connecting!
  {
    process();
  }
}

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

    Serial.print("  FILE: ");
    Serial.print(file.name());
    Serial.print("\tSIZE: ");
    Serial.println(file.size());

    file = root.openNextFile();
  }
}

void reset_Voltammogram_arrays()
{
  for (uint16_t i = 0; i < arr_samples; i++)
  {
    volts[i] = 0;
    amps[i] = 0;
    time_Voltammaogram[i] = 0;
  }
}

inline void saveVoltammogram(float voltage, float current, bool debug)
{
  //@param        voltage: voltage or time depending on type of experiment
  //                       voltage for voltammetry, time for time for
  //                       time evolution experiments like chronoamperometry
  //@param        current: current from the electrochemical cell

  //
  volts[arr_cur_index] = (int16_t)voltage;
  amps[arr_cur_index] = current;
  time_Voltammaogram[arr_cur_index] = millis();
  arr_cur_index++;
  number_of_valid_points_in_volts_amps_array++;

  if (debug)
  {
    Serial.println(" ");
    Serial.println("****************** SAVING TO VOLTAMMOGRAM******************");
    Serial.print("arr_cur_index=");
    Serial.println(arr_cur_index);
    Serial.print(voltage);
    Serial.print(F("\t"));
    Serial.print(current, 5);
    Serial.print(F("\t"));
    Serial.println(micros());
    Serial.println("****************** SAVED TO VOLTAMMOGRAM******************");
    Serial.println(" ");
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
  // Measures IV curve.

  float i_forward = 0;
  uint32_t voltage_step;
  lastTime = millis();
  //Reset Arrays
  reset_Voltammogram_arrays(); // sets volt[i], amps[i],time_Voltammaogram[i]=0 all
  arr_cur_index = 0;
  number_of_valid_points_in_volts_amps_array = 0;

  voltage_step = (endV - startV) / numPoints;
  if (voltage_step == 0)
  {
    voltage_step = 1; // 1 mV minimum
  }

  Serial.println("******************************************************");
  Serial.println("Starting IV sweep with these parameters:");
  Serial.println(startV);
  Serial.println(endV);
  Serial.println(numPoints);
  Serial.println(voltage_step);
  Serial.print("Heap free memory (in bytes)= ");
  Serial.println(ESP.getFreeHeap());

  // Initialize LMP91000:
  initLMP(LMPgainGLOBAL);
  setLMPBias(startV);
  setVoltage(startV);

  //xxxxxxxxxxxxxxxxxxxxxxxxxx
  if (print_output_to_serial)
  {
    Serial.println("RUNNING TEST IV:");
    Serial.print("a_coeff =");
    Serial.println(a_coeff);
    Serial.print("b_coeff =");
    Serial.println(b_coeff);
    Serial.print("startV =");
    Serial.println(startV);
    Serial.print("endV =");
    Serial.println(endV);
    Serial.print("numPoints =");
    Serial.println(numPoints);
    Serial.print("delayTime_ms =");
    Serial.println(delayTime_ms);
    Serial.print("voltage_step =");
    Serial.println(voltage_step);

    Serial.println("Column header meanings:");
    Serial.println("BIAS:");
    Serial.println("T = time in ms");
    Serial.println("Vc = desired cell voltage in mV (internal variable name = voltage)");
    Serial.println("Vset = set cell voltage in mV (internal variable name = vset = dacVout * TIA_BIAS[bias_setting])");
    Serial.println("div = percentage scale (internal variable name = TIA_BIAS[bias_setting])");
    Serial.println("Vdac = ESP32 dac voltage in mV (internal variable name = dacVout)");
    Serial.println("adcbits = analogread(LMP)");
    Serial.println("Vout = TIA output in mV (internal variable name = v1 = pStat.getVoltage(analogRead(LMP), opVolt, adcBits);");
    Serial.println("Vc1 = internal zero in mV also C1 (internal variable name = v2 = dacVout * .5)");
    Serial.println("i_f = forward current in uA (internal variable name = current = (((v1 - v2) / 1000) / TIA_GAIN[LMPgain - 1]) * pow(10, 6); //scales to uA)");

    // Serial.println("xyz = xyz in xyz (internal variable name = xyz)");
    // Serial.println("xyz = xyz in xyz (internal variable name = v)");

    Serial.println("T\tVc\tVset\tdiv\tVdac\tadcbits\tVout\tVc1\ti_f");
    // Serial.println("T,Vc,Vset,div,Vdac,adcbits,Vout,Vc1,i_f");
  }

  for (int16_t this_voltage = startV; this_voltage <= endV; this_voltage += voltage_step)
  {
    i_forward = biasAndSample(this_voltage, delayTime_ms);
    saveVoltammogram(this_voltage, i_forward, false);
    if (print_output_to_serial)
    {
      Serial.println("EOL");
    }
    if (userpause) //will hold the code here until a character is sent over the Serial port
    {
      while (!Serial.available())
        ;
      Serial.read();
    }
  }
  setOutputsToZero();
  Serial.println("Finished IV sweep.");
  Serial.print("Heap free memory (in bytes)= ");
  Serial.println(ESP.getFreeHeap());
  Serial.println("******************************************************");
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
  reset_Voltammogram_arrays(); // sets volt[i], amps[i],time_Voltammaogram[i]=0 all
  number_of_valid_points_in_volts_amps_array = 0;
  arr_cur_index = 0;

  // local variables for statistics:
  float adc_bits_array[numPoints];
  Serial.print("Heap free memory (in bytes)= ");
  Serial.println(ESP.getFreeHeap());
  float adc_bits_array_sum = 0;
  float adc_bits_array_avg = 0;
  float adc_bits_array_std_dev = 0;
  float adc_bits_minus_avg_squared_sum = 0;
  int num_readings_to_average = 1;

  // which num point to avg:
  float num_points_to_average[7] = {1, 5, 10, 50, 100, 500, 1000};

  // set LMP91000 and bias
  initLMP(LMPgainGLOBAL);
  setLMPBias(biasV);
  setVoltage(biasV);

  if (print_output_to_serial)
  {
    Serial.print("num_rdgs");
    Serial.print(F("\t"));

    Serial.print("adc_avg");
    Serial.print(F("\t"));

    Serial.println("adc_std_dev");
  };

  for (int16_t i = 0; i < 7; i += 1) // iterate over # of pts per reading
  {
    adc_bits_array_sum = 0;
    adc_bits_array_avg = 0;
    adc_bits_array_std_dev = 0;
    adc_bits_minus_avg_squared_sum = 0;
    num_readings_to_average = num_points_to_average[i];

    for (int16_t j = 0; j < numPoints; j += 1) // read the adc data
    {
      //handle pulsing and delay
      if (delayTime_ms > 10) // we can afford the time to pulse the LED for 10 ms, just waiting anyways
      {
        pulseLED_on_off(LEDPIN, 10); // signify start of a new data point
        delay(delayTime_ms - 10);
      }
      else if (delayTime_ms < 10) // we cannot afford the time to pulse the LED for 10 ms, so pulse 1 ms only
      {
        if (delayTime_ms > 2)
        {
          pulseLED_on_off(LEDPIN, 1); // signify start of a new data point
          delay(delayTime_ms - 1);
        }
        else if (delayTime_ms < 2)
        { // no LED pulse too fast....
          delay(delayTime_ms);
        }
      }
      adc_bits_array[j] = analog_read_avg(num_readings_to_average, LMP);
      //pulseLED_on_off(LEDPIN, 10);
      // Now populate Volts array etc:
      volts[arr_cur_index] = biasV;
      time_Voltammaogram[arr_cur_index] = millis();
      amps[arr_cur_index] = adc_bits_array[j];
      arr_cur_index++;
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

    if (print_output_to_serial)
    {
      Serial.print(num_readings_to_average);
      Serial.print(F("\t"));

      Serial.print(adc_bits_array_avg);
      Serial.print(F("\t"));

      Serial.println(adc_bits_array_std_dev);
    }
  }
}

void testDACs(uint32_t delayTime_ms)
{

  // steps ESP32 DAC from 0 to 255 digital so you can check it on a multimeter

  //xxxxxxxxxxxxxxxxxxxxxxxxxx

  initLMP(LMPgainGLOBAL);
  setLMPBias(100);
  setVoltage(100);

  //xxxxxxxxxxxxxxxxxxxxxxxxxx
  Serial.println("RUNNING TEST DAC:");

  Serial.println("i\tdacgoal");

  float dacgoal;
  for (int16_t j = 0; j <= 255; j += 1)
  {
    dacWrite(dac, j); // sets ESP32 DAC bits
    dacgoal = 3300.0 * j / 255;
    Serial.print(j); // current time in ms
    Serial.print(F("\t"));
    Serial.println(dacgoal); // desired cell voltage
    if (userpause)           //will hold the code here until a character is sent over the Serial port
    {
      while (!Serial.available())
        ;
      Serial.read();
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

  initLMP(LMPgainGLOBAL);
  setLMPBias(100);
  setVoltage(100);

  //xxxxxxxxxxxxxxxxxxxxxxxxxx
  Serial.println("RUNNING TEST DAC:");

  Serial.println("i\tdacgoal\tadc_voltage xyz");

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
    Serial.print(j); // current time in ms
    Serial.print(F("\t"));
    Serial.print(dacgoal); // desired cell voltage
    Serial.print(F("\t"));
    Serial.println(adc_voltage); // adc voltage

    if (userpause) //will hold the code here until a character is sent over the Serial port
    {
      while (!Serial.available())
        ;
      Serial.read();
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
  pStat.setGain(LMPgainGLOBAL); // TIA feedback resistor , global variable
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
  Serial.println("RUNNING TEST ADC CAL LMP91000:");
  Serial.println("TIA_BIAS[0]=");
  Serial.println(TIA_BIAS[0]);

  // int adc_int_from_dac_int[140]; // for dac 116 to 255, what adc reads
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
    // adc_int_from_dac_int[j - 116] = this_voltage_adc;
    dacgoal = 3300.0 * j / 255;
    if (print_output_to_serial)
    {
      Serial.print(j);
      Serial.print(F("\t"));
      Serial.print(dacgoal);
      Serial.print(F("\t"));
      Serial.println(this_voltage_adc); // desired cell voltage
    }

    if (userpause) //will hold the code here until a character is sent over the Serial port
    {
      while (!Serial.available())
        ;
      Serial.read();
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
  Serial.println("y=a + bx");
  Serial.println("y=dac");
  Serial.println("y=adc");
  Serial.print("a=");
  Serial.println(a);
  Serial.print("b=");
  Serial.println(b);
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

  Serial.println("RUNNING TEST LMP91000:");
  Serial.println("TIA_BIAS[bias_setting_local]=");
  Serial.println(TIA_BIAS[bias_setting_local]);

  for (int16_t i = 0; i <= 13; i += 1) //loop over percentage of Vref applied to cell
  {
    pStat.setBias(i);
    Serial.println("****************************************");
    Serial.println("TIA_BIAS[i]=");
    Serial.println(TIA_BIAS[i]);
    if (userpause) //will hold the code here until a character is sent over the Serial port
    {
      while (!Serial.available())
        ;
      Serial.read();
    }
    if (loop_dac)
    {
      for (int16_t j = 0; j <= 255; j += 1)
      {
        dacWrite(dac, j); // sets ESP32 DAC bits
        dacgoal = 3300.0 * j / 255;
        Serial.print(j); // current time in ms
        Serial.print(F("\t"));
        Serial.println(dacgoal); // desired cell voltage
        if (userpause)           //will hold the code here until a character is sent over the Serial port
        {
          while (!Serial.available())
            ;
          Serial.read();
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
    if (userpause) //will hold the code here until a character is sent over the Serial port
    {
      while (!Serial.available())
        ;
      Serial.read();
    }
    i_backward = biasAndSample(startV, off_time); // xxx comment out to do only forward bias

    saveVoltammogram(j, i_forward - i_backward, false); // this will print voltage, current
    if (print_output_to_serial)
    {
      Serial.println("EOL");
    }
    // Serial.println();
    // if (userpause) //will hold the code here until a character is sent over the Serial port
    // {
    //   while (!Serial.available())
    //     ;
    //   Serial.read();
    // }
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
    //will hold the code here until a character is sent over the Serial port
    //this ensures the experiment will only run when initiated
    while (!Serial.available())
      ;
    Serial.read();
    i_backward = biasAndSample(startV, off_time);
    saveVoltammogram(j, i_forward - i_backward, false); // this will print voltage, current

    //will hold the code here until a character is sent over the Serial port
    //this ensures the experiment will only run when initiated
    while (!Serial.available())
      ;
    Serial.read();
    saveVoltammogram(j, i_forward - i_backward, true); // this will print voltage, currnt
    // Serial.println();
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
  reset_Voltammogram_arrays(); // sets volt[i], amps[i],time_Voltammaogram[i]=0 all

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

    Serial.println("Column header meanings:");
    Serial.println("FORWARD BIAS:");
    Serial.println("T = time in ms");
    Serial.println("Vc = desired cell voltage in mV (internal variable name = voltage)");
    Serial.println("Vset = set cell voltage in mV (internal variable name = vset = dacVout * TIA_BIAS[bias_setting])");
    Serial.println("div = percentage scale (internal variable name = TIA_BIAS[bias_setting])");
    Serial.println("Vdac = ESP32 dac voltage in mV (internal variable name = dacVout)");
    Serial.println("adc_bits = TIA output in adc_bits;");
    Serial.println("(V1) Vout = TIA output in mV (internal variable name = v1 = pStat.getVoltage(analogRead(LMP), opVolt, adcBits);");
    Serial.println("(V2) Vc1 = internal zero in mV also C1 (internal variable name = v2 = dacVout * .5)");
    Serial.println("i_f = forward current in uA (internal variable name = current = (((v1 - v2) / 1000) / TIA_GAIN[LMPgain - 1]) * pow(10, 6); //scales to uA)");

    Serial.println("REVERSE BIAS:");
    Serial.println("T = time in ms");
    Serial.println("Vc = desired cell voltage in mV (internal variable name = voltage)");
    Serial.println("Vset = set cell voltage in mV (internal variable name = vset = dacVout * TIA_BIAS[bias_setting])");
    Serial.println("div = percentage scale (internal variable name = TIA_BIAS[bias_setting])");
    Serial.println("Vdac = ESP32 dac voltage in mV (internal variable name = dacVout)");
    Serial.println("Vout = TIA output in mV (internal variable name = v1 = pStat.getVoltage(analogRead(LMP), opVolt, adcBits);");
    Serial.println("Vc1 = internal zero in mV also C1 (internal variable name = v2 = dacVout * .5)");
    Serial.println("i_R = reverse current in uA (internal variable name = current = (((v1 - v2) / 1000) / TIA_GAIN[LMPgain - 1]) * pow(10, 6); //scales to uA)");

    Serial.println("RESPONSE:");
    Serial.println("Vc = desired cell voltage in mV (internal variable name = voltage FORWARD, should be FORWARD-REVERSE...)");
    Serial.println("I = i_f - i_R in uA (internal variable name = i_forward - i_backward)");
    // Serial.println("xyz = xyz in xyz (internal variable name = xyz)");
    // Serial.println("xyz = xyz in xyz (internal variable name = v)");

    Serial.println("T\tVc\tVset\tdiv\tVdac\tadcbits\tVout\tVc1\ti_f\tT\tVc\tVset\tdiv\tVdac\tadcbits\tVout\tVc1\ti_R\tV\tI\tT");

    // Serial.println("T,Vc,Vset,div,Vdac,adcbits,Vout,Vc1,i_f,T,Vc,Vset,div,Vdac,adcbits,Vout,Vc1,i_R,V,I,T");

    //    Serial.println("T\tVc\tVset\tdiv\tVdac\tVout\tVc1\ti_f\tT\tVc\tVset\tdiv\tVdac\tVout\tVc1\ti_R\tVc\tI");
    //  Serial.println("T\tVc\tVdac\tVout\tVc1\ti_f\tT\tVc\tZ\tVout\ti_R\tLMP\tI");
    //  Serial.println("T\tVc\tVout\tVc1\ti_f\tT\tVc\tZ\tVout\ti_R\tLMP\tI");
    //  Serial.println("T\t\tVc\tVout\tVc1\ti_f\tT\tVc\tZ\tVout\ti_R\tLMP\tI");
    //  Serial.println("T(ms)\tVcell(mV)\tZero(mV)\tLMP\ti_f\tT(ms)\tVcell(mV)\tZero(mV)\tLMP\ti_R\tV(mV)\tI");
    //  Serial.println(F("Time(ms)	Cell set voltage (mV)	Zero(mV)	dacVout	TIA_BIAS[bias_setting],LMP i.e. Vout (mV),VC1(mV),i_forward,Time(ms),Cell set voltage (mV),Zero(mV),dacVout,TIA_BIAS[bias_setting],LMP i.e. Vout (mV),VC1(mV),i_backward,Voltage(mV),Current"));
    //  Serial.println(F("T(ms),Vcell(mV),Zero(mV),dacVout,TIA_BIAS,Vout(mV),VC1(mV),i_f,Time(ms),Cell set voltage (mV),Zero(mV),dacVout,TIA_BIAS[bias_setting],LMP i.e. Vout (mV),VC1(mV),i_backward,Voltage(mV),Current"));
  }

  initLMP(lmpGain);
  pulseAmp = abs(pulseAmp);
  uint32_t off_time = pulse_period - pulse_width;

  setLMPBias(startV); // -200 mV to start
  setVoltage(startV); // -200 mV to start
  // Serial.println();

  unsigned long time1 = millis();
  while (millis() - time1 < quietTime)
    ;

  if (startV < endV)
  {

    runNPVForward(startV, endV, pulseAmp, pulse_width, off_time);
  }
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
  arr_cur_index = 0;

  for (uint8_t i = 0; i < cycles; i++)
  {
    //j starts at startV
    //    for (j;j <= vertex1; j += stepV)
    for (; j <= vertex1; j += stepV)
    {
      i_cv = biasAndSample(j, rate);
      // Serial.print(i + 1);
      // Serial.print(F(","));
      saveVoltammogram(j, i_cv, false);
      if (print_output_to_serial)
      {
        Serial.println("EOL");
      }
      // Serial.println();
    }
    j -= 2 * stepV; //increment j twice to avoid biasing at the vertex twice

    //j starts right below the first vertex
    //    for (j; j >= vertex2; j -= stepV)
    for (; j >= vertex2; j -= stepV)
    {
      i_cv = biasAndSample(j, rate);
      // Serial.print(i + 1);
      // Serial.print(F(","));
      saveVoltammogram(j, i_cv, false);
      if (print_output_to_serial)
      {
        Serial.println("EOL");
      }
      // Serial.println();
    }
    j += 2 * stepV; //increment j twice to avoid biasing at the vertex twice

    //j starts right above the second vertex
    //for (j; j <= endV; j += stepV)
    for (; j <= endV; j += stepV)
    {
      i_cv = biasAndSample(j, rate);
      // Serial.print(i + 1);
      // Serial.print(F(","));
      saveVoltammogram(j, i_cv, false);
      if (print_output_to_serial)
      {
        Serial.println("EOL");
      }
      // Serial.println();
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
  number_of_valid_points_in_volts_amps_array = 0;
  arr_cur_index = 0;

  for (uint8_t i = 0; i < cycles; i++)
  {
    //j starts at startV
    //    for (j; j >= vertex1; j -= stepV)
    for (; j >= vertex1; j -= stepV)
    {
      i_cv = biasAndSample(j, rate);
      // Serial.print(i + 1);
      // Serial.print(F(","));
      saveVoltammogram(j, i_cv, false);
      if (print_output_to_serial)
      {
        Serial.println("EOL");
      }
      // Serial.println();
    }
    j += 2 * stepV; //increment j twice to avoid biasing at the vertex twice

    //j starts right above vertex1
    //    for (j; j <= vertex2; j += stepV)
    for (; j <= vertex2; j += stepV)
    {
      i_cv = biasAndSample(j, rate);
      // Serial.print(i + 1);
      // Serial.print(F(","));
      saveVoltammogram(j, i_cv, false);
      if (print_output_to_serial)
      {
        Serial.println("EOL");
      }
      // Serial.println();
    }
    j -= 2 * stepV; //increment j twice to avoid biasing at the vertex twice

    //j starts right below vertex2
    //for (j; j >= endV; j -= stepV)
    for (; j >= endV; j -= stepV)
    {
      i_cv = biasAndSample(j, rate);
      // Serial.print(i + 1);
      // Serial.print(F(","));
      saveVoltammogram(j, i_cv, false);
      if (print_output_to_serial)
      {
        Serial.println("EOL");
      }
      // Serial.println();
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
  if (print_output_to_serial)
  {

    Serial.println(F("Time(ms),Zero,LMP,Current,Cycle,Voltage,Current"));
  }

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
  reset_Voltammogram_arrays(); // sets volt[i], amps[i],time_Voltammaogram[i]=0 all

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
    saveVoltammogram(j, i_forward - i_backward, false);
    if (print_output_to_serial)
    {
      Serial.println("EOL");
    }
    // Serial.println();
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
    number_of_valid_points_in_volts_amps_array++;
    saveVoltammogram(j, i_forward - i_backward, false);
    if (print_output_to_serial)
    {
      Serial.println("EOL");
    }
    // Serial.println();
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
  // Serial.println(F("Time(ms),Zero,LMP,Time(ms),Zero,LMP,Voltage,Current"));

  initLMP(lmpGain);
  stepV = abs(stepV);
  pulseAmp = abs(pulseAmp);
  freq = (uint16_t)(1000.0 / (2 * freq)); //converts frequency to milliseconds

  //Reset Arrays
  reset_Voltammogram_arrays(); // sets volt[i], amps[i],time_Voltammaogram[i]=0 all

  arr_cur_index = 0;
  number_of_valid_points_in_volts_amps_array = 0;

  //testing shows that time is usually off by 1 ms or so, therefore
  //we subtract 1 ms from the calculated rate to compensate
  freq -= 1;

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

    Serial.println("Column header meanings:");
    Serial.println("FORWARD BIAS:");
    Serial.println("T = time in ms");
    Serial.println("Vc = desired cell voltage in mV (internal variable name = voltage)");
    Serial.println("Vset = set cell voltage in mV (internal variable name = vset = dacVout * TIA_BIAS[bias_setting])");
    Serial.println("div = percentage scale (internal variable name = TIA_BIAS[bias_setting])");
    Serial.println("Vdac = ESP32 dac voltage in mV (internal variable name = dacVout)");
    Serial.println("adc_bits = TIA output in adc_bits;");
    Serial.println("(V1) Vout = TIA output in mV (internal variable name = v1 = pStat.getVoltage(analogRead(LMP), opVolt, adcBits);");
    Serial.println("(V2) Vc1 = internal zero in mV also C1 (internal variable name = v2 = dacVout * .5)");
    Serial.println("i_f = forward current in uA (internal variable name = current = (((v1 - v2) / 1000) / TIA_GAIN[LMPgain - 1]) * pow(10, 6); //scales to uA)");

    Serial.println("REVERSE BIAS:");
    Serial.println("T = time in ms");
    Serial.println("Vc = desired cell voltage in mV (internal variable name = voltage)");
    Serial.println("Vset = set cell voltage in mV (internal variable name = vset = dacVout * TIA_BIAS[bias_setting])");
    Serial.println("div = percentage scale (internal variable name = TIA_BIAS[bias_setting])");
    Serial.println("Vdac = ESP32 dac voltage in mV (internal variable name = dacVout)");
    Serial.println("Vout = TIA output in mV (internal variable name = v1 = pStat.getVoltage(analogRead(LMP), opVolt, adcBits);");
    Serial.println("Vc1 = internal zero in mV also C1 (internal variable name = v2 = dacVout * .5)");
    Serial.println("i_R = reverse current in uA (internal variable name = current = (((v1 - v2) / 1000) / TIA_GAIN[LMPgain - 1]) * pow(10, 6); //scales to uA)");

    Serial.println("RESPONSE:");
    Serial.println("Vc = desired cell voltage in mV (internal variable name = voltage FORWARD, should be FORWARD-REVERSE...)");
    Serial.println("I = i_f - i_R in uA (internal variable name = i_forward - i_backward)");
    Serial.println("xyz = xyz in xyz (internal variable name = xyz)");
    Serial.println("xyz = xyz in xyz (internal variable name = v)");

    // Serial.println("T,Vc,Vset,div,Vdac,adcbits,Vout,Vc1,i_f,T,Vc,Vset,div,Vdac,adcbits,Vout,Vc1,i_R,V,I,T");
    Serial.println("T\tVc\tVset\tdiv\tVdac\tadcbits\tVout\tVc1\ti_f\tT\tVc\tVset\tdiv\tVdac\tadcbits\tVout\tVc1\ti_R\tV\tI\tT");

    //    Serial.println("T\tVc\tVset\tdiv\tVdac\tVout\tVc1\ti_f\tT\tVc\tVset\tdiv\tVdac\tVout\tVc1\ti_R\tVc\tI");
    //  Serial.println("T\tVc\tVdac\tVout\tVc1\ti_f\tT\tVc\tZ\tVout\ti_R\tLMP\tI");
    //  Serial.println("T\tVc\tVout\tVc1\ti_f\tT\tVc\tZ\tVout\ti_R\tLMP\tI");
    //  Serial.println("T\t\tVc\tVout\tVc1\ti_f\tT\tVc\tZ\tVout\ti_R\tLMP\tI");
    //  Serial.println("T(ms)\tVcell(mV)\tZero(mV)\tLMP\ti_f\tT(ms)\tVcell(mV)\tZero(mV)\tLMP\ti_R\tV(mV)\tI");
    //  Serial.println(F("Time(ms),Cell set voltage (mV),Zero(mV),dacVout,TIA_BIAS[bias_setting],LMP i.e. Vout (mV),VC1(mV),i_forward,Time(ms),Cell set voltage (mV),Zero(mV),dacVout,TIA_BIAS[bias_setting],LMP i.e. Vout (mV),VC1(mV),i_backward,Voltage(mV),Current"));
    //  Serial.println(F("T(ms),Vcell(mV),Zero(mV),dacVout,TIA_BIAS,Vout(mV),VC1(mV),i_f,Time(ms),Cell set voltage (mV),Zero(mV),dacVout,TIA_BIAS[bias_setting],LMP i.e. Vout (mV),VC1(mV),i_backward,Voltage(mV),Current"));
  }

  //Reset Arrays
  reset_Voltammogram_arrays(); // sets volt[i], amps[i],time_Voltammaogram[i]=0 all

  number_of_valid_points_in_volts_amps_array = 0;
  arr_cur_index = 0;

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
    number_of_valid_points_in_volts_amps_array++;

    saveVoltammogram(j, i_forward - i_backward, false);
    // Serial.println();
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
    number_of_valid_points_in_volts_amps_array++;

    saveVoltammogram(j, i_forward - i_backward, false);
    // Serial.println();
  }
}

void runDPV(uint8_t lmpGain, int16_t startV, int16_t endV,
            int8_t pulseAmp, int8_t stepV, uint32_t pulse_period,
            uint32_t pulse_width, bool setToZero)
{
  //Serial.println("Time(ms),Zero(mV),LMP,Voltage(mV)," + current);
  if (print_output_to_serial)
  {
    Serial.println(F("Time(ms),Zero(mV),LMP,Voltage(mV),Current"));
  }

  initLMP(lmpGain);
  stepV = abs(stepV);
  pulseAmp = abs(pulseAmp);
  uint32_t off_time = pulse_period - pulse_width;

  //Reset Arrays
  reset_Voltammogram_arrays(); // sets volt[i], amps[i],time_Voltammaogram[i]=0 all

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
  digitalWrite(LEDPIN, HIGH); // just leave it on during sweep
  initLMP(lmpGain);

  reset_Voltammogram_arrays(); // Reset Arrays volt[i], amps[i],time_Voltammaogram[i]=0 all
  arr_cur_index = 0;
  number_of_valid_points_in_volts_amps_array = 0;

  float a = a_coeff; //    //  Calibrate coefficients (make a local copy of the global ones):
  float b = b_coeff; //    //  Calibrate coefficients (make a local copy of the global ones):

  // default samples=100
  //  if (samples > arr_samples / 3){ // max 2500/3=833; default 100
  if (samples > (arr_samples/3)) //  arr_samples is max size of amps,volts array
  { // max 2500/3=833; default 100
    // samples = (float)arr_samples / 3.0;
    samples = arr_samples/3;
    Serial.print("Samples>arr_samples, adjusting accordingly to samples = ");
    Serial.println(samples);
  }

  int16_t voltageArray[3] = {pre_stepV, v1, v2}; // default 50, 50, 50
  uint32_t timeArray[3] = {quietTime, t1, t2};   // default 2000,2000,2000

  //i = 0 is pre-step voltage
  //i = 1 is first step potential
  //i = 2 is second step potential
  for (uint8_t i = 0; i < 3; i++)
  {
    //the time between samples is determined by the
    //number of samples inputted by the user
    uint32_t fs = (double)timeArray[i] / samples; // default 2000/100=20
    unsigned long startTime = millis();

    Serial.print("samples=");
    Serial.println(samples);
    Serial.print("fs=");
    Serial.println(fs);

    //Print column headers
    if (print_output_to_serial)
    {
      Serial.println("Voltage(mV),Time(ms), Current(microA)");
    }

    //set bias potential
    setLMPBias(voltageArray[i]);
    setVoltage(voltageArray[i]);

    while (millis() - startTime < timeArray[i]) // default 2000, so for 2 seconds
    {
      // Serial.print("arr_cur_index = ");
      // Serial.print(arr_cur_index);
      // Serial.print(F("\t")); // tab
      // Serial.print("millis = ");
      // Serial.print(millis());
      // Serial.print(F("\t")); // tab
      // Serial.print("startTime = ");
      // Serial.print(startTime);
      // Serial.print(F("\t")); // tab
      // Serial.print("timeArray[i] = ");
      // Serial.print(timeArray[i]);
      // Serial.println(F("\t")); // tab

      // Read output voltage of the transimpedance amplifier
      int adc_bits;                                                 // will be output voltage of TIA amplifier, "VOUT" on LMP91000 diagram, also C2
      adc_bits = analog_read_avg(num_adc_readings_to_average, LMP); //  hard wired in BurkeLab ESP32Stat Rev 3.5 to LMP i.e ESP32 pin 32 (ADC1_CH7)
      float v1;
      v1 = (3.3 / 255.0) * (1 / (2.0 * b)) * (float)adc_bits - (a / (2.0 * b)) * (3.3 / 255.0); // LMP is wired to Vout of the LMP91000
      v1 = v1 * 1000.0;
      float v2 = dacVout * .5; //the zero of the internal transimpedance amplifier
      // V2 is not measured in  BurkeLab ESP32Stat Rev 3.5 and assumed to be half dacVout, calibration helps this see above
      float current = 0;
      if (lmpGain == 0)                                    // using external feedback resistor, not (yet) suppored with BurkeLab ESP32Stat Rev 3.5
        current = (((v1 - v2) / 1000) / RFB) * pow(10, 9); //scales to nA
      else
        current = (((v1 - v2) / 1000) / TIA_GAIN[lmpGain - 1]) * pow(10, 6); //scales to uA
                                                                             //Sample and save data
      volts[arr_cur_index] = voltageArray[i];
      time_Voltammaogram[arr_cur_index] = millis();
      amps[arr_cur_index] = current;
      number_of_valid_points_in_volts_amps_array++;
      if (print_output_to_serial)
      // if (false)
      {
        Serial.println("************BEGIN CA POINT:*******************");
        // Serial.print("adc_bits= ");
        // Serial.print(adc_bits);
        // Serial.print(F("\t")); // tab
        // Serial.print("v1= ");
        // Serial.print(v1);
        // Serial.print(F("\t")); // tab
        // Serial.print("dacVout= ");
        // Serial.print(dacVout);
        // Serial.print(F("\t")); // tab
        // Serial.print("v2= ");
        // Serial.print(v2);
        // Serial.print(F("\t")); // tab
        // Serial.print("current= ");
        // Serial.print(current);
        // Serial.print(F("\t")); // tab
        // Serial.print("lmpGain= ");
        // Serial.print(lmpGain);
        // Serial.print(F("\t")); // tab
        Serial.print("arr_cur_index = ");
        Serial.print(arr_cur_index);
        Serial.print(F("\t")); // tab
        Serial.print("Volts = ");
        Serial.print(volts[arr_cur_index]);
        Serial.print(F("\t")); // tab
        Serial.print("Amps = ");
        Serial.print(amps[arr_cur_index]);
        Serial.print(F("\t")); // tab
        Serial.print("Time = ");
        Serial.print(time_Voltammaogram[arr_cur_index]);
        // Serial.print(F("\t")); // tab
        // Serial.print("TIA_BIAS[bias_setting] = ");
        // Serial.print(TIA_BIAS[bias_setting]);
        // Serial.print(F("\t")); // tab
        // Serial.println();
        // Serial.print("TIA_GAIN[lmpGain - 1] = ");
        // Serial.print(TIA_GAIN[lmpGain - 1]);
        // Serial.print(F("\t")); // tab
        Serial.println();

        Serial.println("**************END CA POINT*****************");
      }

      // if (print_output_to_serial)
      if (false)
      {
        Serial.print("Volts = ");
        Serial.print(volts[arr_cur_index]);
        Serial.print(F("\t")); // tab
        Serial.print("Amps = ");
        Serial.print(amps[arr_cur_index]);
        Serial.print(F("\t")); // tab
        Serial.print("Time = ");
        Serial.print(time_Voltammaogram[arr_cur_index]);
        Serial.print(F("\t")); // tab
        Serial.println();
      }

      arr_cur_index++;
      delay(fs);
    }
    digitalWrite(LEDPIN, HIGH); // off at end of sweep
  }

  arr_cur_index = 0;
  if (setToZero)
    setOutputsToZero();
}

void runNPVandPrintToSerial()
// runNPVandPrintToSerial
{

  //##############################NORMAL PULSE VOLTAMMETRY##############################
  LMPgainGLOBAL = 7;
  runNPV(LMPgainGLOBAL, -200, 500, 10, 50, 200, 500, 6, true);
  // Run Normal Pulse Voltammetry with gain 350000, -200 to + 500 mV, 10 mV step, 50 microsecond width,
  //  200 microsecond period, 500 microsecond quiet time, range micro amps) // micro or milli???

  Serial.println(F("Voltage,Current")); // the final array
  for (uint16_t i = 0; i < arr_samples; i++)
  {
    Serial.print(volts[i]);
    Serial.print(F("\t"));
    Serial.println(amps[i]);
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

  Serial.println(F("Voltage,Current")); // the final array
  for (uint16_t i = 0; i < arr_samples; i++)
  {
    Serial.print(volts[i]);
    Serial.print(F("\t"));
    Serial.println(amps[i]);
  }
}

void runSWVForwardandPrintToSerial()
// runSWVandPrintToSerial (forward)
{
  //  ##############################SQUARE WAVE VOLTAMMETRY (Forward -- Oxidation)##############################

  //  runSWV(LMPgain, -400, 500, 50, 1, 31.25, true);

  Serial.println(F("Voltage,Current"));
  for (uint16_t i = 0; i < arr_samples; i++)
  {
    Serial.print(volts[i]);
    Serial.print(F(","));
    Serial.println(amps[i]);
  }
}

void runSWVReverseandPrintToSerial()
// runCAandPrintToSerial (reverse)
{

  //  ##############################SQUARE WAVE VOLTAMMETRY (Reverse -- Reduction)##############################

  // runSWV(LMPgain, -30, -500, 50, 66, 62.5, true);
  Serial.println(F("Voltage,Current"));
  for (uint16_t i = 0; i < arr_samples; i++)
  {
    Serial.print(volts[i]);
    Serial.print(F(","));
    Serial.println(amps[i]);
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
    Serial.print(volts[i]);
    Serial.print(F(","));
    Serial.println(amps[i]);
  }
  Serial.println("Quiet time till next Amp run");
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

void handleFileDelete(AsyncWebServerRequest *request)
{
  Serial.println("in file delete");
  if (request->params() == 0)
  {
    return request->send(500, "text/plain", "BAD ARGS");
  }
  AsyncWebParameter *p = request->getParam(0);
  String path = p->value();
  Serial.println("handleFileDelete: " + path);
  if (path == "/")
  {
    return request->send(500, "text/plain", "BAD PATH");
  }

  if (!SPIFFS.exists(path))
  {
    return request->send(404, "text/plain", "FileNotFound");
  }

  SPIFFS.remove(path);
  request->send(200, "text/plain", "");
  path = String();
}

void handle_websocket_text(uint8_t *payload)
{
  // do something...
  Serial.printf("handle_websocket_text called for: %s\n", payload);

  // test JSON parsing...
  //char m_JSONMessage[] = "{\"Key1\":123,\"Key2\",345}";
  //StaticJsonDocument<1000> m_JSONdoc;
  //deserializeJson(m_JSONdoc, m_JSONMessage); // m_JSONdoc is now a json object
  //int m_key1_value = m_JSONdoc["Key1"];
  // Serial.println(m_key1_value);

  // Parse JSON payload
  StaticJsonDocument<1000> m_JSONdoc_from_payload;
  DeserializationError m_error = deserializeJson(m_JSONdoc_from_payload, payload); // m_JSONdoc is now a json object
  if (m_error)
  {
    Serial.println("deserializeJson() failed with code ");
    Serial.println(m_error.c_str());
  }
  // Serial.println(m_key2_value);
  // now to iterate over (unknown) keys, we have to cast the StaticJsonDocument object into a JsonObject:
  // see https://techtutorialsx.com/2019/07/09/esp32-arduinojson-printing-the-keys-of-the-jsondocument/
  JsonObject m_JsonObject_from_payload = m_JSONdoc_from_payload.as<JsonObject>();
  // Iterate and print to serial:
  //   uint8_t LMPgain_control_panel = 6; // Feedback resistor of TIA.
  // int num_adc_readings_to_average_control_panel = 1;
  // int sweep_param_delayTime_ms_control_panel = 50;
  // int cell_voltage_control_panel = 100;
  for (JsonPair keyValue : m_JsonObject_from_payload)
  {
    String m_key_string = keyValue.key().c_str();

    if (m_key_string == "change_cell_voltage_to")
    {
      Serial.println("change_cell_voltage_to called");
      int m_new_cell_voltage = m_JSONdoc_from_payload["change_cell_voltage_to"];
      Serial.println(m_new_cell_voltage);
      cell_voltage_control_panel = m_new_cell_voltage;
      if (Sweep_Mode == CTLPANEL)
      {
        // set cell voltage, send params back to browswer such as percentage and dac settings
        Serial.println("Calling setLMPBias and setVoltage to:");
        Serial.println(cell_voltage_control_panel);
        setLMPBias(cell_voltage_control_panel);
        setVoltage(cell_voltage_control_panel);
        // send percent setting, dacvout back to browswer xyzxyz
        temp_json_string = "{\"dacVout\":";
        temp_json_string += String(dacVout, DEC);
        temp_json_string += ",\"percentage\":";
        temp_json_string += String(TIA_BIAS[bias_setting]);
        temp_json_string += "}";
        m_websocketserver.broadcastTXT(temp_json_string.c_str(), temp_json_string.length());
      }
    }
    if (m_key_string == "change_num_readings_to_average_per_point_to")
    {
      Serial.println("change_num_readings_to_average_per_point_to called");
      int m_new_num_readings_to_average_per_point = m_JSONdoc_from_payload["change_num_readings_to_average_per_point_to"];
      Serial.println(m_new_num_readings_to_average_per_point);
      num_adc_readings_to_average_control_panel = m_new_num_readings_to_average_per_point;
    }
    if (m_key_string == "change_delay_between_points_ms_to")
    {
      Serial.println("change_delay_between_points_ms_to called");
      int m_new_delay_between_points_ms = m_JSONdoc_from_payload["change_delay_between_points_ms_to"];
      Serial.println(m_new_delay_between_points_ms);
      sweep_param_delayTime_ms_control_panel = m_new_delay_between_points_ms;
    }
    if (m_key_string == "change_lmpGain_to")
    {
      Serial.println("change_lmpGain_to called");
      int m_new_lmpGain = m_JSONdoc_from_payload["change_lmpGain_to"];
      LMPgainGLOBAL = m_new_lmpGain;
      if (Sweep_Mode == CTLPANEL)
      {
        Serial.println(LMPgainGLOBAL);
        pStat.setGain(LMPgainGLOBAL);
      }
    }
    // change_control_panel_is_active_to
    if (m_key_string == "change_control_panel_is_active_to")
    {
      Serial.println("change_control_panel_is_active_to called");
      bool m_new_control_panel_active_state = m_JSONdoc_from_payload["change_control_panel_is_active_to"];
      Serial.println(m_new_control_panel_active_state);
      if (m_new_control_panel_active_state)
      {
        Sweep_Mode = CTLPANEL;
        send_is_sweeping_status_over_websocket(true);
      }
      if (!m_new_control_panel_active_state)
      {
        Sweep_Mode = dormant;
        send_is_sweeping_status_over_websocket(false);
      }
      // set control panel to active or inactive, depending on message
    }
  }

  // if (true == false) // if key = "change_cell_voltage_to"
  // {
  //   m_websocket_send_rate = (float)atof((const char *)&payload[0]); // adjust data send rate used in loop
  // }
  //deserializeJson(m_JSONdoc, payload); // m_JSONdoc is now a json object that was payload delivered by websocket message
}

// Called when receiving any WebSocket message
void onWebSocketEvent(uint8_t num,
                      WStype_t type,
                      uint8_t *payload,
                      size_t length)
{
  // Serial.println("onWebSocketEvent called");
  // Figure out the type of WebSocket event
  switch (type)
  {

  // Client has disconnected
  case WStype_DISCONNECTED:
    Serial.printf("[%u] Disconnected!\n", num);
    break;

  // New client has connected
  case WStype_CONNECTED:
  {
    IPAddress ip = m_websocketserver.remoteIP(num);
    Serial.printf("[%u] Connection from ", num);
    Serial.println(ip.toString());
  }
  break;

  // Echo text message back to client
  case WStype_TEXT:
    // Serial.println(payload[0,length-1]); // this doesn't work....
    Serial.printf("[%u] Received text: %s\n", num, payload);
    // m_websocketserver.sendTXT(num, payload);
    // if (true == false) // later change to if message has certain format:
    // {
    //   m_websocket_send_rate = (float)atof((const char *)&payload[0]); // adjust data send rate used in loop
    // }
    handle_websocket_text(payload);

    break;

  // For everything else: do nothing
  case WStype_BIN:
  case WStype_ERROR:
  case WStype_FRAGMENT_TEXT_START:
  case WStype_FRAGMENT_BIN_START:
  case WStype_FRAGMENT:
  case WStype_FRAGMENT_FIN:
  default:
    break;
  }
}

// void sendMessageToWebsocket(int num, char *MessageToSendToWebsocket)
// {
//   //  m_websocketserver.sendTXT(num,  MessageToSendToWebsocket);
//   //m_websocketserver.sendTXT("test hello world");
//   m_websocketserver.sendTXT(num, "Connected");
//   //  m_websocketserver.sendTXT(0, String &payload);
// }

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
  server.addHandler(new AsyncCallbackJsonWebHandler("/button1pressed", [](AsyncWebServerRequest *request1, JsonVariant &json1)
                                                    {
                                                      const JsonObject &jsonObj1 = json1.as<JsonObject>();
                                                      if (jsonObj1["on"])
                                                      {
                                                        Serial.println("Button 1 pressed. Running CV sweep.");
                                                        // digitalWrite(LEDPIN, HIGH);
                                                        Sweep_Mode = CV;
                                                        send_is_sweeping_status_over_websocket(true);
                                                      }
                                                      request1->send(200, "OK");
                                                    }));
  // Button #2
  server.addHandler(new AsyncCallbackJsonWebHandler("/button2pressed", [](AsyncWebServerRequest *request2, JsonVariant &json2)
                                                    {
                                                      const JsonObject &jsonObj2 = json2.as<JsonObject>();
                                                      if (jsonObj2["on"])
                                                      {
                                                        Serial.println("Button 2 pressed. Running NPV sweep.");
                                                        // digitalWrite(LEDPIN, HIGH);
                                                        Sweep_Mode = NPV;
                                                        send_is_sweeping_status_over_websocket(true);
                                                      }
                                                      request2->send(200, "OK");
                                                    }));
  // Button #3
  server.addHandler(new AsyncCallbackJsonWebHandler("/button3pressed", [](AsyncWebServerRequest *request3, JsonVariant &json3)
                                                    {
                                                      const JsonObject &jsonObj3 = json3.as<JsonObject>();
                                                      if (jsonObj3["on"])
                                                      {
                                                        Serial.println("Button 3 pressed. Running SQV sweep.");
                                                        // digitalWrite(LEDPIN, HIGH);
                                                        Sweep_Mode = SQV;
                                                        send_is_sweeping_status_over_websocket(true);
                                                      }
                                                      request3->send(200, "OK");
                                                    }));
  // Button #4
  server.addHandler(new AsyncCallbackJsonWebHandler("/button4pressed", [](AsyncWebServerRequest *request4, JsonVariant &json4)
                                                    {
                                                      const JsonObject &jsonObj4 = json4.as<JsonObject>();
                                                      if (jsonObj4["on"])
                                                      {
                                                        Serial.println("Button 4 pressed. Running CA sweep.");
                                                        // digitalWrite(LEDPIN, HIGH);
                                                        Sweep_Mode = CA;
                                                        send_is_sweeping_status_over_websocket(true);
                                                      }
                                                      request4->send(200, "OK");
                                                    }));
  // Button #5
  server.addHandler(new AsyncCallbackJsonWebHandler("/button5pressed", [](AsyncWebServerRequest *request5, JsonVariant &json5)
                                                    {
                                                      const JsonObject &jsonObj5 = json5.as<JsonObject>();
                                                      if (jsonObj5["on"])
                                                      {
                                                        Serial.println("Button 5 pressed. Running DC sweep.");
                                                        // digitalWrite(LEDPIN, HIGH);
                                                        Sweep_Mode = DCBIAS;
                                                        send_is_sweeping_status_over_websocket(true);
                                                      }
                                                      request5->send(200, "OK");
                                                    }));
  // Button #6
  server.addHandler(new AsyncCallbackJsonWebHandler("/button6pressed", [](AsyncWebServerRequest *request6, JsonVariant &json6)
                                                    {
                                                      const JsonObject &jsonObj6 = json6.as<JsonObject>();
                                                      if (jsonObj6["on"])
                                                      {
                                                        Serial.println("Button 6 pressed. Running IV sweep.");
                                                        // digitalWrite(LEDPIN, HIGH);
                                                        Sweep_Mode = IV;
                                                        send_is_sweeping_status_over_websocket(true);
                                                      }
                                                      request6->send(200, "OK");
                                                    }));

  // Button #7
  server.addHandler(new AsyncCallbackJsonWebHandler("/button7pressed", [](AsyncWebServerRequest *request7, JsonVariant &json7)
                                                    {
                                                      const JsonObject &jsonObj7 = json7.as<JsonObject>();
                                                      if (jsonObj7["on"])
                                                      {
                                                        Serial.println("Button 7 pressed. Running CAL sweep.");
                                                        // digitalWrite(LEDPIN, HIGH);
                                                        Sweep_Mode = CAL;
                                                        send_is_sweeping_status_over_websocket(true);
                                                      }
                                                      request7->send(200, "OK");
                                                    }));
  // Button #8
  server.addHandler(new AsyncCallbackJsonWebHandler("/button8pressed", [](AsyncWebServerRequest *request8, JsonVariant &json8)
                                                    {
                                                      const JsonObject &jsonObj8 = json8.as<JsonObject>();
                                                      if (jsonObj8["on"])
                                                      {
                                                        Serial.println("Button 8 pressed. Running MISC_MODE sweep.");
                                                        // digitalWrite(LEDPIN, HIGH);
                                                        Sweep_Mode = MISC_MODE;
                                                        send_is_sweeping_status_over_websocket(true);
                                                      }
                                                      request8->send(200, "OK");
                                                    }));
  // Button #9
  server.addHandler(new AsyncCallbackJsonWebHandler("/button9pressed", [](AsyncWebServerRequest *request9, JsonVariant &json9)
                                                    {
                                                      const JsonObject &jsonObj9 = json9.as<JsonObject>();
                                                      if (jsonObj9["on"])
                                                      {
                                                        Serial.println("Button 9 pressed.");
                                                        // digitalWrite(LEDPIN, HIGH);
                                                        Sweep_Mode = dormant;
                                                        send_is_sweeping_status_over_websocket(false);
                                                      }
                                                      request9->send(200, "OK");
                                                    }));
  // Button #10
  server.addHandler(new AsyncCallbackJsonWebHandler("/button10pressed", [](AsyncWebServerRequest *request10, JsonVariant &json10)
                                                    {
                                                      const JsonObject &jsonObj10 = json10.as<JsonObject>();
                                                      if (jsonObj10["on"])
                                                      {
                                                        Serial.println("Button 10 pressed.");
                                                        // digitalWrite(LEDPIN, HIGH);
                                                        Sweep_Mode = dormant;
                                                        send_is_sweeping_status_over_websocket(false);
                                                      }
                                                      request10->send(200, "OK");
                                                    }));

  server.serveStatic("/", SPIFFS, "/").setDefaultFile("index.html");

  server.on("/downloadfile", HTTP_GET, [](AsyncWebServerRequest *request)
            { request->send(SPIFFS, "/data.txt", "text/plain", true); });

  server.on("/rebootnanostat", HTTP_GET, [](AsyncWebServerRequest *request)
            {
              // reboot the ESP32
              request->send(200, "text/HTML", "  <head> <meta http-equiv=\"refresh\" content=\"5; URL=index.html\" /> <meta name=\"viewport\" content=\"width=device-width, initial-scale=1.0\"> </head> <body> <h1> Rebooting! </h1>  </body>");
              delay(500);
              ESP.restart();
            });

  server.onNotFound([](AsyncWebServerRequest *request)
                    {
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
  server.on("/actionpage.html", HTTP_POST, [](AsyncWebServerRequest *request)
            {
              String message;
              Serial.println("actionpage.html, HTTP_POST actionpage received , processing....");

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
                  //Serial.print(F("\t"))

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
              // request->send(200, "text/HTML", "Sweep data saved. Click <a href=\"/index.html\">here</a> to return to main page.");
              request->send(200, "text/HTML", "  <head> <meta http-equiv=\"refresh\" content=\"2; URL=index.html\" /> <meta name=\"viewport\" content=\"width=device-width, initial-scale=1.0\"> </head> <body> <h1> Settings saved! </h1> <p> Returning to main page. </p> </body>");
              // request->send(200, "OK");

              //   <head>
              //   <meta http-equiv="refresh" content="5; URL=https://www.bitdegree.org/" />
              // </head>
              // <body>
              //   <p>If you are not redirected in five seconds, <a href="https://www.bitdegree.org/">click here</a>.</p>
              // </body>

              // request->send(200, "text/URL", "www.google.com");
              // request->send(200, "text/URL", "<meta http-equiv=\"Refresh\" content=\"0; URL=https://google.com/\">");
              // <meta http-equiv="Refresh" content="0; URL=https://example.com/">
            });

  // Wifitools stuff:
  // Save credentials:
  server.on("/saveSecret", HTTP_POST, [](AsyncWebServerRequest *request)
            { handleGetSavSecreteJsonNoReboot(request); });

  // Wifi scan:
  server.on("/wifiScan.json", HTTP_GET, [](AsyncWebServerRequest *request)
            { getWifiScanJson(request); });

  // List directory:
  server.on("/list", HTTP_ANY, [](AsyncWebServerRequest *request)
            { handleFileList(request); });

  // Delete file
  server.on(
      "/edit", HTTP_DELETE, [](AsyncWebServerRequest *request)
      { handleFileDelete(request); });

  // Peter Burke custom code:
  server.on(
      "/m_fupload", HTTP_POST, [](AsyncWebServerRequest *request) {},
      [](AsyncWebServerRequest *request, const String &filename, size_t index, uint8_t *data,
         size_t len, bool final)
      { handleUpload(request, filename, "files.html", index, data, len, final); });

  // From https://github.com/me-no-dev/ESPAsyncWebServer/issues/542#issuecomment-573445113
  // handling uploading firmware file
  server.on(
      "/m_firmware_update", HTTP_POST, [](AsyncWebServerRequest *request)
      {
        if (!Update.hasError())
        {
          AsyncWebServerResponse *response = request->beginResponse(200, "text/plain", "OK");
          response->addHeader("Connection", "close");
          request->send(response);
          ESP.restart();
        }
        else
        {
          AsyncWebServerResponse *response = request->beginResponse(500, "text/plain", "ERROR");
          response->addHeader("Connection", "close");
          request->send(response);
        }
      },
      handleFirmwareUpload);

  // handling uploading filesystem file
  // see https://github.com/espressif/arduino-esp32/blob/371f382db7dd36c470bb2669b222adf0a497600d/libraries/HTTPUpdateServer/src/HTTPUpdateServer.h
  server.on(
      "/m_filesystem_update", HTTP_POST, [](AsyncWebServerRequest *request)
      {
        if (!Update.hasError())
        {
          AsyncWebServerResponse *response = request->beginResponse(200, "text/plain", "OK");
          response->addHeader("Connection", "close");
          request->send(response);
          ESP.restart();
        }
        else
        {
          AsyncWebServerResponse *response = request->beginResponse(500, "text/plain", "ERROR");
          response->addHeader("Connection", "close");
          request->send(response);
        }
      },
      handleFilesystemUpload);

  // Done with configuration, begin server:
  server.begin();
}

void setup()
{
  // blinky pin for diagnostic:
  pinMode(LEDPIN, OUTPUT);

  // Start serial interface:
  Wire.begin(); // uses default microcontroller pins
  Serial.begin(115200);
  while (!Serial)
    ;

  Serial.println("Welcome to NanoStat, Firmware Rev. 0.1.2!");

  // initialize ADC:
  analogReadResolution(12);

  //############################### ENABLE THE POTENTIOSTAT #######################################
  pStat.setMENB(MENB);
  delay(50);
  pStat.standby();
  delay(50);
  initLMP(0); // Initializes the LMP91000 to the appropriate settings

  //############################### SPIFFS STARTUP #######################################
  if (!SPIFFS.begin(true))
  {
    Serial.println("An Error has occurred while mounting SPIFFS");
    return;
  }

  //############################# STATIC WIFI #####################################

  // WiFi.mode(WIFI_STA);
  // WiFi.begin(SSID, PASSWORD);
  // while (WiFi.waitForConnectResult() != WL_CONNECTED)
  // {
  //   Serial.println("Connected Failed! Rebooting...");
  //   delay(1000);
  //   ESP.restart();
  // }
  // Serial.println("Connected!");
  // Serial.println("The local IP address is:");
  // Serial.println(WiFi.localIP()); //print the local IP address

  //#############################  WIFITOOL CUSTOMIZED #####################################
  // Used this repo as a basis for ideas. https://github.com/oferzv/wifiTool

  bool m_autoconnected_attempt_succeeded = false;
  m_autoconnected_attempt_succeeded = connectAttempt("", ""); // uses SSID/PWD stored in ESP32 secret memory.....
  // Serial.print("m_autoconnected_attempt_succeeded = ");
  // Serial.println(m_autoconnected_attempt_succeeded);
  if (!m_autoconnected_attempt_succeeded)
  {
    // try SSID/PWD from file...
    Serial.println("Failed to connect.");
    String m_filenametopass = "/credentials.JSON";
    m_autoconnected_attempt_succeeded = readSSIDPWDfile(m_filenametopass);
  }
  if (!m_autoconnected_attempt_succeeded)
  {
    // start AP server
    // Serial.println("connect failed, starting AP server");
    setUpAPService();
    runWifiPortal();

    // MDNS.begin("nanostat"); // see https://randomnerdtutorials.com/esp32-access-point-ap-web-server/
  }

  // connectAttempt(SSID,PASSWORD); // uses SSID/PWD stored in ESP32 secret memory.....

  //#############################  WIFITOOL #####################################

  // wifiTool.begin(false);
  // if (!wifiTool.wifiAutoConnect())
  // {
  //   Serial.println("fail to connect to wifi!!!!");
  //   wifiTool.runApPortal();
  // }

  // Serial.println("wifitools called ");
  // delete &wifiTool;
  // delay(2000);

  //############################# DNS #####################################
  MDNS.begin("nanostat");

  //############################# WEBSERVER & WIFI #####################################

  server.reset(); // try putting this in setup
  configureserver();
  //  runWifiPortal_after_connected_to_WIFI(); // Allows some system level tools such as saving wifi credentials and scan, OTA firmware upgrade, file directory..
  // configureserver has the code in it little by little, can't do configuration after starting server which happens inside configureserver() method as of now...

  //############################# WEBSOCKET #####################################

  m_websocketserver.begin();
  m_websocketserver.onEvent(onWebSocketEvent); // Start WebSocket server and assign callback

  //############################# READ CALIBRATION FILE IF THERE IS ONE #####################################

  // SPIFFS.remove("/calibration.JSON"); // manual delete to test code.
  readCalFile();

  //############################# BLINK LED TO SHOW SETUP COMPLETE #####################################
  Serial.print("Heap free memory (in bytes)= ");
  Serial.println(ESP.getFreeHeap());
  Serial.println(F("Setup complete."));
  blinkLED(LEDPIN, 15, 1000); // blink LED to show setup is complete, also give settle time to LMP91000
}

void loop()
{

  //will hold the code here until a character is sent over the Serial port
  //this ensures the experiment will only run when initiated
  //  Serial.println(F("Press enter to begin a sweep."));
  //  while (!Serial.available())
  //    ;
  //  Serial.read();
  // Look for and handle WebSocket data

  m_websocketserver.loop();

  if (m_send_websocket_test_data_in_loop == true) // do things here in loop at full speed
  {
    // Pseudocode: xxx_period_in_ms_xxx=period_in_s * 1e3 = (1/freqHz)*1e3
    if (millis() - last_time_sent_websocket_server > (1000 / m_websocket_send_rate)) // every half second, print
    {
      //    sendTimeOverWebsocketJSON();
      sendValueOverWebsocketJSON(100 * 0.5 * sin(millis() / 1e3)); // value is sine wave of time , frequency 0.5 Hz, amplitude 100.
      last_time_sent_websocket_server = millis();
    }
    // m_microsbefore_websocketsendcalled=micros();
    // sendTimeOverWebsocketJSON(); // takes 2.5 ms on average, when client is connected, else 45 microseconds...
    // Serial.println(micros()-m_microsbefore_websocketsendcalled);
  }

  if (Sweep_Mode == NPV)
  {
    // void runNPV(uint8_t lmpGain, int16_t startV, int16_t endV,
    //         int8_t pulseAmp, uint32_t pulse_width, uint32_t pulse_period,
    //         uint32_t quietTime, uint8_t range, bool setToZero)
    LMPgainGLOBAL = sweep_param_lmpGain;
    runNPV(sweep_param_lmpGain, sweep_param_startV_NPV, sweep_param_endV_NPV,
           sweep_param_pulseAmp_NPV, sweep_param_width_NPV, sweep_param_period_NPV,
           sweep_param_quietTime_NPV, 1, sweep_param_setToZero);
    writeVoltsCurrentArraytoFile();
    // sendVoltammogramWebsocketJSON();
    sendVoltammogramWebsocketBIN();
    Sweep_Mode = dormant;
    send_is_sweeping_status_over_websocket(false);
  }
  else if (Sweep_Mode == CV)
  {
    LMPgainGLOBAL = sweep_param_lmpGain;
    runCV(sweep_param_lmpGain, sweep_param_cycles_CV, sweep_param_startV_CV,
          sweep_param_endV_CV, sweep_param_vertex1_CV, sweep_param_vertex2_CV, sweep_param_stepV_CV,
          sweep_param_rate_CV, sweep_param_setToZero);
    writeVoltsCurrentArraytoFile();
    // sendVoltammogramWebsocketJSON();
    sendVoltammogramWebsocketBIN();
    Sweep_Mode = dormant;
    send_is_sweeping_status_over_websocket(false);
  }
  else if (Sweep_Mode == SQV)
  {
    LMPgainGLOBAL = sweep_param_lmpGain;

    runSWV(sweep_param_lmpGain, sweep_param_startV_SWV, sweep_param_endV_SWV,
           sweep_param_pulseAmp_SWV, sweep_param_stepV_SWV, sweep_param_freq_SWV, sweep_param_setToZero);
    writeVoltsCurrentArraytoFile();
    // sendVoltammogramWebsocketJSON();
    sendVoltammogramWebsocketBIN();
    Sweep_Mode = dormant;
    send_is_sweeping_status_over_websocket(false);
  }
  else if (Sweep_Mode == CA)
  {
    LMPgainGLOBAL = sweep_param_lmpGain;
    runAmp(sweep_param_lmpGain, sweep_param_pre_stepV_CA, sweep_param_quietTime_CA,
           sweep_param_V1_CA, sweep_param_t1_CA, sweep_param_V2_CA, sweep_param_t2_CA,
           sweep_param_samples_CA, 1, sweep_param_setToZero);
    writeVoltsCurrentArraytoFile();
    // sendVoltammogramWebsocketJSON();
    sendVoltammogramWebsocketBIN();
    Sweep_Mode = dormant;
    send_is_sweeping_status_over_websocket(false);
  }
  else if (Sweep_Mode == DCBIAS)
  {
    LMPgainGLOBAL = sweep_param_lmpGain;

    testNoiseAtABiasPoint(sweep_param_biasV_noisetest, sweep_param_numPoints_noisetest,
                          sweep_param_delayTime_ms_noisetest);
    writeVoltsCurrentArraytoFile();
    // sendVoltammogramWebsocketJSON();
    sendVoltammogramWebsocketBIN();
    Sweep_Mode = dormant;
    send_is_sweeping_status_over_websocket(false);
  }
  else if (Sweep_Mode == IV)
  {
    LMPgainGLOBAL = sweep_param_lmpGain;

    testIV(sweep_param_startV_IV, sweep_param_endV_IV, sweep_param_numPoints_IV,
           sweep_param_delayTime_ms_IV);
    writeVoltsCurrentArraytoFile();
    //sendVoltammogramWebsocketJSON();
    sendVoltammogramWebsocketBIN();
    Sweep_Mode = dormant;
    send_is_sweeping_status_over_websocket(false);
  }
  else if (Sweep_Mode == CAL)
  {
    LMPgainGLOBAL = sweep_param_lmpGain;

    calibrateDACandADCs(sweep_param_delayTime_ms_CAL);
    Sweep_Mode = dormant;
    send_is_sweeping_status_over_websocket(false);
    writeCalFile();
  }
  else if (Sweep_Mode == CTLPANEL)
  {

    pStat.setGain(LMPgainGLOBAL);

    delay(sweep_param_delayTime_ms_control_panel);
    // read adc and convert to current:
    analog_read_avg_bits_temp = (float)analog_read_avg(num_adc_readings_to_average_control_panel, LMP);
    v1_temp = (3.3 / 255.0) * (1 / (2.0 * b_coeff)) * analog_read_avg_bits_temp - (a_coeff / (2.0 * b_coeff)) * (3.3 / 255.0); // LMP is wired to Vout of the LMP91000
    v1_temp = v1_temp * 1000.0;
    v2_temp = dacVout * .5;                                                                //the zero of the internal transimpedance amplifier
    amps_temp = (((v1_temp - v2_temp) / 1000) / TIA_GAIN[LMPgainGLOBAL - 1]) * pow(10, 6); //scales to uA
    // create json string to send to broswer:
    temp_json_string = "{\"amps\":";
    temp_json_string += String(amps_temp, DEC);
    temp_json_string += ",\"volts\":";
    temp_json_string += String(cell_voltage_control_panel);
    temp_json_string += ",\"time\":";
    temp_json_string += String(millis());
    temp_json_string += ",\"analog_read_avg_bits\":";
    temp_json_string += String(analog_read_avg_bits_temp);
    temp_json_string += ",\"analog_read_avg_mV\":";
    temp_json_string += String(v1_temp, DEC);
    temp_json_string += "}";
    // send json string to browswer
    m_websocketserver.broadcastTXT(temp_json_string.c_str(), temp_json_string.length());
  }
  else if (Sweep_Mode == MISC_MODE) // list directory to serial
  {
    listDir("/", 3);
    //    delay(250);
    // sendVoltammogramWebsocketJSON();
    // readFileAndPrintToSerial();
    Sweep_Mode = dormant;
    send_is_sweeping_status_over_websocket(false);

    sendVoltammogramWebsocketBIN();
  }
  else
  {
    delay(10);
  }
  last_time_loop_called = millis();
}
