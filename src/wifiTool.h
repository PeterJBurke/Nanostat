/**************************************************************
   wifiTool is a library for the ESP 8266&32/Arduino platform
   SPIFFS oriented AsyncWebServer based wifi configuration tool.
   https://github.com/oferzv/wifiTool
   
   Built by Ofer Zvik (https://github.com/oferzv)
   And Tal Ofer (https://github.com/talofer99)
   Licensed under MIT license
 **************************************************************/

#ifndef WIFITOOL_h
#define WIFITOOL_h

#include "Arduino.h"
#include <DNSServer.h>

#include <ArduinoOTA.h>
#ifdef ESP32
#include <FS.h>
#include <SPIFFS.h>
#include <ESPmDNS.h>
#include <WiFi.h>
#include <AsyncTCP.h>
#elif defined(ESP8266)
#include <ESP8266WiFi.h>
#include <ESPAsyncTCP.h>
#include <ESP8266mDNS.h>
#include <DoubleResetDetector.h>
#endif
#include <ESPAsyncWebServer.h>
#include <SPIFFSEditor.h>


#define WAIT_FOR_WIFI_TIME_OUT 6000UL
#define SECRETS_PATH "/secrets.json"

class WifiTool
{
  public:
    WifiTool();
    uint8_t wifiAutoConnect();
    void runApPortal();
    void runWifiPortal();
    void process();
    void begin();
    void begin(uint8_t autoConnectFlag);
  private:
    std::unique_ptr<DNSServer> dnsServer;
#if defined(ESP32)
    std::unique_ptr<AsyncWebServer> server;
#else
    std::unique_ptr<AsyncWebServer> server;
    std::unique_ptr<DoubleResetDetector> drd;
#endif
    File fsUploadFile;

    // DNS server
    const byte DNS_PORT = 53;
    void updateUpload();
    void setUpAPService();
    boolean connectAttempt(String ssid, String password);
    String getJSONValueByKey(String textToSearch, String key);
    void handleFileList(AsyncWebServerRequest *request);
    void handleFileDelete(AsyncWebServerRequest * request);
    void getWifiScanJson(AsyncWebServerRequest * request);
    void handleGetSavSecreteJson(AsyncWebServerRequest *request);
    int getRSSIasQuality(int RSSI);
    boolean runAP;
    unsigned long restartSystem;
    void handleUpload(AsyncWebServerRequest *request, String filename, String redirect, size_t index, uint8_t *data, size_t len, bool final);
};

#endif
