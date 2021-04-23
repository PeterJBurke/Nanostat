/**************************************************************
   wifiTool is a library for the ESP 8266&32/Arduino platform
   SPIFFS oriented AsyncWebServer based wifi configuration tool.
   https://github.com/oferzv/wifiTool
   
   Built by Ofer Zvik (https://github.com/oferzv)
   And Tal Ofer (https://github.com/talofer99)
   Licensed under MIT license
 **************************************************************/

#include "Arduino.h"
#include "wifiTool.h"

/*
    class CaptiveRequestHandler
*/

class CaptiveRequestHandler : public AsyncWebHandler {
  public:
    CaptiveRequestHandler() {}
    virtual ~CaptiveRequestHandler() {}

    bool canHandle(AsyncWebServerRequest *request) {
      //request->addInterestingHeader("ANY");
      return true;
    }

    void handleRequest(AsyncWebServerRequest *request) {
      request->redirect("/wifi_index.html");
    }
};

/*
    WifiTool()
*/
WifiTool::WifiTool()
{
  runAP = false;
#if defined(ESP8266)
  drd.reset(new DoubleResetDetector(2, 0));
#endif
}


/*
    begin()
*/
void WifiTool::begin()
{
  begin(true);
}

void WifiTool::begin(uint8_t autoConnectFlag) {
#if defined(ESP32)
  boolean isDblReser = false;
#else
  boolean isDblReser = drd->detectDoubleReset();
#endif
  if (isDblReser) {
    Serial.println("Double Reset Detected");
    WiFi.begin("", ""); // clear any last saved parameters
    delay(50);
    // drd stop
#if defined(ESP8266)
    drd->stop();
#endif
    runAP = true;
    setUpAPService();
    // run the portal
    runWifiPortal();
  } else {
    delay(1000); // giving us up to 1 second to hit dbl reset
    // drd stop to prevent dbl click
#if defined(ESP8266)
    drd->stop();
#endif
    if (autoConnectFlag)
    {
      if (!wifiAutoConnect())
      {
        setUpAPService();
        runWifiPortal();
      } //end if
    } //end if
  } //end if
}

/*
    process()
*/

void WifiTool::process() {
  ///DNS
  if (runAP)
    dnsServer->processNextRequest();
  //yield
  yield();
  // Reset flag/timer
  if (restartSystem) {
    if (restartSystem + 1000 < millis()) {
      ESP.restart();
    } //end if
  } //end if
}

/*
    connectAttempt(String ssid, String password)
*/
boolean WifiTool::connectAttempt(String ssid, String password) {
  boolean isWiFiConnected = false;
  // set mode
  WiFi.mode(WIFI_STA);
  // if no SSID is passed we attempt last connected wifi (from WIFI object)
  if (ssid == "") {
	if (WiFi.status() != WL_CONNECTED) {
		WiFi.begin();
    }
  } else {
    int  ssidSize = ssid.length() + 1;
    int  passwordSize = password.length() + 1;
    char ssidArray[ssidSize];
    char passwordArray[passwordSize];
    ssid.toCharArray(ssidArray, ssidSize);
    password.toCharArray(passwordArray, passwordSize);
    WiFi.begin(ssidArray, passwordArray);
  } //end if

  Serial.print(F("\nConnecting Wifi..."));
  unsigned long now = millis();
  while (WiFi.status() != WL_CONNECTED && millis() < now + WAIT_FOR_WIFI_TIME_OUT  ) {
    Serial.print(".");
    delay(250);
  }
  Serial.print(F("\nStatus:"));
  Serial.println(WiFi.status());
  if (WiFi.status() == WL_CONNECTED) {
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

/*
    getJSONValueByKey(String textToSearch, String key)
*/
String WifiTool::getJSONValueByKey(String textToSearch, String key)
{
  if (textToSearch.length() == 0) {
    return String("");
  }

  String searchPhrase = String("\"");
  searchPhrase.concat(key);
  searchPhrase.concat("\":\"");
  int fromPosition = textToSearch.indexOf(searchPhrase, 0);
  if (fromPosition == -1) {
    // return because there is no status or it's null
    return String("");
  }

  fromPosition = fromPosition + searchPhrase.length();
  int toPosition = textToSearch.indexOf("\"", fromPosition);

  if (toPosition == -1) {
    // return because there is no end quote
    return String("");
  }
  textToSearch.remove(toPosition);

  return textToSearch.substring(fromPosition);
} //end get json by value


/*
    wifiAutoConnect()
*/
uint8_t WifiTool::wifiAutoConnect()
{
  uint8_t isAutoConnected = 1;
  String secretsJson;

  // attemp auto connect
  if (!connectAttempt("", "")) {
    Serial.println(F("failed to autoconnect try saved networks"));

    // start spiff // Don't need, it's already started by calling program... PJB 4/22/2021
    // if (!SPIFFS.begin())
    // {
    //   // Serious problem
    //   Serial.println(F("SPIFFS Mount failed"));
    //   isAutoConnected = 2;
    //   return isAutoConnected;
    // } //end if

    //Serial.println(F("SPIFFS Mount succesfull"));

    // read secrets file
    if (SPIFFS.exists(SECRETS_PATH)) {
      File f = SPIFFS.open(SECRETS_PATH, "r");
      if (!f) {
        Serial.println(F("file open failed"));
        isAutoConnected = 3;
        return isAutoConnected;
      } //end if

      // read json from file
      while (f.available()) {
        secretsJson += char(f.read());
      } //end while

      for (byte i = 1; i < 4; i++) {
        String _ssid = getJSONValueByKey(secretsJson, "ssid" + String(i));
        if (_ssid != "" && _ssid != WiFi.SSID()) {
          String _pass = getJSONValueByKey(secretsJson, "pass" + String(i));
          delay(50);
          if (connectAttempt(_ssid, _pass)) {
            break;
          }
        } //end if
      } //end for

      // if not connected
      runAP = !WiFi.isConnected();
    } else {
      Serial.println(F("secrets file is missing"));
      runAP = true;
    }//end if

    if (runAP) {
      isAutoConnected = 0;
    }//end if
    
  } //end if
  
  return isAutoConnected;
} //end void



void WifiTool::runApPortal()
{
  runAP = true;
  setUpAPService();
  runWifiPortal();
}

/*
   getRSSIasQuality(int RSSI)
*/
int WifiTool::getRSSIasQuality(int RSSI) {
  int quality = 0;

  if (RSSI <= -100) {
    quality = 0;
  } else if (RSSI >= -50) {
    quality = 100;
  } else {
    quality = 2 * (RSSI + 100);
  }
  return quality;
}



/*
   getWifiScanJson()
*/
void WifiTool::getWifiScanJson(AsyncWebServerRequest * request)
{
  String json = "{\"scan_result\":[";
  int n = WiFi.scanComplete();
  if (n == -2) {
    WiFi.scanNetworks(true);
  } else if (n) {
    for (int i = 0; i < n; ++i) {
      if (i) json += ",";
      json += "{";
      json += "\"RSSI\":" + String(WiFi.RSSI(i));
      json += ",\"SSID\":\"" + WiFi.SSID(i) + "\"";
      json += "}";
    }
    WiFi.scanDelete();
    if (WiFi.scanComplete() == -2) {
      WiFi.scanNetworks(true);
    }
  }
  json += "]}";
  request->send(200, "application/json", json);
  json = String();
}

/*
   handleGetSavSecreteJson()
*/
void WifiTool::handleGetSavSecreteJson(AsyncWebServerRequest *request) {
  AsyncWebParameter* p;
  String jsonString = "{";
  jsonString.concat("\"ssid1\":\"");
  p = request->getParam("ssid1", true);
  jsonString.concat(p->value());
  jsonString.concat("\",");

  jsonString.concat("\"pass1\":\"");
  p = request->getParam("pass1", true);
  jsonString.concat(p->value().c_str());
  jsonString.concat("\",");

  jsonString.concat("\"ssid2\":\"");
  p = request->getParam("ssid2", true);
  jsonString.concat(p->value().c_str());
  jsonString.concat("\",");

  jsonString.concat("\"pass2\":\"");
  p = request->getParam("pass2", true);
  jsonString.concat(p->value().c_str());
  jsonString.concat("\",");

  jsonString.concat("\"ssid3\":\"");
  p = request->getParam("ssid3", true);
  jsonString.concat(p->value().c_str());
  jsonString.concat("\",");

  jsonString.concat("\"pass3\":\"");
  p = request->getParam("pass3", true);
  jsonString.concat(p->value().c_str());

  jsonString.concat("\"}");

  File file = SPIFFS.open(SECRETS_PATH, "w");
  file.print(jsonString);
  file.close();

  request->send(200, "text/html", "<h1>Restarting .....</h1>");
  restartSystem = millis();
}

/*
   setUpAPService()
*/

void WifiTool::setUpAPService()
{
  Serial.println(F("RUN AP"));
  dnsServer.reset(new DNSServer());
  WiFi.mode(WIFI_AP);
  WiFi.softAPConfig(IPAddress(172, 217, 28, 1), IPAddress(172, 217, 28, 1), IPAddress(255, 255, 255, 0));
  WiFi.softAP("config");
  delay(500);

  /* Setup the DNS server redirecting all the domains to the apIP */
  dnsServer->setErrorReplyCode(DNSReplyCode::NoError);
  dnsServer->start(DNS_PORT, "*", IPAddress(172, 217, 28, 1));

  //Serial.println("dns server config done");

}

void WifiTool::runWifiPortal() {

#if defined(ESP32)
  server.reset(new AsyncWebServer(80));
#else
  server.reset(new AsyncWebServer(80));
#endif
  IPAddress myIP;
  if (runAP)
    myIP = WiFi.softAPIP();
  else
    myIP = WiFi.localIP();

  Serial.print(F("AP IP address: "));
  Serial.println(myIP);

  // start spiff
  if (!SPIFFS.begin())
  {
    // Serious problem
    Serial.println(F("SPIFFS Mount failed"));
    return;
  } //end if


  server->serveStatic("/", SPIFFS, "/").setDefaultFile("wifi_index.html");

  server->on("/saveSecret/", HTTP_ANY, [&, this](AsyncWebServerRequest * request) {
    handleGetSavSecreteJson(request);
  });

  server->on("/list", HTTP_ANY, [&, this](AsyncWebServerRequest * request) {
    handleFileList(request);
  });

  // spiff delete
  server->on("/edit", HTTP_DELETE, [&, this](AsyncWebServerRequest * request) {
    handleFileDelete(request);
  });

  // spiff upload
  server->on("/edit", HTTP_POST, [&, this](AsyncWebServerRequest * request) {},
  [&, this](AsyncWebServerRequest * request, const String & filename, size_t index, uint8_t *data,
            size_t len, bool final) {
    handleUpload(request, filename, "/wifi_spiffs_admin.html", index, data, len, final);
  });

  server->on("/wifiScan.json", HTTP_GET, [&, this](AsyncWebServerRequest * request) {
    getWifiScanJson(request);

  });

  // Simple Firmware Update Form
  server->on("/update", HTTP_GET, [&, this](AsyncWebServerRequest * request) {
    request->send(SPIFFS, "/wifi_upload.html");
  });
  server->on("/update", HTTP_POST, [&, this](AsyncWebServerRequest * request) {
    uint8_t isSuccess = !Update.hasError();
    if (isSuccess)
      restartSystem = millis();
    AsyncWebServerResponse *response = request->beginResponse(200, "text/plain", isSuccess ? "OK" : "FAIL");
    response->addHeader("Connection", "close");
    request->send(response);


    }, [&, this](AsyncWebServerRequest * request, String filename, size_t index, uint8_t *data, size_t len, bool final) {
    if (!index) {
      Serial.printf("Update Start: %s\n", filename.c_str());

#if defined(ESP8266)
      Update.runAsync(true);
      if (!Update.begin((ESP.getFreeSketchSpace() - 0x1000) & 0xFFFFF000)) {
        Update.printError(Serial);
      }
#else
      if (!Update.begin(UPDATE_SIZE_UNKNOWN)) {
        Update.printError(Serial);
      }
#endif
    }
    if (!Update.hasError()) {
      if (Update.write(data, len) != len) {
        Update.printError(Serial);
      }
    }
    if (final) {
      if (Update.end(true)) {
        Serial.printf("Update Success: %uB\n", index + len);
      } else {
        Update.printError(Serial);
      }
    }
  });

  server->on("/restart", HTTP_GET, [&, this](AsyncWebServerRequest * request) {
    request->send(200, "text/html", "OK");
    restartSystem = millis();
  });

  server->onNotFound([](AsyncWebServerRequest * request) {
    Serial.println("handle not found");
    request->send(404);
  });

  server->addHandler(new CaptiveRequestHandler()).setFilter(ON_AP_FILTER);//only when requested from AP

  Serial.println(F("HTTP server started"));
  server->begin();

  while (1) {
    process();
  }
}

void WifiTool::handleFileList(AsyncWebServerRequest *request) {
  Serial.println("handle fle list");
  if (!request -> hasParam("dir")) {
    request->send(500, "text/plain", "BAD ARGS");
    return;
  }


  AsyncWebParameter* p = request->getParam("dir");
  String path = p->value().c_str();
  Serial.println("handleFileList: " + path);
  String output = "[";
#if defined(ESP8266)

  Dir dir = SPIFFS.openDir(path);
  while (dir.next()) {
    File entry = dir.openFile("r");
    if (output != "[") {
      output += ',';
    }
    bool isDir = false;
    output += "{\"type\":\"";
    output += (isDir) ? "dir" : "file";
    output += "\",\"name\":\"";
    output += String(entry.name()).substring(1);
    output += "\"}";
    entry.close();
  }

#else

  File root = SPIFFS.open("/", "r");
  if (root.isDirectory()) {
    Serial.println("here ??");
    File file = root.openNextFile();
    while (file) {
      if (output != "[") {
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
#endif

  path = String();
  output += "]";
  Serial.println(output);
  request->send(200, "application/json", output);
}


void WifiTool::handleFileDelete(AsyncWebServerRequest * request) {
  Serial.println("in file delete");
  if (request->params() == 0) {
    return request->send(500, "text/plain", "BAD ARGS");
  }
  AsyncWebParameter* p = request->getParam(0);
  String path = p->value();
  Serial.println("handleFileDelete: " + path);
  if (path == "/") {
    return request->send(500, "text/plain", "BAD PATH");
  }

  if (!SPIFFS.exists(path)) {
    return request->send(404, "text/plain", "FileNotFound");
  }

  SPIFFS.remove(path);
  request->send(200, "text/plain", "");
  path = String();
}



//==============================================================
//   handleUpload
//==============================================================
void WifiTool::handleUpload(AsyncWebServerRequest *request, String filename, String redirect, size_t index, uint8_t *data, size_t len, bool final) {
  if (!index) {
    if (!filename.startsWith("/")) filename = "/" + filename;
    Serial.println((String)"UploadStart: " + filename);
    fsUploadFile = SPIFFS.open(filename, "w");            // Open the file for writing in SPIFFS (create if it doesn't exist)
  }
  for (size_t i = 0; i < len; i++) {
    fsUploadFile.write(data[i]);
    //Serial.write(data[i]);
  }
  if (final) {
    Serial.println((String)"UploadEnd: " + filename);
    fsUploadFile.close();
    request->redirect(redirect);
  }
}


