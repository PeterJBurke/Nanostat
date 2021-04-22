  // Button #4
  server.addHandler(new AsyncCallbackJsonWebHandler("/button4pressed", [](AsyncWebServerRequest *request4, JsonVariant &json4) {
    const JsonObject &jsonObj4 = json4.as<JsonObject>();
    if (jsonObj4["on"])
    {
      Serial.println("Button 4 pressed.");
      // digitalWrite(LEDPIN, HIGH);
      Sweep_Mode = CV;
    }
    request4->send(200, "OK");
  }));