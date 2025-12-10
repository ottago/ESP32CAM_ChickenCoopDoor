#include <ArduinoOTA.h>

void configureOtaCallbacks() {
  // Port defaults to 8266
  // ArduinoOTA.setPort(8266);

  ArduinoOTA.onStart([]() {
    // Make sure we unmount the filesystem before update.
    LittleFS.end();

    Serial.println(F("OTA Start"));
    Serial.printf(F("%lu free heap.\r\n"), ESP.getFreeHeap());
  });

  ArduinoOTA.onEnd([]() {
    Serial.println(F("OTA End"));
    Serial.printf(F("%lu free heap.\r\n"), ESP.getFreeHeap());

    //delay(3000);
    //ESP.restart();
  });

  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    Serial.printf(F("OTA Progress: %u%% %d free heap\r"),
                  (progress / (total / 100)), ESP.getFreeHeap());
  });

  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf(F("%lu free heap.\n"), ESP.getFreeHeap());
    switch (error) {
      case OTA_AUTH_ERROR:
        Serial.println(F("OTA: Auth Failed"));
        break;

      case OTA_BEGIN_ERROR:
        Serial.println(F("OTA: Begin Failed"));
        break;

      case OTA_CONNECT_ERROR:
        Serial.println(F("OTA: Connect Failed"));
        break;

      case OTA_RECEIVE_ERROR:
        Serial.println(F("OTA: Receive Failed"));
        break;

      case OTA_END_ERROR:
        Serial.println(F("OTA: End Failed"));
        break;

      default:
        Serial.printf(F("Unknown OTA Error (%u)\r\n"), error);
        break;
    }
  });
}
