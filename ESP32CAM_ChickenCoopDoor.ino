/*********************************************************************
   Author: Nicholas Klopfer-Webber
   Date:   2022-12-03
   Board:  ESP32-CAM

   Board to control Chicken coop door remotely.
 *********************************************************************


    Arduino IDE Settings
   --------------------
   Board: Ai Thinker ESP32-CAM
   FQBN: esp32:esp32:esp32cam
   Board Settings:
      CPU Frequency: 240Mhz (WiFi/BT)
      Flash Frequency: 80MHz
      Flash Mode: QIO

   To enable this board in Arduino IDE enter the following url into prefferences -> Additional Boards URL
   https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json
   Open Boards Manager from Tools > Board menu and find ESP32 platform.
   Select the version you need from a drop-down box.


   Hardware
   --------
   ESP32-CAM
   TB6612FNG - Dual Half Bridge
   Mini Switch Mode PSU 12v -> 3.3v


   Flash procedure:
   Connect a TTL serial device ensuring to power the device with 3.3v!!!
   Short GPIO0 / D3 to Gnd.
   In the Arduino IDE:
      Select when it starts spinning, connect RST to Gnd momendarily
   and wait for it to start uploading.  If it fails try again, the timing can be tricky.
   Once flashed, remove the FLASH bridge and reset the device.



  Upload the contents of the data folder with MkSPIFFS Tool ("ESP8266 Sketch Data Upload" in Tools menu in Arduino IDE)
  or you can upload the contents of a folder if you cd in that folder and run the following command:
  for file in `\ls -A1`; do curl -F "file=@$PWD/$file" esp8266fs.local/edit; done
*/

#define ENABLE_CAMERA 1

#if ENABLE_CAMERA
#define CAMERA_MODEL_AI_THINKER  // Has PSRAM
#include "esp_camera.h"
#include "camera_pins.h"
#include "camera_helper.h"
#endif

#include <ESP_NtpTime.h>
#include <ESP_Wifi_Helper.h>
#include <WiFiManager.h>  // https://github.com/tzapu/WiFiManager
#include <WiFiMultiCredentials.h>
#include <Telemetry.h>


#include <ESP_FileBrowser.h>
//#include <SD_MMC_Utils.h>
#include <LittleFS_Utils.h>
#include "OTA.h"
#include "DoorControl.h"



unsigned long readHallTimeMs = 0;



//#define ESP32_CAM_LED 33
//#define ESP32_CAM_FLASH 4

void ntpSyncCallback(uint64_t lastTimeMs, ESP_NtpTime::EventType type);
ESP_NtpTime ntpTime("au.pool.ntp.org", ntpSyncCallback);
#define TELEMETRY_INTERVAL_MS 5 * 60 * 1000
Telemetry telemetry(&ntpTime, "telemetry.ottago.com", 4949, 200, TELEMETRY_INTERVAL_MS);

fs::FS *filesystem;
void fileChangedCallback(String path);

DoorControl doorControl(&ntpTime);

WebServer server(80);

// --- ----- ---------- ---------- ----------
#if ESP32

#include <WiFi.h>
#include <ESPmDNS.h>

#endif
// --- ----- ---------- ---------- ----------




#ifdef ESP32_CAM_FLASH
void IRAM_ATTR flashTimerCb() {
  static bool state = false;
  state = !state;
  ledcWrite(0, state ? 180 : 250);  // Output is inverted.
}
#endif

#if ENABLE_CAMERA
void startCameraServer();

void cameraSetup() {
  //Serial.setDebugOutput(true);

  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.frame_size = FRAMESIZE_UXGA;
  config.pixel_format = PIXFORMAT_JPEG;  // for streaming
  config.grab_mode = CAMERA_GRAB_WHEN_EMPTY;
  config.fb_location = CAMERA_FB_IN_PSRAM;
  config.jpeg_quality = 12;
  config.fb_count = 1;

  // if PSRAM IC present, init with UXGA resolution and higher JPEG quality
  //                      for larger pre-allocated frame buffer.
  if (config.pixel_format == PIXFORMAT_JPEG) {
    if (psramFound()) {
      config.jpeg_quality = 10;
      config.fb_count = 2;
      config.grab_mode = CAMERA_GRAB_LATEST;
    } else {
      // Limit the frame size when PSRAM is not available
      config.frame_size = FRAMESIZE_SVGA;
      config.fb_location = CAMERA_FB_IN_DRAM;
    }
  } else {
    // Best option for face detection/recognition
    config.frame_size = FRAMESIZE_240X240;
#if CONFIG_IDF_TARGET_ESP32S3
    config.fb_count = 2;
#endif
  }

  // camera init
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf(PSTR("Camera init failed with error 0x%x\r\n"), err);
    return;
  }

  sensor_t *s = esp_camera_sensor_get();
  // initial sensors are flipped vertically and colors are a bit saturated
  if (s->id.PID == OV3660_PID) {
    s->set_vflip(s, 1);        // flip it back
    s->set_brightness(s, 1);   // up the brightness just a bit
    s->set_saturation(s, -2);  // lower the saturation
  }
  // drop down frame size for higher initial frame rate
  if (config.pixel_format == PIXFORMAT_JPEG) {
    s->set_framesize(s, FRAMESIZE_QVGA);
  }

  // XXX WiFi.setSleep(false);

  startCameraServer();
}
#endif

void setup() {
  const char *hostname = "ChickCam";
 
  Serial.begin(115200);
  delay(100);
  Serial.println(F("Booting"));
  Serial.flush();

  filesystem = LittleFS_setup();
  int16_t savedCredsCount = WifiInit(hostname, "Ch1ck3n5Need@Updat3", filesystem, &ntpTime);
  /*
  , []() {
    // Indicate that the captive Wifi configuration portal is starting.
    // TODO - Maybe flash the lights...
  });
*/
  configureOtaCallbacks();

  Serial.println(F("Starting Telemetry..."));
  Telemetry::instance()->setHostname(hostname);
  Telemetry::instance()
    ->appendMacAddress()
    ->appendHostname()
    ->append("SSID", WiFi.SSID())
    ->append("SavedAP#", savedCredsCount)
    ->appendResetReason()
    ->send();

  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());

  Serial.println(F("Starting FileBrowser..."));
  FileBrowserSetup(&server, *filesystem, &fileChangedCallback);
  Serial.println(F("FileBrowser Started."));

  Serial.printf(PSTR("--- LittleFS.list ---\r\n"));
  list("/");
  Serial.printf(PSTR("--- LittleFS.printDirectory ---\r\n"));
  printDirectory(filesystem);
  Serial.printf(PSTR("--- ---- ---- ---\r\n"));


#ifdef ESP32_CAM_FLASH
  // Ensure the FLASH LED is off, otherwise is glows fainntly.
  // Note: We need to do this after initalising the SD card in 1-bit mode since it still configured all HS2 pins.
  ledcAttachPin(ESP32_CAM_FLASH, 0 /* PWM Channel */);
  // Initialize channels
  ledcSetup(0 /* PWM Channel 0-15 */, 5000 /* PWM Freq */, 8 /* Resolution 1-16 bits */);
  ledcWrite(0, 255 - 20);  // Output is inverted -> On very dimly.

  // Setup a timer to flash the light when we're waiting for the WiFiManager to do its thing...
  hw_timer_t *timer = timerBegin(1 /* Timer to use 0-3 */, ESP.getCpuFreqMHz() / 1000000 /* Prescaler */, true /* Count up=true, down=false */);
  timerAttachInterrupt(timer, &flashTimerCb, true /* ??? */);
  timerAlarmWrite(timer, 500000 /* Counter match */, true /* Auto reload */);
  timerAlarmEnable(timer);  // Start timer.
#endif

#ifdef ESP32_CAM_LED
  pinMode(ESP32_CAM_LED, OUTPUT);
  digitalWrite(ESP32_CAM_LED, HIGH);
#endif

  // Get this going soon, since it allows manual control even before Wifi is up.
  doorControl.begin();


  if (!WiFi.isConnected()) {
    Serial.printf(PSTR("Failed to connect to WiFi!  Rebooting...\r\n"));
#ifdef ESP32_CAM_LED
    digitalWrite(ESP32_CAM_LED, LOW);
#endif
    delay(30 * 1000);
    ESP.restart();
  }


#ifdef ESP32_CAM_FLASH
  timerAlarmDisable(timer);
#endif

  
  Serial.println(F("Starting FileBrowser..."));
  FileBrowserSetup(&server, *filesystem, &fileChangedCallback);

// Ask server to track these headers, required for '/save'.
  const char* headerkeys[] = { "Content-Length" };
  server.collectHeaders(headerkeys, sizeof(headerkeys) / sizeof(headerkeys[0]));
  // FIXME XXX TODO server.on(F("/save"), HTTP_POST, handleSaveSettings);
  
  server.on(F("/hostname"), HTTP_GET, []() {
    server.send(200, "text/plain", Telemetry::instance()->getHostname());
  });

  server.on(F("/time"), HTTP_GET, []() {
    char buf[40];
    strftime(buf, sizeof(buf), "%F %T", ntpTime.getTimeParts());
    server.send(200, "text/plain", buf);
  });

  server.on(F("/uptime"), HTTP_GET, []() {
    char buf[20];
    snprintf(buf, sizeof(buf), "%ld", millis());
    server.send(200, "text/plain", buf);
  });

  server.on(F("/resync"), HTTP_GET, []() {
    ntpTime.sync();
    server.send(200, "text/plain", "Success");
    // ntpTime.getNtpResultStr() + "\n" +
    // ntpTime.getTimezoneResultStr());
  });

  server.on(F("/restart"), HTTP_GET, []() {
    server.send(200, "text/plain", "Success");
    delay(1000);
    ESP.restart();
  });


  server.on(F("/doorOpen"), HTTP_GET, []() {
    doorControl.open();
    server.send(200, "text/plain", "Success");
  });
  server.on(F("/doorClose"), HTTP_GET, []() {
    doorControl.close();
    server.send(200, "text/plain", "Success");
  });
  server.on(F("/doorStop"), HTTP_GET, []() {
    doorControl.stop();
    server.send(200, "text/plain", "Success");
  });
  server.on(F("/doorStatus"), HTTP_GET, []() {
    server.send(200, "application/json;charset=utf-8",
                doorControl.statusJson());
  });

  server.begin();

#if ENABLE_CAMERA
  cameraSetup();
#endif

  Serial.println(F("Setup complete."));

#ifdef ESP32_CAM_FLASH
  // Turn the Flash off completly.
  ledcWrite(0, 0);
#endif
}



void ntpSyncCallback(uint64_t lastTimeMs, ESP_NtpTime::EventType type) {
  if (type == ESP_NtpTime::EventType::AFTER_SYNC && ntpTime.hasSync()) {
    doorControl.calculateNextCycleTime();
  }
}

void fileChangedCallback(String path) {
  Serial.printf(PSTR("fileChangedCallback - %s\r\n"), path.c_str());
  doorControl.calculateNextCycleTime();
}


void loop() {
  ArduinoOTA.handle();
  server.handleClient();
  ntpTime.handle();
  Telemetry::handle();
  doorControl.handle();

#if defined(ESP32_CAM_FLASH) || defined(ESP32_CAM_LED)
  unsigned long millisNow = millis();
#endif

#ifdef ESP32_CAM_FLASH
  if (millisNow > 60000) {
    uint16_t value = (millisNow / 1000) % 60;
    ledcWrite(0, 255 * 60 / value);
  }
#endif

#ifdef ESP32_CAM_LED
  static unsigned long ledFlashTime = 0;
  if (ledFlashTime < millisNow) {
    ledFlashTime = millisNow + 500;

    digitalWrite(ESP32_CAM_LED, !digitalRead(ESP32_CAM_LED));
  }
#endif


  /*
  if (millisNow > readHallTimeMs) {
    readHallTimeMs = millisNow + 1000;

    Serial.printf(PSTR("Hall: %d\r\n"), hallRead());
  }
  */
}
