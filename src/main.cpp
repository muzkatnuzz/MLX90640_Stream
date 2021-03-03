
#include "Arduino.h"
#include "WiFi.h"
#include "ESPAsyncWebServer.h"

#include "mlx90640-cam-rtos.h"

AsyncWebServer webServer(80);

void handleNotFound(AsyncWebServerRequest *request);

/*
  Next one is an include with wifi credentials.
  This is what you need to do:

  1. Create a file called "home_wifi_multi.h" in the same folder   OR   under a separate subfolder of the "libraries" folder of Arduino IDE. (You are creating a "fake" library really - I called it "MySettings").
  2. Place the following text in the file:
  #define SSID1 "replace with your wifi ssid"
  #define PWD1 "replace your wifi password"
  3. Save.

  Should work then
*/
#include "home_wifi_multi.h"

// ==== Handle invalid URL requests ============================================
void handleNotFound(AsyncWebServerRequest *request)
{
  String message = "Server is running!\n\n";
  message += "URI: ";
  message += request->url();
  message += "\nMethod: ";
  message += (request->method() == HTTP_GET) ? "GET" : "POST";
  message += "\nArguments: ";
  message += request->args();
  message += "\n";
  request->send(200, "text / plain", message);
}

// ==== SETUP method ==================================================================
void setup()
{
  // Setup Serial connection:
  Serial.begin(115200);
  delay(1000); // wait for a second to let Serial connect
  Serial.printf("setup: free heap  : %d\n", ESP.getFreeHeap());

  //  Configure and connect to WiFi
  IPAddress ip;

  WiFi.mode(WIFI_STA);
  WiFi.begin(SSID1, PWD1);
  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(F("."));
  }
  ip = WiFi.localIP();
  Serial.println(F("WiFi connected"));
  Serial.println("");
  Serial.print("Stream Link: http://");
  Serial.print(ip);
  Serial.println("/stream");

  // configure web server
  webServer.onNotFound(handleNotFound);

  configureCamera(&webServer, 25, 33);
  startCamera();
  
  // TODO move before or after camera start?!
  webServer.begin();

  Serial.printf("setup complete: free heap  : %d\n", ESP.getFreeHeap());
}

void loop() {
  // this seems to be necessary to let IDLE task run and do GC
  vTaskDelay(1000);
}