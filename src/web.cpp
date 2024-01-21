#include <Arduino.h>
#include <DNSServer.h>
#include <WiFiClient.h>
#if defined(ESP8266)
#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
ESP8266WebServer Webserver(80);
#include <ESP8266mDNS.h>
#else
#include <WiFi.h>
#include <WebServer.h>
WebServer Webserver(80);
#include <ESPmDNS.h>
#endif
#include <WebSocketsServer.h>
#include <Hash.h>
#include <WiFiManager.h>         //https://github.com/tzapu/WiFiManager
#include "index_html.h"
#include "sketch_js.h"


WebSocketsServer webSocket = WebSocketsServer(81);

char DEVICE_NAME[32] = "bno055-a";

/* Set the delay between fresh samples */
#define BNO055_SAMPLERATE_DELAY_MS (10)

bool Connected = false;

const char IMU_JSON[] PROGMEM = R"=====({"heading":%f,"pitch":%f,"roll":%f})=====";

void handleRoot() {
  char html[1024];

  snprintf_P(html, sizeof(html), INDEX_HTML,
      DEVICE_NAME, 1000/BNO055_SAMPLERATE_DELAY_MS);
  Webserver.send(200, "text/html", html);
}

void handleSketch() {
  Webserver.send_P(200, "text/javascript", SKETCH_JS);
}

void handleNotFound(){
  String message = "File Not Found\n\n";
  message += "URI: ";
  message += Webserver.uri();
  message += "\nMethod: ";
  message += (Webserver.method() == HTTP_GET)?"GET":"POST";
  message += "\nArguments: ";
  message += Webserver.args();
  message += "\n";
  for (uint8_t i=0; i<Webserver.args(); i++){
    message += " " + Webserver.argName(i) + ": " + Webserver.arg(i) + "\n";
  }
  Webserver.send(404, "text/plain", message);
}

void webSocketEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t length)
{
  static uint32_t lastMillis = 0;

  //Serial.printf("webSocketEvent(%d, %d, ...)\r\n", num, type);
  switch(type) {
    case WStype_DISCONNECTED:
      Serial.printf("[%u] Disconnected!\r\n", num);
      break;
    case WStype_CONNECTED:
      {
        IPAddress ip = webSocket.remoteIP(num);
        Serial.printf("[%u] Connected from %d.%d.%d.%d url: %s\r\n", num, ip[0], ip[1], ip[2], ip[3], payload);
        // Send the current orientation
      }
      break;
    case WStype_TEXT:
      //Serial.printf("[%u] [%u ms] get Text: %s\r", num, millis()-lastMillis, payload);
      lastMillis = millis();
      break;
    case WStype_BIN:
      Serial.printf("[%u] get binary length: %u\r\n", num, length);
      hexdump(payload, length);

      // echo data back to browser
      webSocket.sendBIN(num, payload, length);
      break;
    default:
      Serial.printf("Invalid WStype [%d]\r\n", type);
      break;
  }
}

/**************************************************************************/
/*
    Arduino setup function (automatically called at startup)
*/
/**************************************************************************/
void webserver_setup()
{
  WiFiManager wifiManager;
  //reset saved settings
  //wifiManager.resetSettings();

  //fetches ssid and pass from eeprom and tries to connect
  //if it does not connect it starts an access point with the specified name
  //and goes into a blocking loop awaiting configuration
  wifiManager.autoConnect(DEVICE_NAME);

  Webserver.on("/", handleRoot);
  Webserver.on("/sketch.js", handleSketch);

  Webserver.onNotFound(handleNotFound);

  Webserver.begin();
  Serial.println("HTTP server started");

  webSocket.begin();
  webSocket.onEvent(webSocketEvent);
  Serial.println("WebSocket server started");
}

void webserver_loop()
{
  if (WiFi.status() == WL_CONNECTED) {
    if (!Connected) {
      Serial.print(F("WiFi connected! IP address: "));
      Serial.println(WiFi.localIP());
      Connected = true;
    }
  }
  else {
    if (Connected) {
      Serial.println(F("WiFi not connected!"));
      Connected = false;
    }

    //WiFiManager
    //Local intialization. Once its business is done, there is no need to keep it around
    WiFiManager wifiManager;

    //fetches ssid and pass from eeprom and tries to connect
    //if it does not connect it starts an access point with the specified name
    //and goes into a blocking loop awaiting configuration
    wifiManager.autoConnect(DEVICE_NAME);

    Serial.print(F("WiFi connected! IP address: "));
    Serial.println(WiFi.localIP());
    Connected = true;
  }

  webSocket.loop();
  Webserver.handleClient();
}

void webserver_update(float roll, float pitch, float yaw)
{
  // Send JSON over websocket
  char payload[80];
  snprintf_P(payload, sizeof(payload), IMU_JSON, yaw, pitch, roll);
  webSocket.broadcastTXT(payload, strlen(payload));
}