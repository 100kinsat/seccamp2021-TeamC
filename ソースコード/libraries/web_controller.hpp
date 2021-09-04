#pragma once

#include "./state.hpp"
#include <WiFi.h>
#include <WebServer.h>
#include <ESPmDNS.h>

#define LOG_KIND 4

class WebController {
  public:
    WebController(const char * ssid, const char * password, State * state);

    void handleClient();
    void setValue(int index, char * value);

  private:
    WebServer server{80};
    const char * host = "esp32";
    String logs[LOG_KIND];
    State * state;
};

WebController::WebController(const char * ssid, const char * password, State * state) {
  this->state = state;
  WiFi.softAP(ssid, password);
  MDNS.begin(host);

  server.on("/", [&]() {
    server.send(200, "text/html", "\
<html>\
  <head>\
    <title>CanSat Controller</title>\
    <style>\
      body { background-color: #cccccc; font-family: Arial, Helvetica, Sans-Serif; Color: #000088; }\
    </style>\
    <script>\
      setTimeout(function update() {\
        fetch('/log')\
          .then(res => res.text())\
          .then(text => document.getElementById('log').innerHTML = text)\
          .then(() => setTimeout(update, 250))\
      }, 250);\
    </script>\
  </head>\
  <body>\
    <h1>CanSat Controller</h1>\
    <pre id='log'></pre>\
    <p>\
      -- CONTROL --\
    </p>\
    <p>\
      <form action='/goal' method='get'>\
        <label for='lat'>Goal lat,long:</label>\
        <input type='text' name='lat'>\
        <input type='text' name='long'>\
        <input type='submit' value='Update'>\
      </form>\
    </p>\
    <p>\
      <form action='/targetYaw' method='get'>\
        <label for='yaw'>Fixed targetYaw, speed:</label>\
        <input type='text' name='yaw'>\
        <input type='text' name='speed'>\
        <input type='submit' value='Update'>\
      </form>\
    </p>\
    <p>\
      <form action='/northYaw' method='get'>\
        <label for='yaw'>Set northYaw:</label>\
        <input type='text' name='yaw'>\
        <input type='submit' value='Update'>\
      </form>\
    </p>\
    <p>\
      <a href='/magCalibrate'>Do magCalibrate</a>\
    </p>\
    <p>\
      <a href='/stop'>Stop motor</a>\
    </p>\
    <p>\
      SD Log:\
      <a href='/sd/enable'>Enable</a>\
      <a href='/sd/disable'>Disable</a>\
    </p>\
    <p>\
      -- SYSTEM --\
    </p>\
    <p>\
      <form action='/log_show' method='get'>\
        <label for='number'>Show log:</label>\
        <input type='text' name='number'>\
        <input type='submit' value='Show'>\
      </form>\
    </p>\
    <p>\
      <form action='/log_dl' method='get'>\
        <label for='number'>Download log:</label>\
        <input type='text' name='number'>\
        <input type='submit' value='Show'>\
      </form>\
    </p>\
    <p>\
      <a href='/reboot'>Reboot (EN button)</a>\
    </p>\
  </body>\
</html>");
  });

  server.on("/log", [&]() {
    String buffer = "Uptime: ";
    int sec = millis() / 1000;
    int min = sec / 60;
    int hr = min / 60;
    sec %= 60;
    min %= 60;
    if (hr < 10) {
      buffer += "0";
    }
    buffer += String(hr);
    buffer += ":";
    if (min < 10) {
      buffer += "0";
    }
    buffer += String(min);
    buffer += ":";
    if (sec < 10) {
      buffer += "0";
    }
    buffer += String(sec);
    buffer += "\n";

    for (int i = 0; i < LOG_KIND; i++) {
      buffer += logs[i];
    }
    server.send(200, "text/plain", buffer);
  });

  server.on("/reboot", [&]() {
    xTaskCreatePinnedToCore(this->state->userRequestReboot, "reboot", 1024, this->state, 1, NULL, 0);
    server.sendHeader("Location", "/");
    server.send(302);
  });

  server.on("/goal", [&]() {
    String latString = server.arg("lat");
    String longString = server.arg("long");
    double lat = 0.0;
    if (latString[0] != '\x00') {
      lat = std::stod(latString.c_str());
    }
    double lng = 0.0;
    if (longString[0] != '\x00') {
      lng = std::stod(longString.c_str());
    }
    State newState;
    newState.goalLat = lat;
    newState.goalLong = lng;
    State * args[2];
    args[0] = this->state;
    args[1] = &newState;
    TaskHandle_t handle;
    xTaskCreatePinnedToCore(this->state->copyGoal, "copyGoal", 1024, args, 1, &handle, 0);
    server.sendHeader("Location", "/");
    server.send(302);
    delay(100);
    vTaskDelete(handle);
  });

  server.on("/targetYaw", [&]() {
    String yawString = server.arg("yaw");
    String speedString = server.arg("speed");
    double yaw = 0.0;
    if (yawString[0] != '\x00') {
      yaw = std::stod(yawString.c_str());
    }
    int speed = 255;
    if (speedString[0] != '\x00') {
      speed = std::stoi(speedString.c_str());
    }
    State newState;
    newState.targetYaw = yaw;
    newState.currentSpeed = speed;
    State * args[2];
    args[0] = this->state;
    args[1] = &newState;
    TaskHandle_t handle;
    xTaskCreatePinnedToCore(this->state->copyTargetYaw, "copyTargetYaw", 1024, args, 1, &handle, 0);
    server.sendHeader("Location", "/");
    server.send(302);
    delay(100);
    vTaskDelete(handle);
  });

  server.on("/northYaw", [&]() {
    String yawString = server.arg("yaw");
    double yaw = 0.0;
    if (yawString[0] != '\x00') {
      yaw = std::stod(yawString.c_str());
    }
    State newState;
    newState.northYaw = yaw;
    State * args[2];
    args[0] = this->state;
    args[1] = &newState;
    TaskHandle_t handle;
    xTaskCreatePinnedToCore(this->state->copyNorthYaw, "copyNorthYaw", 1024, args, 1, &handle, 0);
    server.sendHeader("Location", "/");
    server.send(302);
    delay(100);
    vTaskDelete(handle);
  });

  server.on("/magCalibrate", [&]() {
    State * args[1];
    args[0] = this->state;
    TaskHandle_t handle;
    xTaskCreatePinnedToCore(this->state->magCalibrate, "magCalibrate", 1024, args, 1, &handle, 0);
    server.sendHeader("Location", "/");
    server.send(302);
    delay(100);
    vTaskDelete(handle);
  });

  server.on("/stop", [&]() {
    State * args[1];
    args[0] = this->state;
    TaskHandle_t handle;
    xTaskCreatePinnedToCore(this->state->stop, "stop", 1024, args, 1, &handle, 0);
    server.sendHeader("Location", "/");
    server.send(302);
    delay(100);
    vTaskDelete(handle);
  });

  server.on("/sd/enable", [&]() {
    State newState;
    newState.enableSdLog = true;
    State * args[2];
    args[0] = this->state;
    args[1] = &newState;
    TaskHandle_t handle;
    xTaskCreatePinnedToCore(this->state->copySdLogState, "copySdLogState", 1024, args, 1, &handle, 0);
    server.sendHeader("Location", "/");
    server.send(302);
    delay(100);
    vTaskDelete(handle);
  });

  server.on("/sd/disable", [&]() {
    State newState;
    newState.enableSdLog = false;
    State * args[2];
    args[0] = this->state;
    args[1] = &newState;
    TaskHandle_t handle;
    xTaskCreatePinnedToCore(this->state->copySdLogState, "copySdLogState", 1024, args, 1, &handle, 0);
    server.sendHeader("Location", "/");
    server.send(302);
    delay(100);
    vTaskDelete(handle);
  });

  server.on("/log_dl", [&]() {
    String path = "/log/" + server.arg("number") + ".csv";
    File dataFile = SD.open(path.c_str());

    if (!dataFile) {
      server.send(404);
      return false;
    }

    String dataType = "application/octet-stream";

    if (server.streamFile(dataFile, dataType) != dataFile.size()) {
      Serial.println("Sent less data than expected!");
    }

    dataFile.close();
    return true;
  });

  server.on("/log_show", [&]() {
    String path = "/log/" + server.arg("number") + ".csv";
    File dataFile = SD.open(path.c_str());

    if (!dataFile) {
      server.send(404);
      return false;
    }

    String dataType = "text/plain";

    if (server.streamFile(dataFile, dataType) != dataFile.size()) {
      Serial.println("Sent less data than expected!");
    }

    dataFile.close();
    return true;
  });

  server.onNotFound([&]() {
    String message = "File Not Found\n\n";
    message += "URI: ";
    message += server.uri();
    message += "\nMethod: ";
    message += (server.method() == HTTP_GET) ? "GET" : "POST";
    message += "\nArguments: ";
    message += server.args();
    message += "\n";

    for (uint8_t i = 0; i < server.args(); i++) {
      message += " " + server.argName(i) + ": " + server.arg(i) + "\n";
    }

    server.send(404, "text/plain", message);
  });
  server.begin();
}

void WebController::handleClient() {
  server.handleClient();
}

void WebController::setValue(int index, char * value) {
  logs[index] = String(value);
}
