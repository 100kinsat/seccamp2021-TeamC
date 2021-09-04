#pragma once

#include "state.hpp"
#include "sd_log.hpp"
#include "cansat_io.hpp"
#include "FS.h"

// WebServer の有効化
#define ENABLE_WEB_SERVER true

#if ENABLE_WEB_SERVER == true
#include "web_controller.hpp"
#endif

#define SD_BUFFER_COUNT 128
#define QUEUE_BUFFER_SIZE 256

// WebServer 用 WiFi AP の接続情報
#define WIFI_SSID "Espressif-32CS"
#define WIFI_PASS "oOxzAnmt"

class LogTask {
  public:
    LogTask(SdLog * sdLog, CanSatIO * canSatIO, State * state);
    void sendToLoggerTask(String &buffer, bool skippable);
    void sendToLoggerTask(const char * buffer, bool skippable);
    bool setupTask();
    void restartOnError();
    void restartOnError(int blinkCount);
    int openNextLogFile(fs::FS &fs);
    void closeFile();
    bool checkSdHealth();

    QueueHandle_t queue;
    SdLog * sdLog;
    State * state;
    char sdBuffer[SD_BUFFER_COUNT + 8][QUEUE_BUFFER_SIZE];
    int sdBufferPosition = 0;
    unsigned long sdLastWrite = 0;
    unsigned long sdWriteInterval = 2000;
    bool requestClose = false;
    bool blockSdWrite = false;
    bool requestFlush = false;
    int writeFailedCount = 0;

  private:
    static void loggerTask(void *pvParameters);
    void flushFile();

    CanSatIO * canSatIO;
};

LogTask::LogTask(SdLog * sdLog, CanSatIO * canSatIO, State * state) {
  this->sdLog = sdLog;
  this->canSatIO = canSatIO;
  this->state = state;
}

void LogTask::sendToLoggerTask(String &buffer, bool skippable) {
  sendToLoggerTask(buffer.c_str(), skippable);
}

void LogTask::sendToLoggerTask(const char * buffer, bool skippable) {
  if (buffer[0] == '\x00') {
    return;
  }
  BaseType_t status;

  Serial.print(buffer);

  if (!skippable || uxQueueMessagesWaiting(queue) == 0) {
    status = xQueueSend(queue, buffer, 0);

    if (status != pdPASS) {
      Serial.println("rtos queue send failed");
    }
  }
}

bool LogTask::setupTask() {
  queue = xQueueCreate(64, QUEUE_BUFFER_SIZE);

  if(queue != NULL) {
    xTaskCreatePinnedToCore(this->loggerTask, "loggerTask", 4096, this, 1, NULL, 1);
  } else {
    Serial.println("rtos queue create error, stopped");
    return false;
  }
  return true;
}

void LogTask::loggerTask(void *pvParameters) {
  LogTask *thisPointer = (LogTask *) pvParameters;
  BaseType_t status;
  char buffer[QUEUE_BUFFER_SIZE];
  const TickType_t tick = 10U; // [ms]

  #if ENABLE_WEB_SERVER == true
  WebController webController(WIFI_SSID, WIFI_PASS, thisPointer->state);
  #endif

  while (true) {
    status = xQueueReceive(thisPointer->queue, buffer, tick);
    if(status == pdPASS) {
      if (thisPointer->state->enableSdLog && thisPointer->sdBufferPosition < SD_BUFFER_COUNT) {
        strcpy(thisPointer->sdBuffer[thisPointer->sdBufferPosition], buffer);
        thisPointer->sdBufferPosition++;
      }

      #if ENABLE_WEB_SERVER == true
      if (buffer[0] == 'M') {
        webController.setValue(0, buffer);
      }
      if (buffer[0] == 'G') {
        webController.setValue(1, buffer);
      }
      if (buffer[0] == 'S') {
        webController.setValue(2, buffer);
      }
      if (buffer[0] == 'L') {
        webController.setValue(3, buffer);
      }
      #endif
    } else {
      if (uxQueueMessagesWaiting(thisPointer->queue) != 0) {
        Serial.println("rtos queue receive error?");
      }
    }

    if (0 < thisPointer->sdBufferPosition && !thisPointer->blockSdWrite) {
      unsigned long now = millis();
      if (
        thisPointer->requestClose ||
        thisPointer->requestFlush ||
        SD_BUFFER_COUNT < thisPointer->sdBufferPosition ||
        thisPointer->sdWriteInterval < now - thisPointer->sdLastWrite
      ) {
        for (int i = 0; i < thisPointer->sdBufferPosition; i++) {
          bool writeStatus = thisPointer->sdLog->writeLog(thisPointer->sdBuffer[i]);
          if (writeStatus) {
            thisPointer->writeFailedCount = 0;
          } else {
            Serial.println("SD append failed.");
            thisPointer->writeFailedCount++;
          }
        }
        thisPointer->sdBufferPosition = 0;
        thisPointer->sdLastWrite = now;
        if (thisPointer->requestClose) {
          thisPointer->blockSdWrite = true;
          thisPointer->requestClose = false;
        }
        thisPointer->requestFlush = false;
      }
    }

    if (10 < thisPointer->writeFailedCount) {
      Serial.println("sd write failed, stopped");
      thisPointer->restartOnError(6);
    }

    #if ENABLE_WEB_SERVER == true
    webController.handleClient();
    #endif

    if (thisPointer->state->rebootByUserRequest) {
      thisPointer->restartOnError(2);
    }
  }
}

void LogTask::restartOnError() {
  restartOnError(10);
}

void LogTask::restartOnError(int blinkCount) {
  unsigned long start = millis();
  unsigned long lastChanged = 0;
  bool isLEDOn = false;
  int i = 0;
  while (i < blinkCount * 2) {
    if (200 < millis() - lastChanged) {
      lastChanged = millis();
      isLEDOn = !isLEDOn;
      i++;
    }
    if (isLEDOn) {
      canSatIO->setLEDOn();
    } else {
      canSatIO->setLEDOff();
    }
  }
  Serial.println("Restarting...");
  delay(100);
  ESP.restart();
}

int LogTask::openNextLogFile(fs::FS &fs) {
  int number = sdLog->openNextLogFile(fs);
  blockSdWrite = false;
  state->logFileNumber = number;
  return number;
}

void LogTask::closeFile() {
  requestClose = true;
  while (requestClose) {
    delay(1);
  }
  sdLog->closeFile();
}

void LogTask::flushFile() {
  requestFlush = true;
  while (requestFlush) {
    delay(1);
  }
}

bool LogTask::checkSdHealth() {
  sendToLoggerTask("State,Sd test ok\n", false);
  flushFile();
  return writeFailedCount == 0;
}
