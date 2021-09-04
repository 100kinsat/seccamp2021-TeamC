// Reference:
// https://github.com/espressif/arduino-esp32/tree/master/libraries/SD

# pragma once

/*
 * Connect the SD card to the following pins:
 *
 * SD Card | ESP32
 *    D2       -
 *    D3       SS
 *    CMD      MOSI
 *    VSS      GND
 *    VDD      3.3V
 *    CLK      SCK
 *    VSS      GND
 *    D0       MISO
 *    D1       -
 */

#include "Arduino.h"
#include "FS.h"
#include "SD.h"
#include "SPI.h"


#define LOG_DIR "/log"
#define PREVIOUS_NUMBER_FILE "/prev_log_number.txt"

class SdLog {
  public:
    SdLog();

    void closeFile();
    bool writeLog(const char * message);
    bool createDirIfNotExist(fs::FS &fs, const char * dirname);

    int openNextLogFile(fs::FS &fs);

  private:
    String fileName;
    File file;
    bool isAvailable = false;
    uint8_t cardType;

    bool openFile(fs::FS &fs, const char *path);
    bool existDir(fs::FS &fs, const char * dirname);
    bool createDir(fs::FS &fs, const char * path);
    bool existFile(fs::FS &fs, const char * filename);
    bool renameFile(fs::FS &fs, const char * path1, const char * path2);
};

/**
 * @brief SDカードの初期化処理
 *
 */
SdLog::SdLog() {
  if (!SD.begin()) {
    return;
  }
  cardType = SD.cardType();

  if (cardType == CARD_NONE) {
    return;
  }

  isAvailable = true;
}

bool SdLog::openFile(fs::FS &fs, const char *path) {
  file = fs.open(path, FILE_WRITE);
  return (bool)file;
}

void SdLog::closeFile() {
  file.close();
}

/**
 * @brief ファイルに書き込む
 *
 * @param message
 */
bool SdLog::writeLog(const char * message) {
  if (!file) {
    return false;
  }
  if (message[0] == '\x00') {
    return true;
  }
  bool result = file.print(message) != 0;
  file.flush();
  if (!result) {
    file.close();
  }
  return result;
}

int SdLog::openNextLogFile(fs::FS &fs) {
  File numberFile = fs.open(PREVIOUS_NUMBER_FILE);
  int number = 0;
  if (numberFile) {
    number = numberFile.parseInt() + 1;
    numberFile.close();
  }

  String numberString = String(number);

  numberFile = fs.open(PREVIOUS_NUMBER_FILE, FILE_WRITE);
  if (numberFile) {
    numberFile.print(numberString);
    numberFile.close();
  }

  fileName = LOG_DIR;
  fileName += "/";
  fileName += numberString;
  fileName += ".csv";

  createDirIfNotExist(fs, LOG_DIR);

  if (existFile(fs, fileName.c_str())) {
    String newFileName = String(fileName);
    newFileName += ".backup.";
    newFileName += String(esp_random());
    renameFile(fs, fileName.c_str(), newFileName.c_str());
  }

  openFile(fs, fileName.c_str());

  return number;
}


bool SdLog::createDirIfNotExist(fs::FS &fs, const char * dirname) {
  return existDir(fs, dirname) || createDir(fs, dirname);
}

bool SdLog::existDir(fs::FS &fs, const char * dirname){
  File root = fs.open(dirname);
  return root && root.isDirectory();
}

bool SdLog::createDir(fs::FS &fs, const char * path){
  return fs.mkdir(path);
}

bool SdLog::existFile(fs::FS &fs, const char * filename){
  File root = fs.open(filename);
  return root;
}

bool SdLog::renameFile(fs::FS &fs, const char * path1, const char * path2){
  return fs.rename(path1, path2);
}
