#pragma once

#include "./state.hpp"
#include <TinyGPS++.h>

#define GPS_RX_PIN 2
#define GPS_DEFAULT_BAUD 9600
#define GPS_DEBUG false

class GPS {
  public:
    GPS(State * state);

    bool changeUpdateInterval();
    void encode();
    void getHeader(String& buffer);
    void readValues(String& buffer);

    double hdop = 0.0;
    bool locationIsValid = false;
    double distanceToGoal = 0.0;
    double courseToGoal = 0.0;
    double speedKmph = 0.0;
    double course = 0.0;

    double lat = 0.0;
    double lng = 0.0;

    TinyGPSPlus gps;

  private:
    HardwareSerial *ss;
    State * state;

    void debugWrite(char message);
    void debugPrintln(const char * message);
};

GPS::GPS(State * state) {
  ss = new HardwareSerial(GPS_RX_PIN);
  ss->begin(GPS_DEFAULT_BAUD);
  this->state = state;
}

bool GPS::changeUpdateInterval() {
  for (unsigned long start = millis(); millis() - start < 500;) {
    while (ss->available()) {
      ss->read();
    }
    delay(1);
  }
  bool seems_ok = false;
  for (int i = 0; !seems_ok && i < 4; i++) {
    debugPrintln("GPS,101");
    ss->write("$PMTK101*32\r\n");
    ss->flush();
    unsigned long start = millis();
    while (!seems_ok && millis() - start < 500) {
      while (ss->available()){
        ss->read();
        seems_ok = true;
      }
      delay(1);
    }
  }
  for (unsigned long start = millis(); millis() - start < 1000;) {
    while (ss->available()) {
      ss->read();
    }
    delay(1);
  }
  debugPrintln("GPS,251");
  ss->write("$PMTK251,115200*1F\r\n");
  ss->flush();
  debugPrintln("GPS,115200");
  ss->updateBaudRate(115200);
  for (unsigned long start = millis(); millis() - start < 500;) {
    while (ss->available()) {
      ss->read();
    }
    delay(1);
  }
  seems_ok = false;
  for (int i = 0; !seems_ok && i < 5; i++) {
    debugPrintln("GPS,101");
    ss->write("$PMTK101*32\r\n");
    ss->flush();
    unsigned long start = millis();
    while (!seems_ok && millis() - start < 500) {
      while (ss->available()){
        ss->read();
        seems_ok = true;
      }
      delay(1);
    }
  }
  for (unsigned long start = millis(); millis() - start < 1000;) {
    while (ss->available()) {
      ss->read();
    }
    delay(1);
  }
  debugPrintln("GPS,225");
  ss->write("$PMTK225,0*2B\r\n");
  ss->write("$PMTK220,100*2F\r\n");
  ss->write("$PMTK300,100,0,0,0,0*2C\r\n");
  ss->write("$PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*28\r\n");
  ss->flush();
  const char* pmtk001 = "$PMTK001,314,3*36";
  seems_ok = false;
  int pos = 0;
  unsigned long start = millis();
  unsigned long ms = 3000;
  while (!seems_ok && millis() - start < ms) {
    char readValue = ss->read();
    delay(1);
    if (readValue == pmtk001[pos]) {
      pos++;
    } else {
      pos = 0;
    }
    if (pos == 17) {
      debugPrintln("GPS,pmtk001");
      seems_ok = true;
    }
    delay(1);
  }
  debugPrintln("GPS,setup end");
  return seems_ok;
}

void GPS::encode() {
  while (ss->available()) {
    gps.encode(ss->read());
  }
}

void GPS::getHeader(String& buffer) {
  buffer += String("GPS,Sats,HDOP,Latitude,Longitude,Fix Age,DateTime,DateAge,Alt,Course,Speed,Card,DistanceToG,CourseToG,CardToG,CharsRX,SentencesRX,ChecksumFail\n");
}

void GPS::readValues(String& buffer) {
  buffer += "GPS,";

  if (gps.satellites.isValid()) {
    buffer += String(gps.satellites.value());
  }
  buffer += String(",");

  if (gps.hdop.isValid()) {
    hdop = gps.hdop.hdop();
    buffer += String(hdop, 6);
  }
  buffer += String(",");

  locationIsValid = gps.location.isValid();
  if (locationIsValid) {
    lat = gps.location.lat() + state->latOffset;
    buffer += String(lat, 6);
  }
  buffer += String(",");

  if (locationIsValid) {
    lng = gps.location.lng() + state->lngOffset;
    buffer += String(lng, 6);
  }
  buffer += String(",");

  if (locationIsValid) {
    buffer += String(gps.location.age());
  }
  buffer += String(",");

  if (gps.date.isValid())
  {
    buffer += String(gps.date.year());
    buffer += String("-");
    uint8_t month = gps.date.month();
    if (month < 10) {
      buffer += String("0");
    }
    buffer += String(month);
    buffer += String("-");
    uint8_t day = gps.date.day();
    if (day < 10) {
      buffer += String("0");
    }
    buffer += String(day);
    buffer += String("T");
  }
  if (gps.time.isValid())
  {
    uint8_t hour = gps.time.hour();
    if (hour < 10) {
      buffer += String("0");
    }
    buffer += String(hour);
    buffer += String(":");
    uint8_t minute = gps.time.minute();
    if (minute < 10) {
      buffer += String("0");
    }
    buffer += String(minute);
    buffer += String(":");
    uint8_t second = gps.time.second();
    if (second < 10) {
      buffer += String("0");
    }
    buffer += String(second);
  }
  buffer += String(",");

  if (gps.date.isValid())
  {
    buffer += String(gps.date.age());
  }
  buffer += String(",");


  if (gps.altitude.isValid()) {
    buffer += String(gps.altitude.meters(), 6);
  }
  buffer += String(",");

  if (gps.course.isValid()) {
    course = gps.course.deg();
    buffer += String(course, 6);
  }
  buffer += String(",");

  if (gps.speed.isValid()) {
    speedKmph = gps.speed.kmph();
    buffer += String(speedKmph, 6);
  }
  buffer += String(",");

  if (gps.course.isValid()) {
    buffer += String(TinyGPSPlus::cardinal(gps.course.deg()));
  }
  buffer += String(",");

  distanceToGoal =
    TinyGPSPlus::distanceBetween(
      lat,
      lng,
      state->goalLat,
      state->goalLong);

  if (locationIsValid) {
    buffer += String(distanceToGoal, 6);
  }
  buffer += String(",");

  courseToGoal =
    TinyGPSPlus::courseTo(
      lat,
      lng,
      state->goalLat,
      state->goalLong);

  if (locationIsValid) {
    buffer += String(courseToGoal, 6);
  }
  buffer += String(",");

  const char *cardinalToGoal = TinyGPSPlus::cardinal(courseToGoal);

  if (locationIsValid) {
    buffer += String(cardinalToGoal);
  }
  buffer += String(",");

  buffer += String(gps.charsProcessed());
  buffer += String(",");
  buffer += String(gps.sentencesWithFix());
  buffer += String(",");
  buffer += String(gps.failedChecksum());
  buffer += String("\n");
}

void GPS::debugPrintln(const char * message) {
  #if GPS_DEBUG == true
  Serial.println(message);
  #endif
}
