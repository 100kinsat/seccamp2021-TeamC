#pragma once

#include "MPU9250.h"

#define LED 2
#define FLIGHT_PIN 34
#define BUTTON 35

class CanSatIO {
  public:
    CanSatIO();

    void setLEDOn();
    void setLEDOff();
    bool isFlightPinInserted();
    bool isButtonJustPressed();

  private:
    bool isButtonPressedPrev = false;
    bool isButtonPressedNow = false;

    bool isButtonPressed();
};

CanSatIO::CanSatIO() {
  pinMode(LED, OUTPUT);
  pinMode(FLIGHT_PIN, INPUT);
  pinMode(BUTTON, INPUT);
}

void CanSatIO::setLEDOn() {
  digitalWrite(LED, HIGH);
}

void CanSatIO::setLEDOff() {
  digitalWrite(LED, LOW);
}

bool CanSatIO::isFlightPinInserted() {
  return !digitalRead(FLIGHT_PIN);
}

bool CanSatIO::isButtonJustPressed() {
  isButtonPressedPrev = isButtonPressedNow;
  isButtonPressedNow = isButtonPressed();
  return !isButtonPressedPrev && isButtonPressedNow;
}

bool CanSatIO::isButtonPressed() {
  return !digitalRead(BUTTON);
}
