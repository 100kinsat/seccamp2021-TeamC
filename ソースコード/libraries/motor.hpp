#pragma once

#include "Arduino.h"

class Motor {
  public:
    Motor();

    void move(int pwm_a, int pwm_b);
    void forward(int pwm); // 前進
    void back(int pwm); // 後退
    void turn(bool cw, int pwm);
    void stop(); // 停止

  private:
    const int MOTOR_A[3] = {13, 4, 25}; // AIN1, AIN2, PWMA
    const int MOTOR_B[3] = {14, 27, 26}; // BIN1, BIN2, PWMB

    const int CHANNEL_A = 0;
    const int CHANNEL_B = 1;

    const int LEDC_TIMER_BIT = 8;
    const int LEDC_BASE_FREQ = 490;
    const int LEDC_VALUE_MAX = (1 << LEDC_TIMER_BIT) - 1;

    const int VALUE_MIN = 30;
};

Motor::Motor() {
  for (int i = 0; i < 3; i++) {
    pinMode(MOTOR_A[i], OUTPUT);
    pinMode(MOTOR_B[i], OUTPUT);
  }

  ledcSetup(CHANNEL_A, LEDC_BASE_FREQ, LEDC_TIMER_BIT);
  ledcSetup(CHANNEL_B, LEDC_BASE_FREQ, LEDC_TIMER_BIT);

  ledcAttachPin(MOTOR_A[2], CHANNEL_A);
  ledcAttachPin(MOTOR_B[2], CHANNEL_B);
}

void Motor::move(int pwm_left, int pwm_right) {
  if (std::abs(pwm_left) <= VALUE_MIN && std::abs(pwm_right) <= VALUE_MIN && pwm_left != pwm_right) {
    if ((0 < pwm_left && 0 < pwm_right) || (pwm_left < 0 && pwm_right < 0)) {
      if (std::abs(pwm_left) < std::abs(pwm_right)) {
        pwm_left = 0;
      } else {
        pwm_right = 0;
      }
    }
  }
  // 左モータ
  if (pwm_left == 0) {
    digitalWrite(MOTOR_A[0], LOW);
    digitalWrite(MOTOR_A[1], LOW);
    ledcWrite(CHANNEL_A, HIGH);
  } else {
    bool cw = pwm_left < 0;
    int value = std::min(std::abs(pwm_left), LEDC_VALUE_MAX);
    value = std::max(value, VALUE_MIN);
    digitalWrite(MOTOR_A[(int)!cw], LOW);
    digitalWrite(MOTOR_A[(int) cw], HIGH);
    ledcWrite(CHANNEL_A, value);
  }

  // 右モータ
  if (pwm_right == 0) {
    digitalWrite(MOTOR_B[0], LOW);
    digitalWrite(MOTOR_B[1], LOW);
    ledcWrite(CHANNEL_B, HIGH);
  } else {
    bool cw = pwm_right < 0;
    int value = std::min(std::abs(pwm_right), LEDC_VALUE_MAX);
    value = std::max(value, VALUE_MIN);
    digitalWrite(MOTOR_B[(int)!cw], LOW);
    digitalWrite(MOTOR_B[(int) cw], HIGH);
    ledcWrite(CHANNEL_B, value);
  }
}

/**
 * 前進
 */
void Motor::forward(int pwm) {
  move(pwm, pwm);
}

/**
 * 後退
 */
void Motor::back(int pwm) {
  move(-pwm, -pwm);
}

void Motor::turn(bool cw, int pwm) {
  move(pwm * (cw ? 1 : -1), pwm * (cw ? -1 : 1));
}

/**
 * 停止
 */
void Motor::stop() {
  move(0, 0);
}
