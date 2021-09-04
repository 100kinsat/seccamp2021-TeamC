#include "./libraries/state.hpp"
#include "./libraries/sd_log.hpp"
#include "./libraries/motor.hpp"
#include "./libraries/gps.hpp"
#include "./libraries/imu.hpp"
#include "./libraries/cansat_io.hpp"
#include "./libraries/log_task.hpp"
#include <cmath>

// 設定
#define ENABLE_GPS_LOG true
#define REDUCE_MPU_LOG true

State state;
SdLog sdLog;
Motor motor;
GPS gps(&state);
IMU imu(&state);
CanSatIO canSatIO;
LogTask logTask(&sdLog, &canSatIO, &state);

void setup() {
  Serial.begin(115200);
  delay(100);
  Serial.println("start");

  imu.setup();

  // ゴールの緯度経度
  state.setGoal(35.751380, 139.873006);
  state.setPlannedStart(35.750732, 139.873890);

  // キャリブレーション結果に値を変更してください
  imu.mpu.setAccBias(-765.75, -131.07, -222.66);
  imu.mpu.setGyroBias(0.96, -3.33, -2.14);
  imu.mpu.setMagBias(-869.968079, 689.569855, 222.993191);
  // imu.mpu.setMagScale(1, 1, 1);

  if (!logTask.setupTask()) {
    logTask.restartOnError(3);
  }

  createNewLogFile();

  logTask.sendToLoggerTask("State,GPS check.\n", false);
  if (!gps.changeUpdateInterval()) {
    logTask.sendToLoggerTask("State,GPS initialization failed.\n", false);
    logTask.restartOnError(4);
  }
  logTask.sendToLoggerTask("State,GPS OK.\n", false);

  if (!imu.checkValue(5)) {
    logTask.sendToLoggerTask("State,IMU initialization failed.\n", false);
    logTask.restartOnError(5);
  }
  logTask.sendToLoggerTask("State,IMU OK.\n", false);
  imu.magCalibrateInitialize();

  if (!logTask.checkSdHealth()) {
    Serial.println("SD initialization failed.");
    logTask.restartOnError(6);
  }

  logTask.sendToLoggerTask("State,Wait for GPS fix.\n", false);
  canSatIO.setLEDOn();
  state.enableSdLog = false;
  state.vehicleMode = VehicleMode::Stop;
  for (unsigned long start = millis(), lastLog = 0; millis() - start < 2 * 60 * 1000;) {
    gps.encode();
    if (1000 < millis() - lastLog) {
      lastLog = millis();
      String buffer = "";
      gps.readValues(buffer);
      logTask.sendToLoggerTask(buffer, false);
      if (gps.locationIsValid && gps.hdop != 0.0 && gps.hdop <= state.gpsCompensationHdopLimit) {
        logTask.sendToLoggerTask("State,GPS signal is fine.\n", false);
        break;
      }
      if (state.vehicleMode != VehicleMode::Stop) {
        logTask.sendToLoggerTask("State,State was changed.\n", false);
        break;
      }
    }
  }
  state.setGpsOffsetByPlannedStart(gps.lat, gps.lng);
  logTask.sendToLoggerTask("Log,gps offset," + String(state.latOffset, 6) + "," + String(state.lngOffset, 6) + "," + String(gps.lat, 6) + "," + String(gps.lng, 6)  +"\n", false);

  findNearestSubgoal();
  canSatIO.setLEDOff();
  state.enableSdLog = true;
  if (state.vehicleMode == VehicleMode::Stop) {
    state.vehicleMode = VehicleMode::Mission;
  }

  logTask.sendToLoggerTask("State,setup ended.\n", false);
}

void findNearestSubgoal() {
  int minimumIndex = 0;
  state.startDistance = TinyGPSPlus::distanceBetween(
    state.plannedStartLat,
    state.plannedStartLng,
    state.goalLat,
    state.goalLong
  );
  state.subgoalIndexMax = static_cast<int>(state.startDistance);
  double minimumDistance = TinyGPSPlus::distanceBetween(
    gps.lat,
    gps.lng,
    state.plannedStartLat,
    state.plannedStartLng
  );
  double latDiff = state.goalLat - state.plannedStartLat;
  double lngDiff = state.goalLong - state.plannedStartLng;
  for (int i = 0; i <= state.subgoalIndexMax; i++) {
    double ratio = (static_cast<double>(i) / static_cast<double>(state.subgoalIndexMax));
    double distance = TinyGPSPlus::distanceBetween(
      gps.lat,
      gps.lng,
      state.plannedStartLat + (latDiff * ratio),
      state.plannedStartLng + (lngDiff * ratio)
    );
    if (distance < minimumDistance) {
      minimumIndex = i;
      minimumDistance = distance;
    }
  }

  state.subgoalIndex = minimumIndex;
  calculateSubgoal();
}

void calculateSubgoal() {
  double latDiff = state.goalLat - state.plannedStartLat;
  double lngDiff = state.goalLong - state.plannedStartLng;
  double ratio = (static_cast<double>(state.subgoalIndex) / static_cast<double>(state.subgoalIndexMax));
  state.subgoalLat = state.plannedStartLat + (latDiff * ratio);
  state.subgoalLng = state.plannedStartLng + (lngDiff * ratio);
}

void loop() {
  if (canSatIO.isFlightPinInserted()) {
    state.motorLeft = state.currentSpeed;
    state.motorRight = state.currentSpeed;
  } else {
    state.motorLeft = 0;
    state.motorRight = 0;
  }

  const unsigned long ms = 100;
  unsigned long start = millis();

  double yawSum = 0.0;
  int yawItems = 0;

  while (millis() - start < ms) {
    gps.encode();

    if (imu.update()) {
      if (!REDUCE_MPU_LOG) {
        String buffer = "";
        imu.getLogString(buffer);
        logTask.sendToLoggerTask(buffer, true);
      }
      yawSum += imu.yaw;
      yawItems++;
    }

    if (canSatIO.isButtonJustPressed()) {
      // スイッチが押された
      Serial.println("Create new log file");

      logTask.closeFile();
      createNewLogFile();

      imu.magCalibrateInitialize();
      state.vehicleMode = VehicleMode::Mission;

      break;
    }
    state.checkYawForGpsCompensation();
  }

  if (REDUCE_MPU_LOG) {
    String buffer = "";
    imu.getLogString(buffer);
    logTask.sendToLoggerTask(buffer, true);
  }
  String buffer = "";
  double previousGpsYaw = gps.course;
  gps.readValues(buffer);
  state.lat = gps.lat;
  state.lng = gps.lng;
  if (ENABLE_GPS_LOG) {
    logTask.sendToLoggerTask(buffer, false);
  }
  // ゴールの向きにtargetYawを設定
  if (state.vehicleMode == VehicleMode::Mission) {
    state.targetYaw = gps.courseToGoal;
    if (gps.distanceToGoal < 0.5) {
      // 終了条件
      if (state.arrivedGoalAt == 0) {
        state.arrivedGoalAt = millis();
        logTask.sendToLoggerTask("Control,Arrived at a goal. Wait for missionCompleteDecisionDuration," + String(state.missionCompleteDecisionDuration) + "\n", false);
      } else if (state.missionCompleteDecisionDuration * 1000 < millis() - state.arrivedGoalAt) {
        state.vehicleMode = VehicleMode::Completed;
        logTask.sendToLoggerTask("Log,Mission Completed.\n", false);
      }
    } else if (gps.distanceToGoal < 1.0) {
      state.arrivedGoalAt = 0;
      state.currentSpeed = state.defaultSpeed * 0.33;
        logTask.sendToLoggerTask("Control,Changed speed ratio to 0.33.\n", false);
    } else if (gps.distanceToGoal < 3.0) {
      state.arrivedGoalAt = 0;
      state.currentSpeed = state.defaultSpeed * 0.66;
        logTask.sendToLoggerTask("Control,Changed speed ratio to 0.66.\n", false);
    } else {
      state.arrivedGoalAt = 0;
      state.currentSpeed = state.defaultSpeed;
    }
    if (previousGpsYaw != gps.course && 0.3 < gps.speedKmph && gps.hdop <= state.gpsCompensationHdopLimit) {
      state.doGpsCompensation();
    }

    if (state.subgoalIndex != state.subgoalIndexMax) {
      double distanceToSubgoal = TinyGPSPlus::distanceBetween(
        gps.lat,
        gps.lng,
        state.subgoalLat,
        state.subgoalLng
      );
      if (distanceToSubgoal < state.subgoalIncrementThreshold) {
        state.subgoalIndex++;
        calculateSubgoal();
      }

      if (state.subgoalFallbackThreshold < distanceToSubgoal) {
        findNearestSubgoal();
      }

      double courseToSubgoal = TinyGPSPlus::courseTo(
        gps.lat,
        gps.lng,
        state.subgoalLat,
        state.subgoalLng
      );
      state.targetYaw = courseToSubgoal;

      distanceToSubgoal = TinyGPSPlus::distanceBetween(
        gps.lat,
        gps.lng,
        state.subgoalLat,
        state.subgoalLng
      );

      logTask.sendToLoggerTask("Log,subgoal," + String(state.subgoalLat, 6) + "," + String(state.subgoalLng, 6) + "," + String(distanceToSubgoal, 6)  +"\n", false);
    }
  }
  double yawAverage = imu.yaw;
  if (yawItems != 0) {
    yawAverage = yawSum / yawItems;
  }
  state.yawAverage = yawAverage;
  state.gpsYawDiff = state.clipYawDiff(yawAverage - gps.course);
  state.goalDistance = gps.distanceToGoal;

  // state.targetYaw に向ける
  state.yawDiff = state.clipYawDiff(state.targetYaw - yawAverage);
  if (state.yawDiff < -state.yawDiffThreshold) {
    canSatIO.setLEDOff();
    if (state.yawDiff < -state.yawDiffTurnThreshold) {
      state.motorLeft = -state.currentSpeed;
      state.motorRight = state.currentSpeed;
      logTask.sendToLoggerTask("Control,Motor Control,turn ccw," + String(state.yawDiff, 6) + "\n", false);
    } else {
      state.motorLeft = state.currentSpeed - std::abs(state.yawDiff) * 2;
      state.motorRight = state.currentSpeed;
      logTask.sendToLoggerTask("Control,Left motor control," + String(state.yawDiff, 6) + "," + String(-static_cast<int>(std::abs(state.yawDiff) * 2)) + "\n", false);
    }
  } else if (state.yawDiffThreshold < state.yawDiff) {
    canSatIO.setLEDOff();
    if (state.yawDiffTurnThreshold < state.yawDiff) {
      state.motorLeft = state.currentSpeed;
      state.motorRight = -state.currentSpeed;
      logTask.sendToLoggerTask("Control,Motor Control,turn cw," + String(state.yawDiff, 6) + "\n", false);
    } else {
      state.motorLeft = state.currentSpeed;
      state.motorRight = state.currentSpeed - std::abs(state.yawDiff) * 2;
      logTask.sendToLoggerTask("Control,Right motor control," + String(state.yawDiff, 6) + "," + String(-static_cast<int>(std::abs(state.yawDiff) * 2)) + "\n", false);
    }
  } else {
    canSatIO.setLEDOn();
    state.motorLeft = state.currentSpeed;
    state.motorRight = state.currentSpeed;
  }

  state.flightPinInserted = canSatIO.isFlightPinInserted();

  state.detectFallDown = imu.roll < -45 || 45 < imu.roll;

  if (
    !state.flightPinInserted ||
    state.detectFallDown ||
    state.vehicleMode == VehicleMode::Stop ||
    state.vehicleMode == VehicleMode::Completed
  ) {
    state.motorLeft = 0;
    state.motorRight = 0;
  }
  motor.move(state.motorLeft, state.motorRight);
  buffer = "";
  state.getLogString(buffer);
  logTask.sendToLoggerTask(buffer, false);

  if (state.requestMagCalibrate) {
    calibrateMag();
  }
}

void createNewLogFile() {
  int current_log_number = logTask.openNextLogFile(SD);

  Serial.print("Next number: ");
  Serial.println(current_log_number);

  if (ENABLE_GPS_LOG) {
    String message = "";
    gps.getHeader(message);
    logTask.sendToLoggerTask(message, false);
  }

  String message = "";
  imu.getHeader(message);
  logTask.sendToLoggerTask(message, false);

  message = "";
  state.getHeader(message);
  logTask.sendToLoggerTask(message, false);

  if (!logTask.checkSdHealth()) {
    Serial.println("SD initialization failed.");
    logTask.restartOnError(6);
  }
}

void calibrateMag() {
  String buffer = "";
  imu.magCalibrateApply(buffer);
  logTask.sendToLoggerTask(buffer, false);
  state.requestMagCalibrate = false;
}
