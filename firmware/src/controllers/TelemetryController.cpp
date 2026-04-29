#include "controllers/TelemetryController.h"
#include <Arduino.h>
#include "Config.h"
#include "controllers/MotionWorkerController.h"

namespace {
const int kPrintEvery = 5;

void printFieldVector(const char* prefix, const float field[9]) {
  for (int sensor = 0; sensor < 3; ++sensor) {
    const int base = sensor * 3;
    Serial.print(prefix);
    Serial.print(sensor + 1);
    Serial.print('x');
    Serial.print(':');
    Serial.println(field[base + 0]);

    Serial.print(prefix);
    Serial.print(sensor + 1);
    Serial.print('y');
    Serial.print(':');
    Serial.println(field[base + 1]);

    Serial.print(prefix);
    Serial.print(sensor + 1);
    Serial.print('z');
    Serial.print(':');
    Serial.println(field[base + 2]);
  }
}
}

void TelemetryController::begin() {
  tick_ = 0;

  if (!enabled()) {
    return;
  }

  Serial.println(">boot:ok");
  Serial.print(">mode:");
  Serial.println(Config::ENABLE_SENSORLESS_PROFILING ? "sensorless" : "hardware");
  Serial.print(">dualCore:");
  Serial.println(Config::ENABLE_DUAL_CORE_MOTION ? 1 : 0);
  Serial.print(">ekfOut:");
  Serial.println(Config::USE_EKF_FOR_HID_OUTPUT ? 1 : 0);
}

bool TelemetryController::enabled() const {
  return Config::TELEMETRY_LOG_LEVEL != Config::TELEMETRY_LOG_OFF;
}

void TelemetryController::publish(const float motion[6], const float measuredField[9],
                                  const float predictedField[9],
                                  const float estimatedPose[6], float residualRms,
                                  float covarianceTrace, int buttonBits,
                                  bool hidReportSent, const MotionWorkerTiming* timing) {
  if (!enabled()) {
    return;
  }

  tick_++;
  if ((tick_ % kPrintEvery) != 0) {
    return;
  }

  Serial.print(">X:");
  Serial.println(motion[0]);
  Serial.print(">Y:");
  Serial.println(motion[1]);
  Serial.print(">Z:");
  Serial.println(motion[2]);
  Serial.print(">Rx:");
  Serial.println(motion[3]);
  Serial.print(">Ry:");
  Serial.println(motion[4]);
  Serial.print(">Rz:");
  Serial.println(motion[5]);
  Serial.print(">btn:");
  Serial.println(buttonBits & 0x0003);
  Serial.print(">hid:");
  Serial.println(hidReportSent ? 1 : 0);

  Serial.print(">Etx:");
  Serial.println(estimatedPose[0]);
  Serial.print(">Ety:");
  Serial.println(estimatedPose[1]);
  Serial.print(">Etz:");
  Serial.println(estimatedPose[2]);
  Serial.print(">Erx:");
  Serial.println(estimatedPose[3]);
  Serial.print(">Ery:");
  Serial.println(estimatedPose[4]);
  Serial.print(">Erz:");
  Serial.println(estimatedPose[5]);
  Serial.print(">Erms:");
  Serial.println(residualRms);
  Serial.print(">Ptr:");
  Serial.println(covarianceTrace);

  if (timing != nullptr) {
    Serial.print(">tCycle:");
    Serial.println(timing->loopCycleUs);
    Serial.print(">tJacobian:");
    Serial.println(timing->jacobianUs);
    Serial.print(">tMin:");
    Serial.println(timing->minCycleUs);
    Serial.print(">tMax:");
    Serial.println(timing->maxCycleUs);
    Serial.print(">tAvg:");
    Serial.println(timing->avgCycleUs);
  }

  if (Config::TELEMETRY_LOG_LEVEL >= Config::TELEMETRY_LOG_FULL) {
    printFieldVector(">m", measuredField);
    printFieldVector(">p", predictedField);
  }
}
