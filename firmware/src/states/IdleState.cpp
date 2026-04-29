#include "states/IdleState.h"

#include <Arduino.h>
#include <math.h>

#include "Config.h"
#include "Controllers.h"
#include "StateMachine.h"

void IdleState::enter() {
  lastUpdateMs_ = 0;
  lastActivityMs_ = millis();
  ledController.setSolid(Config::LED_IDLE_COLOR);

  if (Config::ENABLE_DUAL_CORE_MOTION) {
    motionWorkerController.setEnabled(true);
  }
}

bool IdleState::handleCalibrationRequest() {
  if (inputController.takeCalibrationRequest()) {
    stateMachine.changeState(&StateMachine::calibratingState);
    return true;
  }
  return false;
}

void IdleState::runMotionPipeline(float dt, unsigned long now) {
  if (Config::ENABLE_DUAL_CORE_MOTION) {
    MotionSnapshot snapshot;
    if (!motionWorkerController.latestSnapshot(&snapshot)) {
      return;
    }

    if (snapshot.motionActive) {
      lastActivityMs_ = now;
    }

    const uint16_t buttonBits = inputController.buttonBits();
    const bool hidReportSent = hidController.sendReports(snapshot.motion, buttonBits);
    if (telemetryController.enabled()) {
      MotionWorkerTiming timing = motionWorkerController.timing();
      telemetryController.publish(snapshot.motion, snapshot.measuredField,
                                  snapshot.predictedField, snapshot.estimatedPose,
                                  snapshot.residualRms, snapshot.covarianceTrace,
                                  buttonBits, hidReportSent, &timing);
    }
    return;
  }

  float raw[9] = {};
  if (!sensorController.readRaw(raw)) {
    return;  // Skip frame on sensor read failure
  }

  float measuredField[9] = {};
  const float* baseline = sensorController.baseline();
  for (int i = 0; i < 9; ++i) {
    measuredField[i] = raw[i] - baseline[i];
  }

  float motion[6] = {};
  motionController.compute(raw, baseline, dt, motion);

  float predictedField[9] = {};
  motionController.computeExpectedField(predictedField);

  float residualEnergy = 0.0f;
  for (int i = 0; i < 9; ++i) {
    const float residual = measuredField[i] - predictedField[i];
    residualEnergy += residual * residual;
  }
  const float residualRms = sqrtf(residualEnergy / 9.0f);

  float estimatorPose[6] = {};
  motionController.estimatorPose(estimatorPose);
  const float covarianceTrace = motionController.estimatorCovarianceTrace();

  if (motionController.hasMotionActivity()) {
    lastActivityMs_ = now;
  }

  const uint16_t buttonBits = inputController.buttonBits();
  const bool hidReportSent = hidController.sendReports(motion, buttonBits);
  if (telemetryController.enabled()) {
    telemetryController.publish(motion, measuredField, predictedField,
                                estimatorPose, residualRms, covarianceTrace,
                                buttonBits, hidReportSent, nullptr);
  }
}

void IdleState::handleSleepTransition(unsigned long now) {
  const unsigned long inactiveMs = now - lastActivityMs_;
  if (inactiveMs >= Config::IDLE_SLEEP_TIMEOUT_MS) {
    stateMachine.changeState(&StateMachine::sleepState);
  }
}

void IdleState::update() {
  inputController.update();

  if (handleCalibrationRequest()) {
    return;
  }

  const unsigned long now = millis();
  if (inputController.takeActivity()) {
    lastActivityMs_ = now;
  }

  const float dt = (lastUpdateMs_ == 0) ? 0.01
                                        : ((now - lastUpdateMs_) / 1000.0);
  lastUpdateMs_ = now;
  runMotionPipeline(dt, now);
  handleSleepTransition(now);
}

void IdleState::exit() {
  if (Config::ENABLE_DUAL_CORE_MOTION) {
    motionWorkerController.stopAndWait();
  }
}
