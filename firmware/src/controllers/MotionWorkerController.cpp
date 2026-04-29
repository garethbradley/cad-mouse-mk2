#include "controllers/MotionWorkerController.h"

#include <Arduino.h>
#include <math.h>
#include "pico/time.h"

#include "Config.h"
#include "Controllers.h"

void MotionWorkerController::begin() {
  mutex_init(&snapshotMutex_);
  lastUpdateMs_ = 0;
  enabled_ = false;
  workerIdle_ = true;
  snapshotTimestampMs_ = 0;

  MotionSnapshot empty{};
  mutex_enter_blocking(&snapshotMutex_);
  snapshot_ = empty;
  mutex_exit(&snapshotMutex_);

  MotionWorkerTiming emptyTiming{};
  timing_ = emptyTiming;
}

void MotionWorkerController::loop() {
  if (!Config::ENABLE_DUAL_CORE_MOTION || !enabled_) {
    workerIdle_ = true;
    lastUpdateMs_ = 0;
    return;
  }

  workerIdle_ = false;
  const uint64_t cycleStartUs = time_us_64();

  const unsigned long now = millis();
  const float dt = (lastUpdateMs_ == 0) ? 0.01f
                                        : ((now - lastUpdateMs_) / 1000.0f);
  lastUpdateMs_ = now;

  float raw[9] = {};
  if (!sensorController.readRaw(raw)) {
    workerIdle_ = true;
    return;
  }

  const float* baseline = sensorController.baseline();
  MotionSnapshot next{};
  for (int i = 0; i < 9; ++i) {
    next.measuredField[i] = raw[i] - baseline[i];
  }

  const uint64_t jacobianStartUs = time_us_64();
  motionController.compute(raw, baseline, dt, next.motion);
  const uint64_t jacobianEndUs = time_us_64();

  motionController.computeExpectedField(next.predictedField);
  motionController.estimatorPose(next.estimatedPose);
  next.covarianceTrace = motionController.estimatorCovarianceTrace();
  next.motionActive = motionController.hasMotionActivity();

  float residualEnergy = 0.0f;
  for (int i = 0; i < 9; ++i) {
    const float residual = next.measuredField[i] - next.predictedField[i];
    residualEnergy += residual * residual;
  }
  next.residualRms = sqrtf(residualEnergy / 9.0f);
  next.valid = true;

  snapshotTimestampMs_ = now;
  mutex_enter_blocking(&snapshotMutex_);
  snapshot_ = next;
  mutex_exit(&snapshotMutex_);

  const uint64_t cycleEndUs = time_us_64();
  const uint32_t cycleDurationUs = (uint32_t)(cycleEndUs - cycleStartUs);
  const uint32_t jacobianDurationUs = (uint32_t)(jacobianEndUs - jacobianStartUs);

  timing_.loopCycleUs = cycleDurationUs;
  timing_.jacobianUs = jacobianDurationUs;
  if (cycleDurationUs < timing_.minCycleUs) {
    timing_.minCycleUs = cycleDurationUs;
  }
  if (cycleDurationUs > timing_.maxCycleUs) {
    timing_.maxCycleUs = cycleDurationUs;
  }
  timing_.numSamples++;
  if (timing_.numSamples > 0) {
    timing_.avgCycleUs =
        (timing_.avgCycleUs * (timing_.numSamples - 1) + cycleDurationUs) /
        timing_.numSamples;
  }

  workerIdle_ = true;
}

void MotionWorkerController::setEnabled(bool enabled) { enabled_ = enabled; }

void MotionWorkerController::stopAndWait() {
  enabled_ = false;
  while (!workerIdle_) {
    yield();
  }
}

bool MotionWorkerController::latestSnapshot(MotionSnapshot* out) const {
  if (out == nullptr) {
    return false;
  }

  mutex_enter_blocking(&snapshotMutex_);
  *out = snapshot_;
  mutex_exit(&snapshotMutex_);
  return out->valid;
}

MotionWorkerTiming MotionWorkerController::timing() const { return timing_; }