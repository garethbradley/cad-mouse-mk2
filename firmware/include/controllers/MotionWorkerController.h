#pragma once

#include "pico/mutex.h"

struct MotionSnapshot {
  float motion[6] = {};
  float measuredField[9] = {};
  float predictedField[9] = {};
  float estimatedPose[6] = {};
  float residualRms = 0.0f;
  float covarianceTrace = 0.0f;
  bool motionActive = false;
  bool valid = false;
};

struct MotionWorkerTiming {
  uint32_t loopCycleUs = 0;
  uint32_t jacobianUs = 0;
  uint32_t snapshotAgeMs = 0;
  uint32_t minCycleUs = 0xFFFFFFFFU;
  uint32_t maxCycleUs = 0;
  uint32_t avgCycleUs = 0;
  uint32_t numSamples = 0;
};

class MotionWorkerController {
 public:
  void begin();
  void loop();
  void setEnabled(bool enabled);
  void stopAndWait();
  bool latestSnapshot(MotionSnapshot* out) const;
  MotionWorkerTiming timing() const;

 private:
  mutable mutex_t snapshotMutex_;
  MotionSnapshot snapshot_;
  MotionWorkerTiming timing_;
  unsigned long snapshotTimestampMs_ = 0;

  volatile bool enabled_ = false;
  volatile bool workerIdle_ = true;
  unsigned long lastUpdateMs_ = 0;
};