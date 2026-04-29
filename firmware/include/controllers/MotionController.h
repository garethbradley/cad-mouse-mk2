#pragma once

#include "controllers/EKFEngine.h"

class MotionController {
 public:
  void reset();
  void compute(const float raw[9], const float* baseline, float dt, float out[6]);
  void computeExpectedField(float out[9]) const;
  void estimatorPose(float out[6]) const;
  float estimatorCovarianceTrace() const;
  bool hasMotionActivity() const;
  const EKFEngine& estimator() const;

 private:
  static float clampf(float v, float lo, float hi);
  static float axisBaseDead(int i);
  static float sensitivityCurve(float value, float dead, float limit);

  // Per-axis Kalman filter state
  float kalmanX_[6] = {};  // Estimated state
  float kalmanP_[6] = {};  // Estimate uncertainty (covariance)
  float kalmanStep(int axis, float measurement);
  EKFEngine estimator_;

  bool motionActive_ = false;
};
