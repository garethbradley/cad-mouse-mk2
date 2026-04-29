#include "controllers/MotionController.h"

#include <Arduino.h>
#include <math.h>

#include "Config.h"

namespace {
enum RawIndex {
  RAW_MAG1_X = 0,
  RAW_MAG1_Y,
  RAW_MAG1_Z,
  RAW_MAG2_X,
  RAW_MAG2_Y,
  RAW_MAG2_Z,
  RAW_MAG3_X,
  RAW_MAG3_Y,
  RAW_MAG3_Z
};

enum AxisIndex {
  AXIS_TX = 0,
  AXIS_TY,
  AXIS_TZ,
  AXIS_RX,
  AXIS_RY,
  AXIS_RZ
};

// Precomputed geometric constants for the equilateral sensor triangle.
const float kOneThird = 1.0f / 3.0f;
const float kSqrt3 = 1.7320508f;          // sqrt(3)
const float kSqrt3Over6 = 0.28867513f;    // sqrt(3)/6
const float kSqrt3Over3 = 0.57735027f;    // sqrt(3)/3
const float kMag2PosX = -0.5f;
const float kMag2PosY = kSqrt3Over6;
const float kMag3PosX = 0.5f;
const float kMag3PosY = kSqrt3Over6;
const float kMag1PosX = 0.0f;
const float kMag1PosY = -kSqrt3Over3;
}  // namespace

void MotionController::reset() {
  estimator_.reset();
  for (int i = 0; i < 6; i++) {
    kalmanX_[i] = 0.0f;
    kalmanP_[i] = 1.0f;
  }
  motionActive_ = false;
}

float MotionController::clampf(float v, float lo, float hi) {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

float MotionController::kalmanStep(int axis, float measurement) {
  // Predict step: uncertainty grows by process noise
  kalmanP_[axis] += Config::KALMAN_Q;

  // Update step: compute Kalman gain
  const float K = kalmanP_[axis] / (kalmanP_[axis] + Config::KALMAN_R);

  // Correct estimate with measurement
  kalmanX_[axis] += K * (measurement - kalmanX_[axis]);

  // Update uncertainty
  kalmanP_[axis] *= (1.0f - K);

  return kalmanX_[axis];
}

float MotionController::axisBaseDead(int i) {
  return (i < 3) ? Config::DEAD_T : Config::DEAD_R;
}

float MotionController::sensitivityCurve(float value, float dead, float limit) {
  // Map the post-dead-zone range [dead, limit] onto [0, limit] with a power curve.
  // This gives fine control at small deflections and fast motion at large ones.
  const float sign = (value >= 0.0f) ? 1.0f : -1.0f;
  const float abs_val = fabs(value);
  if (abs_val <= dead) return 0.0f;

  // Normalize to 0..1 within the active range
  const float range = limit - dead;
  if (range <= 0.0f) return 0.0f;
  const float normalized = clampf((abs_val - dead) / range, 0.0f, 1.0f);

  // Apply power curve and scale back to output range
  return sign * powf(normalized, Config::SENSITIVITY_EXP) * limit;
}

void MotionController::compute(const float raw[9], const float* baseline, float dt,
                               float out[6]) {
  estimator_.predict(dt);

  // Baseline subtraction converts magnetic deltas around the calibrated rest pose.
  const float mag1x = raw[RAW_MAG1_X] - baseline[RAW_MAG1_X];
  const float mag1y = raw[RAW_MAG1_Y] - baseline[RAW_MAG1_Y];
  const float mag1z = raw[RAW_MAG1_Z] - baseline[RAW_MAG1_Z];
  const float mag2x = raw[RAW_MAG2_X] - baseline[RAW_MAG2_X];
  const float mag2y = raw[RAW_MAG2_Y] - baseline[RAW_MAG2_Y];
  const float mag2z = raw[RAW_MAG2_Z] - baseline[RAW_MAG2_Z];
  const float mag3x = raw[RAW_MAG3_X] - baseline[RAW_MAG3_X];
  const float mag3y = raw[RAW_MAG3_Y] - baseline[RAW_MAG3_Y];
  const float mag3z = raw[RAW_MAG3_Z] - baseline[RAW_MAG3_Z];

  const float measuredField[9] = {
      mag1x, mag1y, mag1z, mag2x, mag2y, mag2z, mag3x, mag3y, mag3z,
  };
  estimator_.update(measuredField);

  float tx;
  float ty;
  float tz;
  float rx;
  float ry;
  float rz;

  if (Config::USE_EKF_MOTION_OUTPUT) {
    const EKFEngine::PoseState& s = estimator_.state();
    tx = s.tx;
    ty = s.ty;
    tz = s.tz;
    rx = s.rx;
    ry = s.ry;
    rz = s.rz;
  } else {
    // Translation: average of all three sensors.
    tx = (mag1x + mag2x + mag3x) * kOneThird;
    ty = (mag1y + mag2y + mag3y) * kOneThird;
    tz = (mag1z + mag2z + mag3z) * kOneThird;

    // Rotation estimates from sensor triangle geometry.
    //   Ry: side-to-side tilt (right sensor minus left)
    //   Rx: front/back tilt (top pair minus bottom)
    //   Rz: twist (cross-product per sensor position)
    rx = (kSqrt3 * (mag2z + mag3z - 2.0f * mag1z)) * kOneThird;
    ry = (mag3z - mag2z);
    rz =
        (kMag2PosX * mag2y - kMag2PosY * mag2x) +
        (kMag3PosX * mag3y - kMag3PosY * mag3x) +
        (kMag1PosX * mag1y - kMag1PosY * mag1x);
  }

  // Apply sign fixes and gains
  float y[6];
  y[AXIS_TX] = Config::SIGN_AXIS[AXIS_TX] * tx * Config::GAIN_T[AXIS_TX];
  y[AXIS_TY] = Config::SIGN_AXIS[AXIS_TY] * ty * Config::GAIN_T[AXIS_TY];
  y[AXIS_TZ] = Config::SIGN_AXIS[AXIS_TZ] * tz * Config::GAIN_T[AXIS_TZ];
  y[AXIS_RX] = Config::SIGN_AXIS[AXIS_RX] * rx * Config::GAIN_R[AXIS_RX - 3];
  y[AXIS_RY] = Config::SIGN_AXIS[AXIS_RY] * ry * Config::GAIN_R[AXIS_RY - 3];
  y[AXIS_RZ] = Config::SIGN_AXIS[AXIS_RZ] * rz * Config::GAIN_R[AXIS_RZ - 3];

  // Kalman filter, sensitivity curve, dead zones, and clamp.
  motionActive_ = false;
  for (int i = 0; i < 6; i++) {
    const float dead = axisBaseDead(i);

    if (fabs(y[i]) < dead) {
      // Below dead zone: decay Kalman estimate toward zero gradually.
      // Preserve covariance so the filter doesn't jitter at the boundary.
      kalmanX_[i] *= 0.8f;
      kalmanP_[i] = fmin(kalmanP_[i] + Config::KALMAN_Q * 0.1f, 1.0f);
    } else {
      kalmanStep(i, y[i]);
    }

    // Apply sensitivity curve to filtered output
    out[i] = sensitivityCurve(kalmanX_[i], dead, Config::AXIS_LIMIT);
    if (out[i] != 0.0f) {
      motionActive_ = true;
    }
  }
}

void MotionController::computeExpectedField(float out[9]) const {
  estimator_.computeExpectedField(estimator_.state(), out);
}

void MotionController::estimatorPose(float out[6]) const {
  const EKFEngine::PoseState& s = estimator_.state();
  out[0] = s.tx;
  out[1] = s.ty;
  out[2] = s.tz;
  out[3] = s.rx;
  out[4] = s.ry;
  out[5] = s.rz;
}

float MotionController::estimatorCovarianceTrace() const {
  return estimator_.covarianceTrace();
}

bool MotionController::hasMotionActivity() const { return motionActive_; }

const EKFEngine& MotionController::estimator() const { return estimator_; }
