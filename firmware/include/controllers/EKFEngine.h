#pragma once

#include "Config.h"

class EKFEngine {
 public:
  struct PoseState {
    float tx;
    float ty;
    float tz;
    float rx;
    float ry;
    float rz;
  };

  void reset();
  void predict(float dt);
  void update(const float measuredField[9]);
  void computeExpectedField(const PoseState& state, float out[9]) const;
  const PoseState& state() const;
  float covarianceTrace() const;
  bool geometryIsFullySpecified() const;

 private:
  static Config::Vec3 add(const Config::Vec3& a, const Config::Vec3& b);
  static Config::Vec3 subtract(const Config::Vec3& a, const Config::Vec3& b);
  static Config::Vec3 scale(const Config::Vec3& v, float scalar);
  static float dot(const Config::Vec3& a, const Config::Vec3& b);
  static float magnitude(const Config::Vec3& v);
  static Config::Vec3 rotateSmallAngle(const Config::Vec3& v,
                                       const PoseState& state);
  static Config::Vec3 dipoleFieldAtPoint(const Config::Vec3& samplePoint,
                                         const Config::Vec3& dipolePosition,
                                         const Config::Vec3& dipoleMoment);
  static bool invert9x9(const float input[9][9], float out[9][9]);

  PoseState state_{};
  float covariance_[6][6] = {};
};