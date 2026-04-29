#include "controllers/EKFEngine.h"

#include <math.h>

namespace {
constexpr float kMinimumRadiusMm = 0.1f;
constexpr float kJacobianStep = 0.001f;

float readStateComponent(const EKFEngine::PoseState& state, int index) {
  switch (index) {
    case 0:
      return state.tx;
    case 1:
      return state.ty;
    case 2:
      return state.tz;
    case 3:
      return state.rx;
    case 4:
      return state.ry;
    default:
      return state.rz;
  }
}

void writeStateComponent(EKFEngine::PoseState& state, int index, float value) {
  switch (index) {
    case 0:
      state.tx = value;
      break;
    case 1:
      state.ty = value;
      break;
    case 2:
      state.tz = value;
      break;
    case 3:
      state.rx = value;
      break;
    case 4:
      state.ry = value;
      break;
    default:
      state.rz = value;
      break;
  }
}
}

void EKFEngine::reset() {
  state_ = {};
  for (int row = 0; row < 6; ++row) {
    for (int col = 0; col < 6; ++col) {
      covariance_[row][col] = (row == col) ? 1.0f : 0.0f;
    }
  }
}

void EKFEngine::predict(float dt) {
  (void)dt;
  for (int axis = 0; axis < 6; ++axis) {
    covariance_[axis][axis] += Config::EKF_PROCESS_NOISE[axis];
  }
}

void EKFEngine::update(const float measuredField[9]) {
  float predicted[9] = {};
  computeExpectedField(state_, predicted);

  float H[9][6] = {};
  for (int axis = 0; axis < 6; ++axis) {
    PoseState plus = state_;
    const float center = readStateComponent(state_, axis);
    writeStateComponent(plus, axis, center + kJacobianStep);

    float hPlus[9] = {};
    computeExpectedField(plus, hPlus);

    if (Config::EKF_USE_FORWARD_DIFFERENCE) {
      const float invStep = 1.0f / kJacobianStep;
      for (int measurement = 0; measurement < 9; ++measurement) {
        H[measurement][axis] =
            (hPlus[measurement] - predicted[measurement]) * invStep;
      }
    } else {
      PoseState minus = state_;
      writeStateComponent(minus, axis, center - kJacobianStep);

      float hMinus[9] = {};
      computeExpectedField(minus, hMinus);

      const float invTwoStep = 1.0f / (2.0f * kJacobianStep);
      for (int measurement = 0; measurement < 9; ++measurement) {
        H[measurement][axis] =
            (hPlus[measurement] - hMinus[measurement]) * invTwoStep;
      }
    }
  }

  float innovation[9] = {};
  for (int measurement = 0; measurement < 9; ++measurement) {
    innovation[measurement] = measuredField[measurement] - predicted[measurement];
  }

  float HP[9][6] = {};
  for (int measurement = 0; measurement < 9; ++measurement) {
    for (int col = 0; col < 6; ++col) {
      float sum = 0.0f;
      for (int k = 0; k < 6; ++k) {
        sum += H[measurement][k] * covariance_[k][col];
      }
      HP[measurement][col] = sum;
    }
  }

  float S[9][9] = {};
  for (int row = 0; row < 9; ++row) {
    for (int col = 0; col < 9; ++col) {
      float sum = 0.0f;
      for (int k = 0; k < 6; ++k) {
        sum += HP[row][k] * H[col][k];
      }
      S[row][col] = sum;
    }
    S[row][row] += Config::EKF_MEASUREMENT_NOISE[row];
  }

  float SInv[9][9] = {};
  if (!invert9x9(S, SInv)) {
    return;
  }

  float PHt[6][9] = {};
  for (int row = 0; row < 6; ++row) {
    for (int col = 0; col < 9; ++col) {
      float sum = 0.0f;
      for (int k = 0; k < 6; ++k) {
        sum += covariance_[row][k] * H[col][k];
      }
      PHt[row][col] = sum;
    }
  }

  float K[6][9] = {};
  for (int row = 0; row < 6; ++row) {
    for (int col = 0; col < 9; ++col) {
      float sum = 0.0f;
      for (int k = 0; k < 9; ++k) {
        sum += PHt[row][k] * SInv[k][col];
      }
      K[row][col] = sum;
    }
  }

  float delta[6] = {};
  for (int row = 0; row < 6; ++row) {
    float sum = 0.0f;
    for (int k = 0; k < 9; ++k) {
      sum += K[row][k] * innovation[k];
    }
    delta[row] = sum;
  }

  state_.tx += delta[0];
  state_.ty += delta[1];
  state_.tz += delta[2];
  state_.rx += delta[3];
  state_.ry += delta[4];
  state_.rz += delta[5];

  float KH[6][6] = {};
  for (int row = 0; row < 6; ++row) {
    for (int col = 0; col < 6; ++col) {
      float sum = 0.0f;
      for (int k = 0; k < 9; ++k) {
        sum += K[row][k] * H[k][col];
      }
      KH[row][col] = sum;
    }
  }

  float IminusKH[6][6] = {};
  for (int row = 0; row < 6; ++row) {
    for (int col = 0; col < 6; ++col) {
      IminusKH[row][col] = (row == col ? 1.0f : 0.0f) - KH[row][col];
    }
  }

  float newP[6][6] = {};
  for (int row = 0; row < 6; ++row) {
    for (int col = 0; col < 6; ++col) {
      float sum = 0.0f;
      for (int k = 0; k < 6; ++k) {
        sum += IminusKH[row][k] * covariance_[k][col];
      }
      newP[row][col] = sum;
    }
  }

  for (int row = 0; row < 6; ++row) {
    for (int col = 0; col < 6; ++col) {
      covariance_[row][col] = newP[row][col];
    }
    if (covariance_[row][row] < 1e-6f) {
      covariance_[row][row] = 1e-6f;
    }
  }
}

void EKFEngine::computeExpectedField(const PoseState& state, float out[9]) const {
  const Config::Vec3 translation = {state.tx, state.ty, state.tz};
  Config::Vec3 magnetPositions[3] = {};
  Config::Vec3 magnetMoments[3] = {};

  for (int magnetIndex = 0; magnetIndex < 3; ++magnetIndex) {
    const Config::Vec3 neutralMagnet =
        Config::MAGNET_POSITIONS_NEUTRAL[magnetIndex];
    const Config::Vec3 neutralMoment = Config::MAGNET_MOMENTS_NEUTRAL[magnetIndex];
    magnetPositions[magnetIndex] =
        add(rotateSmallAngle(neutralMagnet, state), translation);
    magnetMoments[magnetIndex] = rotateSmallAngle(neutralMoment, state);
  }

  for (int sensorIndex = 0; sensorIndex < 3; ++sensorIndex) {
    const Config::Vec3 sensorPosition = Config::SENSOR_POSITIONS[sensorIndex];
    Config::Vec3 totalField = {0.0f, 0.0f, 0.0f};

    for (int magnetIndex = 0; magnetIndex < 3; ++magnetIndex) {
      totalField = add(totalField,
                       dipoleFieldAtPoint(sensorPosition,
                                          magnetPositions[magnetIndex],
                                          magnetMoments[magnetIndex]));
    }

    out[sensorIndex * 3 + 0] = totalField.x;
    out[sensorIndex * 3 + 1] = totalField.y;
    out[sensorIndex * 3 + 2] = totalField.z;
  }
}

const EKFEngine::PoseState& EKFEngine::state() const { return state_; }

float EKFEngine::covarianceTrace() const {
  float sum = 0.0f;
  for (int i = 0; i < 6; ++i) {
    sum += covariance_[i][i];
  }
  return sum;
}

bool EKFEngine::geometryIsFullySpecified() const {
  return Config::MAGNET_PITCH_CIRCLE_DIAMETER_MM > 0.0f;
}

Config::Vec3 EKFEngine::add(const Config::Vec3& a, const Config::Vec3& b) {
  return {a.x + b.x, a.y + b.y, a.z + b.z};
}

Config::Vec3 EKFEngine::subtract(const Config::Vec3& a, const Config::Vec3& b) {
  return {a.x - b.x, a.y - b.y, a.z - b.z};
}

Config::Vec3 EKFEngine::scale(const Config::Vec3& v, float scalar) {
  return {v.x * scalar, v.y * scalar, v.z * scalar};
}

float EKFEngine::dot(const Config::Vec3& a, const Config::Vec3& b) {
  return (a.x * b.x) + (a.y * b.y) + (a.z * b.z);
}

float EKFEngine::magnitude(const Config::Vec3& v) {
  return sqrtf(dot(v, v));
}

Config::Vec3 EKFEngine::rotateSmallAngle(const Config::Vec3& v,
                                         const PoseState& state) {
  const Config::Vec3 omega = {state.rx, state.ry, state.rz};
  const Config::Vec3 cross = {
      (omega.y * v.z) - (omega.z * v.y),
      (omega.z * v.x) - (omega.x * v.z),
      (omega.x * v.y) - (omega.y * v.x),
  };
  return add(v, cross);
}

Config::Vec3 EKFEngine::dipoleFieldAtPoint(const Config::Vec3& samplePoint,
                                           const Config::Vec3& dipolePosition,
                                           const Config::Vec3& dipoleMoment) {
  Config::Vec3 r = subtract(samplePoint, dipolePosition);
  float radius = magnitude(r);
  if (radius < kMinimumRadiusMm) {
    radius = kMinimumRadiusMm;
  }

  const float inverseRadius = 1.0f / radius;
  const Config::Vec3 rHat = scale(r, inverseRadius);
  const float mdotr = dot(dipoleMoment, rHat);
  const Config::Vec3 numerator = subtract(scale(rHat, 3.0f * mdotr), dipoleMoment);
  const float inverseRadiusCubed = inverseRadius * inverseRadius * inverseRadius;
  return scale(numerator, inverseRadiusCubed);
}

bool EKFEngine::invert9x9(const float input[9][9], float out[9][9]) {
  float augmented[9][18] = {};

  for (int row = 0; row < 9; ++row) {
    for (int col = 0; col < 9; ++col) {
      augmented[row][col] = input[row][col];
      augmented[row][col + 9] = (row == col) ? 1.0f : 0.0f;
    }
  }

  for (int pivot = 0; pivot < 9; ++pivot) {
    int maxRow = pivot;
    float maxAbs = fabsf(augmented[pivot][pivot]);
    for (int row = pivot + 1; row < 9; ++row) {
      const float candidate = fabsf(augmented[row][pivot]);
      if (candidate > maxAbs) {
        maxAbs = candidate;
        maxRow = row;
      }
    }

    if (maxAbs < 1e-9f) {
      return false;
    }

    if (maxRow != pivot) {
      for (int col = 0; col < 18; ++col) {
        const float tmp = augmented[pivot][col];
        augmented[pivot][col] = augmented[maxRow][col];
        augmented[maxRow][col] = tmp;
      }
    }

    const float pivotVal = augmented[pivot][pivot];
    for (int col = 0; col < 18; ++col) {
      augmented[pivot][col] /= pivotVal;
    }

    for (int row = 0; row < 9; ++row) {
      if (row == pivot) {
        continue;
      }
      const float factor = augmented[row][pivot];
      if (factor == 0.0f) {
        continue;
      }
      for (int col = 0; col < 18; ++col) {
        augmented[row][col] -= factor * augmented[pivot][col];
      }
    }
  }

  for (int row = 0; row < 9; ++row) {
    for (int col = 0; col < 9; ++col) {
      out[row][col] = augmented[row][col + 9];
    }
  }

  return true;
}