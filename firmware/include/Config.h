#pragma once

#include <Arduino.h>

namespace Config {

struct Vec3 {
	float x;
	float y;
	float z;
};

enum TelemetryLogLevel {
	TELEMETRY_LOG_OFF = 0,
	TELEMETRY_LOG_BASIC = 1,
	TELEMETRY_LOG_FULL = 2,
};

// Telemetry verbosity over USB serial:
// OFF = no logs, BASIC = motion/estimator/timing, FULL = BASIC + field model vectors.
const TelemetryLogLevel TELEMETRY_LOG_LEVEL = TELEMETRY_LOG_OFF;
// When true, the motion processing pipeline runs on Core 1, while Core 0 keeps
// HID/state-machine duties.
const bool ENABLE_DUAL_CORE_MOTION = true;
// When true, HID motion output uses EKF pose state instead of the legacy
// heuristic decoder.
const bool USE_EKF_FOR_HID_OUTPUT = true;
// When true, firmware bypasses physical TLx493D reads so timing telemetry can
// be profiled on a bare XIAO RP2040. Set false for real hardware operation.
const bool ENABLE_SENSORLESS_PROFILING = false;

// Hardware pins (XIAO RP2040)
const int PIN_RIGHT_BTN = D0;
const int PIN_LEFT_BTN = D2;
const int PIN_LED_DATA = D3;
const int PIN_LED_LS = D1;
const int PIN_MAG1_LS = D10;
const int PIN_MAG2_LS = D9;
const int PIN_MAG3_LS = D8;

// Samples for calibration offset
const int ZERO_SAMPLES = 200;

// Mechanical geometry for the physics-model motion engine.
const float MAGNET_HEIGHT_MM = 6.0f;
const float SENSOR_TOP_TO_MAGNET_BOTTOM_MM = 5.412f;
const float SENSOR_SENSITIVE_DEPTH_FROM_TOP_MM = 0.65f;
const float MAGNET_PITCH_CIRCLE_DIAMETER_MM = 33.0f;
const float TOP_SENSOR_X_MM = 14.289419f;
const float TOP_SENSOR_Y_MM = 8.25f;
const float BOTTOM_SENSOR_Y_MM = -16.5f;
const float SENSOR_TO_MAGNET_CENTER_Z_MM =
	SENSOR_TOP_TO_MAGNET_BOTTOM_MM + SENSOR_SENSITIVE_DEPTH_FROM_TOP_MM -
	(MAGNET_HEIGHT_MM * 0.5f);

const Vec3 SENSOR_POSITIONS[3] = {
	{0.0f, BOTTOM_SENSOR_Y_MM, 0.0f},
	{-TOP_SENSOR_X_MM, TOP_SENSOR_Y_MM, 0.0f},
	{TOP_SENSOR_X_MM, TOP_SENSOR_Y_MM, 0.0f},
};

const Vec3 MAGNET_POSITIONS_NEUTRAL[3] = {
	{0.0f, BOTTOM_SENSOR_Y_MM, SENSOR_TO_MAGNET_CENTER_Z_MM},
	{-TOP_SENSOR_X_MM, TOP_SENSOR_Y_MM, SENSOR_TO_MAGNET_CENTER_Z_MM},
	{TOP_SENSOR_X_MM, TOP_SENSOR_Y_MM, SENSOR_TO_MAGNET_CENTER_Z_MM},
};

const Vec3 MAGNET_MOMENTS_NEUTRAL[3] = {
	{0.0f, 0.0f, 1.0f},
	{0.0f, 0.0f, 1.0f},
	{0.0f, 0.0f, 1.0f},
};

// Gains and sign fixes
const float GAIN_T[3] = {28.0, 28.0, 24.0};
const float GAIN_R[3] = {18.0, 18.0, 20.0};
const int SIGN_AXIS[6] = {-1, +1, -1, +1, +1, +1};

// Dead zones
const float DEAD_T = 16.0;
const float DEAD_R = 20.0;

// Kalman filter tuning
// Process noise: how much we expect the true value to change per step.
// Higher = more responsive but noisier.
const float KALMAN_Q = 0.5;
// Measurement noise: how noisy the sensor readings are.
// Higher = smoother but more latency.
const float KALMAN_R = 4.0;

// Sensitivity curve exponent.
// 1.0 = linear, 3.0 = cubic (fine control at small deflections, fast at large).
const float SENSITIVITY_EXP = 3.0;

// Final axis output range
const float AXIS_LIMIT = 350.0;

// EKF tuning placeholders. These are not used until the physics-model path is
// wired into MotionController.
const float EKF_PROCESS_NOISE[6] = {0.5f, 0.5f, 0.5f, 0.25f, 0.25f, 0.25f};
const float EKF_MEASUREMENT_NOISE[9] = {4.0f, 4.0f, 4.0f, 4.0f, 4.0f,
										4.0f, 4.0f, 4.0f, 4.0f};
// Faster Jacobian mode: one perturbation per state axis (forward difference)
// instead of central difference (+/- perturbation).
const bool EKF_USE_FORWARD_DIFFERENCE = true;
// Optional EKF correction decimation. 1 = update every frame, 2 = every other
// frame, etc. Predict still runs every frame.
const int EKF_UPDATE_DECIMATION = 2;

// RGB LEDs
const int LED_COUNT = 8;
const int LED_BRIGHTNESS = 40;
const unsigned long LED_IDLE_COLOR = 0x00FF00;
const unsigned long LED_CALIBRATING_COLOR = 0x0000FF;

// FSM timing
const long IDLE_SLEEP_TIMEOUT_MS = 2 * 60 * 1000;

}  // namespace Config
