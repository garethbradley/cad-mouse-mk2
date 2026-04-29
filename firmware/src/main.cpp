#include <Arduino.h>

#include "Config.h"
#include "Controllers.h"
#include "StateMachine.h"

InputController inputController;
LEDController ledController;
SensorController sensorController;
MotionController motionController;
MotionWorkerController motionWorkerController;
HIDController hidController;
TelemetryController telemetryController;

void setup() {
  // Initialize USB HID first
  hidController.begin();

  if (Config::TELEMETRY_LOG_LEVEL != Config::TELEMETRY_LOG_OFF) {
    Serial.begin(115200);
    delay(200);
  }

  inputController.begin();
  ledController.begin();
  sensorController.begin();
  motionController.reset();
  telemetryController.begin();

  stateMachine.changeState(&StateMachine::calibratingState);
}

void setup1() {
  if (Config::ENABLE_DUAL_CORE_MOTION) {
    motionWorkerController.begin();
  }
}

void loop() {
  hidController.task();
  stateMachine.update();
}

void loop1() {
  if (Config::ENABLE_DUAL_CORE_MOTION) {
    motionWorkerController.loop();
  }
}
