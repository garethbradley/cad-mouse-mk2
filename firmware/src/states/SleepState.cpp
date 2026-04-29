#include "states/SleepState.h"

#include <Arduino.h>

#include "Config.h"
#include "Controllers.h"
#include "StateMachine.h"

void SleepState::enter() {
  if (Config::ENABLE_DUAL_CORE_MOTION) {
    motionWorkerController.stopAndWait();
  }

  ledController.off();
}

void SleepState::update() {
  inputController.update();

  if (inputController.takeActivity()) {
    stateMachine.changeState(&StateMachine::idleState);
    return;
  }
}

void SleepState::exit() {}
