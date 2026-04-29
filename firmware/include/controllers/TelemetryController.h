#pragma once

struct MotionWorkerTiming;

class TelemetryController {
 public:
  void begin();
  void publish(const float motion[6], const float measuredField[9],
               const float predictedField[9], const float estimatedPose[6],
               float residualRms, float covarianceTrace, int buttonBits,
               bool hidReportSent, const MotionWorkerTiming* timing = nullptr);
  bool enabled() const;

 private:
  int tick_ = 0;
};
