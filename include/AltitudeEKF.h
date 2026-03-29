#pragma once

#include <math.h>

namespace AltitudeMath {

// Convert pressure [Pa] to altitude [m] by barometric formula.
inline float pressureToAltitudeMeters(float pressurePa, float seaLevelHpa) {
  if (pressurePa <= 0.0f || seaLevelHpa <= 0.0f) {
    return 0.0f;
  }
  const float pressureHpa = pressurePa * 0.01f;
  return 44330.0f * (1.0f - powf(pressureHpa / seaLevelHpa, 0.1903f));
}

}  // namespace AltitudeMath

// 1D altitude EKF with 2 states: [altitude, vertical_velocity]
class AltitudeEKF {
public:
  void setProcessAccelSigma(float sigma) { processAccelSigma_ = sigma; }

  void setInitialCovariance(float altitudeVar, float velocityVar) {
    initP00_ = altitudeVar;
    initP11_ = velocityVar;
  }

  void init(float altitude0, float velocity0 = 0.0f) {
    x0_ = altitude0;
    x1_ = velocity0;
    p00_ = initP00_;
    p01_ = 0.0f;
    p10_ = 0.0f;
    p11_ = initP11_;
    initialized_ = true;
  }

  bool isInitialized() const { return initialized_; }

  // Prediction with gravity-compensated acceleration [m/s^2] and dt [s].
  void predict(float a, float dt) {
    if (!initialized_ || dt <= 0.0f) {
      return;
    }

    const float dt2 = dt * dt;
    const float dt3 = dt2 * dt;
    const float dt4 = dt3 * dt;

    x0_ = x0_ + dt * x1_ + 0.5f * dt2 * a;
    x1_ = x1_ + dt * a;

    const float qa = processAccelSigma_ * processAccelSigma_;
    const float fp00 = p00_ + dt * p10_;
    const float fp01 = p01_ + dt * p11_;
    const float fp10 = p10_;
    const float fp11 = p11_;

    p00_ = fp00 + dt * fp01 + 0.25f * dt4 * qa;
    p01_ = fp01 + 0.5f * dt3 * qa;
    p10_ = fp10 + dt * fp11 + 0.5f * dt3 * qa;
    p11_ = fp11 + dt2 * qa;
  }

  // Measurement update with altitude observation z [m], variance r [m^2].
  void update(float z, float r) {
    if (!initialized_ || r <= 0.0f) {
      return;
    }

    const float s = p00_ + r;
    if (s <= 0.0f) {
      return;
    }

    const float k0 = p00_ / s;
    const float k1 = p10_ / s;
    const float innov = z - x0_;

    x0_ += k0 * innov;
    x1_ += k1 * innov;

    const float prevP00 = p00_;
    const float prevP01 = p01_;
    const float prevP10 = p10_;
    const float prevP11 = p11_;

    p00_ = (1.0f - k0) * prevP00;
    p01_ = (1.0f - k0) * prevP01;
    p10_ = prevP10 - k1 * prevP00;
    p11_ = prevP11 - k1 * prevP01;
  }

  void alignAltitude(float altitude, float altitudeVar) {
    if (!initialized_) {
      init(altitude);
      p00_ = altitudeVar;
      return;
    }
    x0_ = altitude;
    if (altitudeVar > 0.0f) {
      p00_ = altitudeVar;
    }
  }

  float altitude() const { return x0_; }
  float velocity() const { return x1_; }

private:
  float processAccelSigma_ = 0.3f;
  float initP00_ = 100.0f;
  float initP11_ = 10.0f;

  float x0_ = 0.0f;
  float x1_ = 0.0f;
  float p00_ = 100.0f;
  float p01_ = 0.0f;
  float p10_ = 0.0f;
  float p11_ = 10.0f;
  bool initialized_ = false;
};
