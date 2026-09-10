#include "pid.h"
#include <math.h>

static float clamp(float v, float lo, float hi) {
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

PID::PID()
    : _kp(0), _ki(0), _kd(0), _dt(0.005f),
      _out_min(-1000), _out_max(1000), _integral_limit(1000),
      _integral(0), _prev_meas(0), _last_error(0), _last_output(0),
      _d_filt(0), _first_update(true) {}

PID::PID(float kp, float ki, float kd, float dt_s,
         float out_min, float out_max, float integral_limit)
    : _kp(kp), _ki(ki), _kd(kd), _dt(dt_s),
      _out_min(out_min), _out_max(out_max), _integral_limit(integral_limit),
      _integral(0), _prev_meas(0), _last_error(0), _last_output(0),
      _d_filt(0), _first_update(true) {}

float PID::Update(float setpoint, float measurement) {
    float error      = setpoint - measurement;
    // Conditional anti-windup: while the output sits at a clamp, integrating further
    // in the same direction only stores stale correction that gets released late and
    // pumps slow oscillations (observed 2026-07-03: tilt_sp swung clamp-to-clamp with
    // a ~3.5 s period, growing to +/-15 deg excursions). Freeze the integral in the
    // winding direction while saturated; unwinding is always allowed.
    bool wind_hi = (_last_output >= _out_max) && (error > 0.0f);
    bool wind_lo = (_last_output <= _out_min) && (error < 0.0f);
    if (!wind_hi && !wind_lo)
        _integral = clamp(_integral + error * _dt, -_integral_limit, _integral_limit);
    // D on measurement (avoids setpoint kick). On the first tick after Reset there
    // is no valid prev_meas — using 0 injected a one-tick spike of Kd*meas/dt
    // (measured -204 dps at tilt 3.4 deg on bench 2026-07-03), so skip D once.
    float derivative;
    if (_first_update) {
        derivative    = 0.0f;
        _first_update = false;
    } else {
        derivative = -(measurement - _prev_meas) / _dt;
    }
    // Low-pass the D term (alpha 0.3 @ 200 Hz -> ~14 Hz cutoff). The IMU delivers a
    // new sample only every 2nd tick, so raw D is a 100 Hz staircase: (delta/dt) one
    // tick, 0 the next — at Kd=3 and 60 deg/s fall rates that slammed wheel_cmd by
    // +/-200 dps every 5 ms (observed 2026-07-03). 14 Hz passes real pendulum
    // dynamics (1-3 Hz) untouched and crushes the 100 Hz artifact.
    _d_filt += 0.3f * (derivative - _d_filt);
    float output     = clamp(_kp * error + _ki * _integral + _kd * _d_filt,
                             _out_min, _out_max);
    _prev_meas   = measurement;
    _last_error  = error;
    _last_output = output;
    return output;
}

void PID::Reset() {
    _integral  = 0;
    _prev_meas = 0;
    _last_error = 0;
    _last_output = 0;
    _d_filt = 0;
    _first_update = true;
}

void PID::SetGains(float kp, float ki, float kd) {
    _kp = kp; _ki = ki; _kd = kd;
}

void PID::SetLimits(float out_min, float out_max) {
    _out_min = out_min; _out_max = out_max;
}

float PID::GetLastError()  const { return _last_error;  }
float PID::GetLastOutput() const { return _last_output; }
