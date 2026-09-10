#pragma once

class PID {
public:
    PID();
    PID(float kp, float ki, float kd, float dt_s,
        float out_min, float out_max, float integral_limit);

    float Update(float setpoint, float measurement);
    void  Reset();
    void  SetGains(float kp, float ki, float kd);
    void  SetLimits(float out_min, float out_max);

    float GetLastError()  const;
    float GetLastOutput() const;

private:
    float _kp, _ki, _kd, _dt;
    float _out_min, _out_max, _integral_limit;
    float _integral, _prev_meas;
    float _last_error, _last_output;
    float _d_filt;        // low-passed derivative (~14 Hz) — smooths sensor-rate staircase
    bool  _first_update;  // suppress derivative on first tick after Reset (no valid prev_meas yet)
};
