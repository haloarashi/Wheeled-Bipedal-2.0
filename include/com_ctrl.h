#pragma once

// com_ctrl -- vertical CoM regulator (Grand Plan Task 3, Stage 2: IDLE only).
// Reads IMU_GetVerticalAccel() and computes a small common-mode height trim
// meant to absorb vertical disturbances (the ground pushing the body up or
// down), fed into HeightCtrl_SetVerticalTrim() by the caller every tick.
//
// This is feedback-only damping, not a trajectory tracker: with KA=0 (the
// shipped starting gain) it does nothing in response to a commanded height
// change, only resists acceleration-driven disturbance. A pure
// accel->position law (KA alone, with no KV) would be an undamped
// oscillator and ring instead of settling -- KV (velocity/skyhook damping)
// is the term that actually stabilizes it, not optional polish. See the
// Grand Plan (Task 3) for the full derivation.
//
// Caller is responsible for gating (IDLE-only for now) and for actually
// wiring the trim into HeightCtrl_SetVerticalTrim() every tick -- this
// module never touches HeightCtrl or the motor bus directly.

void  ComCtrl_Init();
void  ComCtrl_SetEnabled(bool on);   // false -> resets internal state AND trim -> 0
bool  ComCtrl_IsEnabled();
// hold=true: freeze the integrator INPUT (balance loop is busy -- G4 attempt
// 3 showed the rocking's own vertical accel, +-3..7 m/s^2 at the IMU, rails
// the trim and pumps the balance mode resonantly). v_z keeps washing out and
// the trim keeps slewing/centering, so an impulse caught BEFORE the gate
// closed (a real bump loads v_z in its first ~100 ms, before tilt reacts)
// still gets acted on; new input is ignored until balance quiets down.
void  ComCtrl_Tick(float dt_s, bool hold);   // call at 200 Hz from control_tick
float ComCtrl_GetTrimCm();           // current commanded trim (already slewed)
