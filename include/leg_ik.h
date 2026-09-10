#pragma once

// leg_ik.h — inverse kinematics: wheel target -> absolute MODEL angles (deg).
// Port of leg_kinematics.py ik() (left-leg model convention). Convert output
// to raw per-side motor commands with Calib_RawFromModel() (leg_calib.h).

// Target height (cm, positive down) with wheel directly below hip axis (x=0).
// Returns false if unreachable.
bool LegIK_FromHeightCm(float h_cm, float &theta1_deg, float &theta2_deg);

// General form: x_cm forward offset (+fwd), h_cm height.
bool LegIK_FromXHCm(float x_cm, float h_cm, float &theta1_deg, float &theta2_deg);

// Stance fore-aft wheel offset used by ALL pose generation (stance + height
// control). 2026-07-07: x=0 (wheel under hip axis) proved worse than the old
// eyeballed stance — it moved the balance equilibrium 3.4->4.9 deg and parked
// the pose in a hip-resonance pocket ('8' vs 'h' A/B: old pose quiet, x=0 pose
// jittered). x=-0.616 is the old stance's exact offset (FK of raw
// -42.04/-20.28 in the verified frame); reachable across 8.5-18 cm.
static constexpr float LEG_STANCE_X_CM = -0.616f;
