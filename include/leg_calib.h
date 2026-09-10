#pragma once
// leg_calib.h — model<->motor angle mapping (Milestone 0, calibrated 2026-07-06)
//
// Bench calibration: hand-posed legs, ruler-measured heights, least-squares fit
// via programming/kinematics/calibrate_legs.py. Fit quality:
//   LEFT  RMS 0.14 cm (4 poses)   RIGHT RMS 0.28 cm (4 poses)
//
// Two facts the raw motor angles hide:
//   1. COAXIAL COUPLING — the knee motor rides on the hip arm, so its encoder
//      reads relative to the hip:  theta2_abs = theta1_abs + knee_relative.
//   2. MIRROR — the right leg mirrors the left. Raw right-side angles must be
//      negated before the (left-leg) kinematic model applies, and negated
//      again on the way back out.
//
// "Model" angles below = absolute left-model convention of leg_fk.cpp /
// leg_kinematics.py:  theta1=0 -> hip arm +X (forward), theta2=0 -> upper arm
// -Z (straight down), CCW positive viewed from robot's left.

enum LegSide { LEG_LEFT = 0, LEG_RIGHT = 1 };

// Fitted zero offsets (deg): raw_in_model_space = model + offset.
// Right-side offsets live in mirrored (negated) space — handled below.
static constexpr float CALIB_HIP_OFFSET[2]  = { +0.68f, +9.20f };   // L, R
// Left knee offset = 0 since 2026-07-07: user re-zeroed ID2's motor zero,
// invalidating the fitted -4.88 (old-zero frame). RULER-VERIFIED 2026-07-07:
// at the IK(15.53) boot stance both legs measure ~15.5 cm (old offset would
// have put the left at ~16.2). Jitter also vanished at all heights.
static constexpr float CALIB_KNEE_OFFSET[2] = {  0.00f, -6.74f };   // L, R

// Raw-command safety clamps, from IK at the workspace edges (8.5 / 18.0 cm)
// plus 3 deg margin. Applied to every command produced by Calib_RawFromModel.
static constexpr float CALIB_HIP_MIN[2]  = { -62.0f,  -6.4f };
static constexpr float CALIB_HIP_MAX[2]  = {  -2.1f, +53.5f };
// Left knee clamps shifted +4.88 on 2026-07-07 (frame translation: new zero
// offset 0 vs fitted -4.88, same physical workspace).
static constexpr float CALIB_KNEE_MIN[2] = { -75.9f,  -4.0f };
static constexpr float CALIB_KNEE_MAX[2] = { +10.7f, +82.7f };

static inline float calib_clampf(float v, float lo, float hi) {
    return v < lo ? lo : (v > hi ? hi : v);
}

// Model angles (absolute, left convention, deg) -> RAW motor commands (deg).
// Returns false if the command had to be clamped (caller should treat that as
// "requested pose is outside the calibrated workspace").
static inline bool Calib_RawFromModel(LegSide side,
                                      float theta1_deg, float theta2_deg,
                                      float &hip_raw, float &knee_raw) {
    float m1 = theta1_deg + CALIB_HIP_OFFSET[side];
    float m2 = (theta2_deg - theta1_deg) + CALIB_KNEE_OFFSET[side]; // invert coupling
    if (side == LEG_RIGHT) { m1 = -m1; m2 = -m2; }                  // un-mirror

    hip_raw  = calib_clampf(m1, CALIB_HIP_MIN[side],  CALIB_HIP_MAX[side]);
    knee_raw = calib_clampf(m2, CALIB_KNEE_MIN[side], CALIB_KNEE_MAX[side]);
    return (hip_raw == m1) && (knee_raw == m2);
}

// RAW encoder readings (deg) -> model angles (absolute, left convention, deg).
static inline void Calib_ModelFromRaw(LegSide side,
                                      float hip_raw, float knee_raw,
                                      float &theta1_deg, float &theta2_deg) {
    float m1 = hip_raw, m2 = knee_raw;
    if (side == LEG_RIGHT) { m1 = -m1; m2 = -m2; }                  // mirror in
    theta1_deg = m1 - CALIB_HIP_OFFSET[side];
    theta2_deg = theta1_deg + (m2 - CALIB_KNEE_OFFSET[side]);       // apply coupling
}
