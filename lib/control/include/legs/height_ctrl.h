#pragma once

// height_ctrl — body-height ramp manager (DIABLO-style height-as-setpoint).
// Owns a target height and a rate-limited "current" height; while ramping it
// runs IK -> calibration -> MotorBus_SetLegAngle for both legs, then goes
// silent on arrival (LK motors hold position internally).
//
// Stage 1: caller gates ticks to IDLE (on a stand). Stage 2 (during BALANCE)
// is enabled by the caller behind a flag.

static constexpr float HEIGHT_MIN_CM  = 8.5f;
static constexpr float HEIGHT_MAX_CM  = 18.0f;
static constexpr float HEIGHT_STEP_CM = 0.5f;   // per 'u'/'d' press
static constexpr float HEIGHT_CROUCH_CM = 14.0f;   // 'c' + split low leg. 14.0->11.0->13.0->13.5->14.0
                                                   // (reverted 2026-07-09 to the 07-07 value: low
                                                   // heights ring slightly; 14.0 is perfectly solid
                                                   // for film shooting. LUT[14.0]=3.36 is captured.)
static constexpr float HEIGHT_TALL_CM   = 16.5f;   // 'g'
static constexpr float HEIGHT_EXTRA_CM  = 18.0f;   // 'l'  (17.0 -> 18.0 = HEIGHT_MAX, 2026-07-09)
// nominal = stance height captured at Init()  ('o')

// Differential leg height (DIABLO-style side lean): diff = h_right - h_left.
// Each leg runs IK at h -/+ diff/2 (left/right), same x-offset. Slew-limited.
static constexpr float HEIGHT_DIFF_STEP_CM = 0.25f;  // per ','/'.' press
static constexpr float HEIGHT_DIFF_MAX_CM  = 7.0f;   // 3.0 -> 7.0 (2026-07-09) so 9/0 split reaches true c/l
                                                     // (11/18): diff = l-c = 7.0, +-3.5 cm/leg. UNTESTED beyond
                                                     // +-2.0 -- manual lean ','/'.' can now reach this too.

// Experimental: the five-bar linkage's own IK geometry stops being solvable
// past ~20.0 cm at LEG_STANCE_X_CM (verified numerically 2026-07-09 — NOT
// 21.0 as visually estimated). Only HeightCtrl_SetTargetExperimental() can
// reach past HEIGHT_MAX_CM; every other setter (presets, u/d, diff/split)
// resets the ceiling back to HEIGHT_MAX_CM first, so a prior experimental
// call can never leak into normal operation.
static constexpr float HEIGHT_EXPERIMENTAL_MAX_CM = 20.0f;

// Vertical CoM trim hard backstop (com_ctrl's own TRIM_MAX is tighter at
// ±1.0 cm -- this is defense-in-depth at the actuation boundary, not the
// primary limit).
static constexpr float VTRIM_MAX_CM = 2.0f;

void  HeightCtrl_Init(float h0_cm);   // current = target = nominal = h0, diff = 0, vtrim = 0
bool  HeightCtrl_SetTarget(float h_cm); // clamped; false if request was clamped
bool  HeightCtrl_SetTargetExperimental(float h_cm); // clamped to HEIGHT_EXPERIMENTAL_MAX_CM, not HEIGHT_MAX_CM
bool  HeightCtrl_DiffStep(float d_cm);  // adjust diff target; false if clamped
bool  HeightCtrl_SetDiff(float d_cm);   // set diff target directly; false if clamped
void  HeightCtrl_DiffLevel();           // diff target -> 0

// Vertical CoM trim (common-mode, cm): added to BOTH legs' height, on top of
// the base ramp + diff. Not slewed here -- the caller (com_ctrl) already
// slews it; this setter just applies a hard ±2.0 cm backstop regardless of
// what the caller sends, and is NOT itself rate-limited or gated to a stage
// -- gating (IDLE-only for now) is the caller's job. No target/current split
// like height -- unlike a human-issued setpoint, this changes every tick
// from a live control loop, so there's nothing to "ramp toward": it IS the
// instantaneous commanded value already.
bool  HeightCtrl_SetVerticalTrim(float cm);  // clamped ±2.0; false if request was clamped
float HeightCtrl_GetVerticalTrimCm();
float HeightCtrl_GetDiffCm();           // current (ramped) diff
// Last-commanded raw hip angle (degrees, motor frame) -- diagnostic (2026-07-15),
// for comparing against a real encoder read (MotorBus_GetLegAngle) to isolate
// which hip motor is tracking poorly during/after a ramp. Check
// HeightCtrl_HasSentHip() first -- before the first-ever send this session,
// the Get* values are 0, not a real "commanded angle".
bool  HeightCtrl_HasSentHip();
float HeightCtrl_GetLastSentHipL();
float HeightCtrl_GetLastSentHipR();
float HeightCtrl_GetDiffTargetCm();
float HeightCtrl_GetLeftCm();           // current per-leg heights
float HeightCtrl_GetRightCm();
void  HeightCtrl_StepUp();
void  HeightCtrl_StepDown();
void  HeightCtrl_Preset_Crouch();
void  HeightCtrl_Preset_Nominal();
void  HeightCtrl_Preset_Tall();
void  HeightCtrl_Preset_Extra();
void  HeightCtrl_Freeze();            // target = current; stops the ramp where it is
void  HeightCtrl_Tick();              // call at 200 Hz; sends leg cmds at 50 Hz while ramping
float HeightCtrl_GetTargetCm();
float HeightCtrl_GetCurrentCm();
bool  HeightCtrl_IsRamping();
