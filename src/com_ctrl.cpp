#include "com_ctrl.h"
#include <math.h>
#include "imu.h"

// All TUNE-ON-BENCH per the Grand Plan -- these are starting values from
// the design spec, not measured/optimal.

// Deadband on (vertical accel - self-motion FF) before it feeds the washout
// integrator. MEASURED 2026-07-13 (logs/2026-07-13_120716_vacc_baseline.log,
// 18 s quiet balance, engage transient excluded): housekeeping noise floor
// rms=0.10, p99=0.25, max=0.33 m/s^2 -- the original 0.15 guess sat at ~p90,
// INSIDE the noise, so the integrator was fed continuously (the "perpetual
// height wander"). 0.45 = 1.4x measured max; real events are far above it
// (G1 hand-lift peaked at +9.1). Note this zeroes noise going INTO an
// integrator, not an applied correction -- no release-and-reoscillate risk
// (the Task 2 deadband lesson).
static constexpr float AZ_DEADBAND_MS2 = 0.45f;

// Washout (high-pass) time constant for the accel->velocity estimate. Plain
// integration of a_z would drift forever on any small DC bias; decaying
// toward 0 with this time constant trades away true-DC tracking (which
// isn't available here anyway -- absolute height isn't cleanly observable,
// per the Grand Plan) for a bounded, non-drifting "recent vertical
// velocity" signal.
static constexpr float TAU_WASH_S = 0.8f;

// trim_target = clamp(-KA*a_z - KV*v_z, +-TRIM_MAX). KA=0 to start: an
// accel->position term with NO damping is an undamped oscillator (settled
// design decision) -- KV (velocity/skyhook damping) is what's allowed to do
// anything on the first bench pass (G3). Raise KA only after KV is
// confirmed stable and quiet at rest.
// Task 4 (BALANCE integration) gains, 2026-07-12. History:
//   spec start KA=0/KV=4/MAX=1.0/SLEW=4 -> G3 visibility KV=6/MAX=1.8/SLEW=6
//   -> these.
// KA raised off 0 per the G3 lesson: the washout velocity term has ~0.8s of
// memory, so a completed disturbance leaves an opposite-signed tail that
// (through the actuation lag) dominates what you SEE. The accel term is
// memoryless -- it acts during the event only -- so KA>0 shifts the response
// toward the event and away from the tail. 0.1 cm/(m/s^2) = 1 cm at a 10
// m/s^2 impact; the 0.15 deadband keeps its noise contribution ~0.02 cm.
// KV=6 kept (bench-validated at G3). TRIM_MAX back to the spec's +-1.0 for
// BALANCE -- +-1.8 was a bench-visibility value; autonomous height authority
// while balancing starts conservative (G4 may raise it after CSV review).
// G4 attempt 3 (2026-07-13) re-tune: KV 6->2, KA back to 0, slew 6->2.5.
// Attempt 3 railed the trim bang-bang at ~0.5 Hz and dragged the balance
// loop into a +-6 deg / +-800 dps limit cycle: deadband-then-high-gain acts
// as a RELAY around the balance resonance, and no deadband can sit above
// the rocking's own vertical accel (+-3..7 m/s^2). New anchor: commanded
// height ramps at 3 cm/s are PROVEN stable during balance (r/k combos), so
// the trim's dynamics are capped to ramp-like rates and the busy-gate (see
// ComCtrl_Tick) breaks the resonance path. KA rejoins only after this
// baseline is demonstrably stable.
static constexpr float KA_START    = 0.0f;    // cm per (m/s^2)  TUNE-ON-BENCH (off until stable)
static constexpr float KV_START    = 2.0f;    // cm per (m/s)
static constexpr float TRIM_MAX_CM = 1.0f;

// Slew the actual (sent) trim toward trim_target. Faster than the 1.15 cm/s
// diff slew in height_ctrl -- absorbing a real disturbance needs to react
// quicker than a human-issued lean command does.
static constexpr float TRIM_SLEW_CM_S = 2.5f;  // ~= the proven-in-balance ramp speed

// Trim centering (added after G4 attempt 2, 2026-07-12): a balancing robot
// is never acceleration-quiet -- the balance loop's own housekeeping (tilt
// wobble, wheel surges, gravity leak) drizzles just enough residual az past
// the deadband/self-FF to keep the washout integrator fed, so the trim
// wandered "bit by bit, perpetually" instead of holding a height. This
// leaks the trim toward 0 with a ~2 s time constant: far slower than a bump
// event (which punches through at full strength), far faster than the
// wander it suppresses. Nominal height becomes the attractor.
static constexpr float TAU_CENTER_S = 2.0f;

// Self-motion rejection (added after the G4 first-attempt oscillation,
// 2026-07-12): on the ground, the loop's own leg motion vertically
// accelerates the chassis -- ~0.7 m/s^2 for a 1 cm trim at the 6 cm/s slew,
// ~5x the deadband -- so the IMU hears the actuator and the loop excites
// itself (G3 on a hand-held robot never saw this: legs swung freely, chassis
// didn't move). Fix = kinematic feedforward: the commanded trim trajectory
// is known exactly, so its second difference IS the self-generated vertical
// accel (assuming ground contact + legs tracking); subtract it, matched
// through the same EMA alpha the IMU-side accel gets, and the loop only
// hears EXTERNAL disturbance. On a stand (no ground contact) the
// subtraction injects a phantom *opposing* signal instead -- negative
// feedback, safely stabilizing, so IDLE bench behavior stays benign.
// SELF_FF_GAIN scales the subtraction: 1.0 = full cancellation assuming
// perfect tracking; lower if bench shows overcompensation. TUNE-ON-BENCH.
static constexpr float SELF_FF_GAIN = 1.0f;
static constexpr float EMA_ALPHA    = 0.3f;   // matches imu.cpp's s_vacc_filt

static bool  s_enabled = false;
static float s_ka   = KA_START;
static float s_kv   = KV_START;
static float s_vz   = 0.0f;   // washed-out vertical velocity estimate, m/s
static float s_trim = 0.0f;   // current (slewed) trim, cm -- what callers read
static float s_trim_p1 = 0.0f, s_trim_p2 = 0.0f;  // trim history for the 2nd difference
static float s_aself_filt = 0.0f;  // EMA'd commanded self-accel, m/s^2

void ComCtrl_Init() {
    s_enabled = false;
    s_vz      = 0.0f;
    s_trim    = 0.0f;
    s_trim_p1 = s_trim_p2 = 0.0f;
    s_aself_filt = 0.0f;
}

void ComCtrl_SetEnabled(bool on) {
    s_enabled = on;
    if (!on) {
        s_vz   = 0.0f;
        s_trim = 0.0f;
        s_trim_p1 = s_trim_p2 = 0.0f;
        s_aself_filt = 0.0f;
    }
}

bool ComCtrl_IsEnabled() { return s_enabled; }

void ComCtrl_Tick(float dt_s, bool hold) {
    if (!s_enabled) { s_vz = 0.0f; s_trim = 0.0f; return; }

    // Self-motion feedforward: commanded self-accel = 2nd difference of the
    // trim history (cm -> m via 0.01), matched-EMA'd, subtracted from the
    // measured accel BEFORE the deadband -- so the deadband gates residual
    // noise, not the difference of two large correlated signals.
    float a_self_raw = (s_trim - 2.0f * s_trim_p1 + s_trim_p2) * 0.01f / (dt_s * dt_s);
    s_aself_filt += EMA_ALPHA * (a_self_raw - s_aself_filt);

    float az = IMU_GetVerticalAccel() - SELF_FF_GAIN * s_aself_filt;
    if (hold) az = 0.0f;  // balance busy: go deaf, keep acting on stored v_z
    if (fabsf(az) < AZ_DEADBAND_MS2) az = 0.0f;

    // Washout integrator: integrate accel into velocity, then bleed it back
    // toward 0 with time constant TAU_WASH_S (high-pass, not a plain
    // integral -- keeps this from drifting off on any small DC bias).
    s_vz += az * dt_s;
    s_vz -= s_vz * (dt_s / TAU_WASH_S);

    // Sign: body shoved UP (az > 0) -> legs should SHORTEN (trim negative)
    // to absorb the shove -- same idea as knees bending on landing a jump.
    float trim_target = -s_ka * az - s_kv * s_vz;
    if (trim_target >  TRIM_MAX_CM) trim_target =  TRIM_MAX_CM;
    if (trim_target < -TRIM_MAX_CM) trim_target = -TRIM_MAX_CM;

    // Slew the actually-commanded trim toward the target; shift the history
    // AFTER the move so the 2nd difference above sees the full trajectory.
    float max_step = TRIM_SLEW_CM_S * dt_s;
    float d = trim_target - s_trim;
    if (d >  max_step) d =  max_step;
    if (d < -max_step) d = -max_step;
    s_trim_p2 = s_trim_p1;
    s_trim_p1 = s_trim;
    s_trim += d;
    s_trim -= s_trim * (dt_s / TAU_CENTER_S);  // centering leak (see TAU_CENTER_S)
}

float ComCtrl_GetTrimCm() { return s_trim; }
