#include <Arduino.h>
#include <IntervalTimer.h>
#include "imu.h"
#include "leg_fk.h"
#include "leg_ik.h"
#include "leg_calib.h"
#include "height_ctrl.h"
#include "com_ctrl.h"
#include "pid.h"
#include "motor_bus.h"
#include "wheel_control.h"

static void print_commands();  // forward declaration

// --- Tilt axis: Roll goes negative when tilting forward → TILT_SIGN = -1
#define TILT_AXIS_IS_ROLL
static constexpr float TILT_SIGN       = +1.0f;
static constexpr float TILT_OFFSET_DEG = +3.40f; // re-measured 2026-07-03 (Kd=1.5 run): true eq at
                                                 // nominal stance. Now the SEED for the tilt-offset
                                                 // table below (equilibrium tilt is height-dependent);
                                                 // PID2's Ki still trims the residual per run.

#ifdef TILT_AXIS_IS_PITCH
  #define GET_TILT() IMU_GetPitch()
#else
  #define GET_TILT() IMU_GetRoll()
#endif

// --- Side-tilt axis: confirmed 2026-07-09 on the bench (robot on a stand,
// hand-tilted, watched live on 'p'). GET_TILT() above already claims Roll for
// fore/aft, so side-to-side comes out of the other channel: tilting the bot
// to ITS RIGHT (ID3/ID4 side) reads POSITIVE on IMU_GetPitch(), left is
// negative. Offset is the reading at level stance.
//
// UPDATED 2026-07-12 from bench captures ('F'): IDLE-on-stand = 3.994 deg
// (was 4.34 eyeballed); BALANCE (riding at nominal height) = 3.670/3.645
// deg, two captures agreeing to 0.025 deg -- a real ~0.33 deg IDLE-vs-
// BALANCE gap (bigger than SIDE_TILT_DEADBAND_DEG), consistent with the
// eyeballed motor zeros loading asymmetrically under full weight-on-wheels
// vs resting on the stand. Using the IDLE value here since the side-tilt
// loop ('y') is IDLE-only for now (Stage 1) -- IF it's ever extended to run
// during BALANCE, it needs its OWN ~3.66 offset, not this one; don't reuse
// this constant there without re-deriving it, same load-dependence lesson.
static constexpr float SIDE_TILT_OFFSET_DEG = 3.99f;
// BALANCE-state side-tilt offset -- RUNTIME, auto-calibrated each session
// (2026-07-17, was a hardcoded constexpr). History: three different "correct"
// values captured the same day (1.741, 3.14, converged-1.8) -- with y off,
// IMU_GetPitch() is uncontrolled and reflects floor tilt + robot asymmetry
// COMBINED, not a pure robot constant, so a compile-time number goes stale
// with any reposition. Fix: 'V' runs an on-demand calibration (balance at 'h',
// legs level, no posture loops active) that waits for the fore-aft loop to
// settle (reusing balance_is_busy(), the same detector com_ctrl's busy-gate
// uses), then averages IMU_GetPitch() over OFFSET_CAL_WINDOW_TICKS of
// CONTINUOUS calm samples. A jerk mid-average DISCARDS the attempt and re-waits
// -- crucial for correctness: a jerk moves the robot between equilibria (user
// saw 2.05 before a jerk, 1.8 after), so averaging pre+post-jerk samples would
// blend two different values into a wrong offset. Every averaged sample must
// come from ONE uninterrupted settled stretch. If it can't string together a
// continuous window within the time budget, it aborts -- an offset from a robot
// that won't settle is meaningless. Falls back to SIDE_TILT_OFFSET_BAL_DEFAULT_DEG
// (last known good) until calibrated this session -- do NOT trust the default
// across a reposition/reboot without re-running 'V'.
static constexpr float SIDE_TILT_OFFSET_BAL_DEFAULT_DEG = 1.8f;
static float s_side_tilt_offset_bal = SIDE_TILT_OFFSET_BAL_DEFAULT_DEG;

// Offset-calibration ('V') state machine -- see control_tick() for the tick
// logic. SETTLE phase requires this many CONSECUTIVE non-busy ticks before
// starting the average; AVERAGE phase then integrates pitch over this many
// CONTINUOUS non-busy ticks. A busy tick during AVERAGE discards the attempt
// back to SETTLE (a partial average could span two equilibria -- never commit
// it). MAX_WAIT is a TOTAL budget across all attempts: if the robot can't hold
// a continuous window within it, abort (it isn't settling).
static constexpr uint32_t OFFSET_CAL_SETTLE_TICKS   = (uint32_t)(2.0f  * 200.0f);  // 2s continuously calm to start
static constexpr uint32_t OFFSET_CAL_WINDOW_TICKS   = (uint32_t)(15.0f * 200.0f);  // 15s continuous average (user-specified)
static constexpr uint32_t OFFSET_CAL_MAX_WAIT_TICKS = (uint32_t)(90.0f * 200.0f);  // 90s total budget, else abort
static bool     s_offset_cal_active   = false;
static bool     s_offset_cal_settling = false;   // sub-phase: waiting for a calm stretch
static uint32_t s_offset_cal_settle_n = 0;        // consecutive non-busy ticks so far (this settle attempt)
static uint32_t s_offset_cal_wait_n   = 0;        // TOTAL active ticks since 'V' start (budget timeout)
static uint32_t s_offset_cal_sample_n = 0;
static float    s_offset_cal_sum      = 0.0f;
static uint32_t s_offset_cal_progress_n = 0;      // ticks since last progress print

// --- Robot states ---
enum RobotState : uint8_t { STATE_IDLE, STATE_BALANCE, STATE_ESTOP };
static volatile RobotState state = STATE_IDLE;

// --- 200 Hz control timer (deferred-ISR pattern) ---
static IntervalTimer controlTimer;
static volatile bool  tick_flag = false;
static void onTick() { tick_flag = true; }

// --- PID controllers ---
// PID1 (inner): tilt (deg) → wheel speed command (deg/s)
//   Confirmed good 2026-07-03: converges 4->2.65 deg in ~0.4 s, hovers at eq with cmd~0.
// PID2 (outer): wheel velocity (deg/s) → tilt setpoint (deg)
//   Enabled 2026-07-03 after PID1 ground validation. Without it the translational state
//   is unfed-back (slow ~2 s oscillation grew until divergence — observed). Error is in
//   dps, so integral_limit 100 gives Ki full authority of ~±3 deg (old limit 5 capped
//   the I-term at a useless 0.15 deg).
static PID pid1(55.0f, 0.0f, 2.0f,  0.005f, -1000.0f, 1000.0f, 500.0f);  // Kp: 65->60->65->57->50->45, 2026-07-15
                                                                          // (45 hand-set by user, reported as
                                                                          // "much better" -- slight residual
                                                                          // oscillation remains). Below the original
                                                                          // "empirically best" 35/40/65 test value,
                                                                          // which pre-dates this session's l->o
                                                                          // incidents -- untested against them.
                                                                          // Kd: 2.0->2.5->2.0, 2026-07-15, back to
                                                                          // the chosen damping compromise. History:
                                                                          // 3.0 grew a ~12 Hz tremble, 1.5 clean but
                                                                          // left a +/-0.6 deg 1.3 s limit cycle post
                                                                          // sign fix -> 2.0 was the chosen damping
                                                                          // compromise (ceiling noted as
                                                                          // ~3.0). 2.5 sits between the known-good
                                                                          // 2.0 and the known-bad-for-tremble 3.0 --
                                                                          // untested, not a repeat of a known failure.
static PID pid2(0.025f, 0.01f, 0.0f, 0.005f,   -5.0f,    5.0f, 300.0f);  // velocity loop, fed FILTERED speed.
                                                                          // I-limit 100->300 (2026-07-07): at Ki=0.01,
                                                                          // limit 100 = only +/-1.0 deg steady trim; the
                                                                          // new IK stance moved true eq to ~4.9 deg, a
                                                                          // 1.55 deg error from the 3.40 seed -> PID2
                                                                          // saturated, permanent backward creep (observed,
                                                                          // log-verified: integral pinned at cap, speed
                                                                          // ~-60 steady). 300 -> +/-3 deg authority.
                                                                          // Kp: 0.05 rang at ~1 Hz -> 0.025 stable.
                                                                          // Ki=0.02 enabled 2026-07-03 after both loops
                                                                          // verified stable: P-only demanded a permanent
                                                                          // -34 dps creep for the 0.85 deg offset error and
                                                                          // ran out of authority (observed slow runaway).
                                                                          // I-limit 100 -> up to +/-2 deg of slow trim.

// Low-passed wheel speed for the outer loop. The motor tracks speed commands within
// ~1 tick, so feeding pid2 raw speed closes a positive loop of gain Kp1*Kp2 at 200 Hz
// (5.2 at 65*0.08 -> clamp-to-clamp bang-bang, observed 2026-07-03). Drift is a
// seconds-scale signal; EMA tau = dt/alpha = 0.005/0.05 = 0.1 s.
static float s_speed_filt = 0.0f;

// Side-tilt (roll) PD: SIDE_TILT_OFFSET_DEG-corrected IMU_GetPitch() -> HeightCtrl
// diff target (h_right - h_left, cm). Output clamped to +/-2.0 cm -- HEIGHT_DIFF_MAX_CM
// is 7.0, but height_ctrl.h flags diff as "UNTESTED beyond +/-2.0", so this new loop
// starts inside the already-validated range rather than the full mechanical limit.
// Ki=0, Kd=0 to start: verify sign/scale on the bench first, same tuning order the
// primary balance loop used (P-only -> add D -> add I last).
// Kp: 0.3 -> 1.0 -> 1.5 (2026-07-16). CLAMP: +-2.0 -> +-3.5 cm (2026-07-16). Key
// finding: raising Kp 0.3->1.0 barely changed the leg extension because the +-2.0
// OUTPUT CLAMP was saturating -- any tilt > ~2 deg pinned diff at 2.0 regardless of
// Kp. The +-2.0 was set when diff was "untested beyond +-2.0"; full splits (9/0) are
// now tested to +-3.5 cm/leg, so the clamp is raised to match (HEIGHT_DIFF_MAX_CM/2).
// This, not Kp, is what lets it "extend more strongly" for larger tilts. Kp 1.5
// targets the upper end of the plan's 60-70% P-only correction.
// Ki: 0 -> 0.3 -> 0.0, Kp: 1.5 -> 0.4, clamp: +-3.5 -> +-1.2 cm (2026-07-17,
// post near-fall CSV). At Kp=1.5/Ki=0.3/clamp=3.5, riding into a 1cm bump made
// h_diff OSCILLATE (+-1cm swings) then RAIL at -3.5 and stick -- that sustained
// max asymmetric lean coupled into the fore/aft axis and drove pid2 into its
// output clamp (tilt_sp pinned at LUT+5.0 for seconds), wheel speed ran away to
// ~600 dps, fore/aft tilt hit 9 deg before recovery (hand-caught). Root cause:
// too much gain + too much authority + no isolation from fore/aft disturbance
// events (see balance_is_busy() gate below, and the filter above it). Ki=0
// removes the wind-up-and-hold failure mode entirely; Kp=0.4 and the +-1.2 cm
// clamp bound how much fore/aft-destabilizing lean this loop can ever command,
// even if the sideways reading is misbehaving. Retune UP from here only after
// re-verifying stability on a STATIC block (E2), not a ride-into-bump (E2b).
static PID side_pid(0.4f, 0.0f, 0.0f, 0.005f, -1.2f, 1.2f, 3.0f);

// Deadband on the raw side-tilt reading, and a hysteresis before re-issuing
// HeightCtrl_SetDiff() -- without these, IMU noise (~0.1-0.2 deg) keeps the
// diff target jittering every tick, which keeps HeightCtrl "ramping" and
// fires leg commands at 50 Hz continuously even at rest (risks re-triggering
// the 2026-07-07 hip-jiggle, which was traffic/pose sensitive).
static constexpr float SIDE_TILT_DEADBAND_DEG   = 0.3f;
static constexpr float SIDE_TILT_DIFF_HYST_CM   = 0.05f;

// Low-pass on the sideways reading (2026-07-17, post near-fall CSV). The loop
// acts on s_side_tilt_filt, never the raw per-tick side_tilt -- smooths the
// noise/transients that fed the oscillate-then-rail failure. TUNE-ON-BENCH:
// ALPHA=0.1 ~= tau 0.05s at 200Hz (5-tick lag) -- lower = smoother/slower,
// higher = snappier/noisier. Re-seeded to the raw value (no filter lag) on
// every resume from a busy-gated pause -- see balance_is_busy() gate below.
static constexpr float SIDE_TILT_FILT_ALPHA = 0.1f;
static float s_side_tilt_filt = 0.0f;

// Offset-capture QoL: on enabling 'y', average raw IMU_GetPitch() over ~1s
// and print it as a suggested SIDE_TILT_OFFSET_DEG -- informational only,
// does NOT auto-adopt (user decides after comparing against the hardcoded
// value and the G2 bench result).
static constexpr int   SIDE_TILT_CAPTURE_SAMPLES = 200;  // ~1s at 200 Hz
static bool  s_side_tilt_capturing   = false;
static int   s_side_tilt_capture_n   = 0;
static float s_side_tilt_capture_sum = 0.0f;

// General-purpose pitch (side-tilt) offset capture, 'F' -- unlike the 'y'
// capture above (IDLE + closed-loop-enabled only), this one is state-
// agnostic and read-only: no actuation gated on it, so it also works while
// BALANCING. Exists because the leg motor zero references were eyeballed,
// not precision-calibrated -- under balancing load one side may bear
// slightly more force than the other, so the true level-point offset while
// balancing could genuinely differ from the IDLE-on-a-stand offset. Same
// averaging window (SIDE_TILT_CAPTURE_SAMPLES) so the two are comparable.
static bool  s_pitch_capture_active = false;
static int   s_pitch_capture_n      = 0;
static float s_pitch_capture_sum    = 0.0f;

// --- Keyboard ride (arrow keys over serial) ---
// Serial carries no key-up events, so each arrow press commands motion for
// RIDE_DEADMAN_MS and the OS key-repeat refreshes it while held; release -> the
// window expires and the target coasts back to 0. Targets are slew-limited so a
// keypress never steps the setpoint. Fwd/back rides through PID2's velocity
// setpoint (robot leans itself into motion, balance loop stays in charge).
// Turn is a differential wheel offset — cancels exactly in avg_speed, so the
// balance loop never sees it.
static constexpr float    RIDE_SPEED_FWD_DPS  = 92.4f;  // 80->92->100->110->132->105->92.4 (2026-07-17): 105 still
                                                         // drifted past a cruise limit (wheel-speed SATURATION -- no
                                                         // headroom above cruise for balance corrections). Dropped to
                                                         // match RIDE_SPEED_BACK_DPS (92.4), the backward limit that
                                                         // has not been drifting. This is the wheel-ceiling floor;
                                                         // going lower trades ride speed for more correction headroom.
static constexpr float    RIDE_SPEED_BACK_DPS = 92.4f;  // fwd -30% (2026-07-09): backward felt too fast
static constexpr float    TURN_DPS            = 59.4f;  // per-wheel differential (40->45->54->59.4, +10% 2026-07-09)
static constexpr uint32_t RIDE_DEADMAN_MS     = 600;    // > OS initial key-repeat delay
static constexpr float    RIDE_SLEW_PER_TICK  = 0.8f;   // 160 dps/s ramp
static constexpr float    TURN_SLEW_PER_TICK  = 0.5f;   // 100 dps/s ramp
static float    s_ride_target = 0.0f, s_ride_v = 0.0f;  // fwd/back: target + slewed value
static float    s_turn_target = 0.0f, s_turn_v = 0.0f;  // turn: target + slewed value
static uint32_t s_ride_deadline = 0, s_turn_deadline = 0;

static void ride_clear() {
    s_ride_target = 0.0f; s_ride_v = 0.0f;
    s_turn_target = 0.0f; s_turn_v = 0.0f;
}

// Balance-busy detector (factored out 2026-07-17, was inline in the com_ctrl
// gate only -- G4 attempt 3 lesson: while the fore-aft loop is actively
// fighting, its own rocking contaminates OTHER sensor-derived signals (vertical
// accel there; sideways IMU_GetPitch() here, for the 'V' offset calibration).
// Thresholds sit above the measured quiet-balance envelope (|tilt err| <= ~0.6
// deg, wheel deviation <= ~25 dps, 2026-07-13 baseline) and below bump-reaction
// levels. Wheel criterion is DEVIATION from the commanded ride speed, not
// absolute output, so it doesn't read "busy" for an entire ordinary ride.
static bool balance_is_busy() {
    return (state == STATE_BALANCE) &&
           (fabsf(pid1.GetLastError())            > 1.2f ||
            fabsf(pid1.GetLastOutput() - s_ride_v) > 120.0f);
}

// Arm the pitch-offset calibration state machine. Shared by the auto-start on
// 'b' and the manual 'V' re-trigger -- each caller checks its own preconditions
// (BALANCE, 'y' off, legs level) and prints its own message.
static void arm_offset_cal() {
    s_offset_cal_active     = true;
    s_offset_cal_settling   = true;
    s_offset_cal_settle_n   = 0;
    s_offset_cal_wait_n     = 0;
    s_offset_cal_sample_n   = 0;
    s_offset_cal_sum        = 0.0f;
    s_offset_cal_progress_n = 0;
}

// --- Telemetry ---
static bool csv_stream   = false;
static bool s_vacc_stream = false;  // 'E' -- hands-free vacc readout for G1 bench test

// Diagnostic probe 1 (2026-07-15), 'T': control_tick() wall-clock duration.
// Answers whether a leg-send tick actually exceeds the 5ms/200Hz budget --
// the suspected mechanism behind the 100Hz send-rate incident, never
// directly measured. Read-only: wraps the EXISTING call in loop(), no
// change to control_tick() itself or anything it does. Off by default.
static bool     s_tick_timing_enabled = false;
static uint32_t s_tick_dur_min = 0xFFFFFFFFUL;
static uint32_t s_tick_dur_max = 0;
static uint64_t s_tick_dur_sum = 0;
static uint32_t s_tick_dur_n   = 0;
static constexpr uint32_t TICK_TIMING_WINDOW = 200;  // report every ~1s at 200Hz

// Diagnostic probe 2 (2026-07-15), 'I': IMU packets consumed per control
// tick. Resolves the unexplained contradiction between pid.cpp's D-filter
// comment (claims new IMU samples arrive only every 2nd tick, ~100Hz
// effective) and the 200Hz assumption baked into the staleness guard and
// vacc EMA. Read-only tally of IMU_GetPacketsLastUpdate(), no control-path
// change. Off by default.
static bool     s_imu_rate_probe_enabled = false;
static uint32_t s_imu_probe_zero  = 0;  // ticks with 0 new packets (stale repeat)
static uint32_t s_imu_probe_one   = 0;  // ticks with exactly 1 (expected at 200Hz)
static uint32_t s_imu_probe_multi = 0;  // ticks with 2+ (tick fell behind)
static uint32_t s_imu_probe_ticks = 0;
static constexpr uint32_t IMU_PROBE_WINDOW = 200;  // report every ~1s at 200Hz

// Hip tracking probe (2026-07-15), 'H': commanded vs ACTUAL (encoder) angle
// for ID1 (hip-L) and ID3 (hip-R), streamed during and after a height ramp --
// to isolate which hip motor is causing the persisting end-of-ramp
// oscillation (only ID1/ID3's internal gains were re-tuned on the LK tool,
// not ID2/ID4). Decimated to 50Hz (every 4th tick of the 200Hz loop): a real
// encoder read adds bus time on top of the ramp's own position-command
// traffic on the SAME Serial5 bus, so this deliberately doesn't poll every
// tick -- polling too fast risks perturbing the very oscillation it's
// trying to measure. Off by default, read-only, no control-path change.
static bool     s_hip_track_enabled = false;
static constexpr uint32_t HIP_TRACK_EVERY_N_TICKS = 4;  // 200Hz/4 = 50Hz

// ID3 (hip-R) angle/speed/current live probe (2026-07-16) -- bench-tuning
// aid: streams a CSV the host can graph live while gains are adjusted on the
// LK motor tool over its OWN separate wire to ID3 (confirmed with user,
// physically isolated from this Teensy's Serial5 bus -- no multi-master
// contention). 50Hz, same cadence as the hip-tracking probe -- comfortably
// above the ~1-12Hz jitter frequencies seen in prior PID2/hip investigations.
static bool     s_id3_probe_enabled = false;
static constexpr uint32_t ID3_PROBE_EVERY_N_TICKS = 4;  // 200Hz/4 = 50Hz

static RobotState prev_state = STATE_IDLE;
static volatile uint32_t tick_count = 0;
// Set only by the temp-ESTOP path (MotorBus_EStop pauses ALL motors, legs
// included, with no software re-arm). 'b' must refuse to enter BALANCE while
// this is set — only a power cycle clears it (reboot re-inits to false).
static bool s_estop_needs_power_cycle = false;

// Stage 2 gate: height changes during BALANCE (default OFF — Stage 1 is
// IDLE-on-stand only). Toggle with '2'.
static bool s_height_in_balance = false;

// Side-tilt (roll) closed loop: default OFF, IDLE-only for now (Stage 1, same
// staging as height/lean before it). Toggle with 'y'. BALANCE-gating is a
// deliberate later step, not done here.
static bool s_side_tilt_enabled = false;

// Vertical CoM regulator (com_ctrl): default OFF, IDLE-only for now (Task 3,
// Stage 2 of the Grand Plan). Toggle with 'j'. This flag is "user wants it
// on" and persists across state changes -- ComCtrl's OWN internal enabled
// flag (set via ComCtrl_SetEnabled() every tick below) tracks "is it
// actually running right now" (only true while this flag AND state==IDLE),
// same two-layer pattern as s_side_tilt_enabled/side_pid above. BALANCE-
// gating is Task 4, not done here.
static bool s_com_ctrl_enabled = false;

// Task 4 gate: allow the posture loops (vertical CoM now; side-tilt joins in
// Task 4b) to run during BALANCE. Deliberately separate from '2'
// (s_height_in_balance = manual height/lean commands in BALANCE) so the
// autonomous loops and the human commands can be enabled independently.
// Default OFF; toggle '4' (digit freed by the 2026-07-12 command cleanup).
static bool s_posture_in_balance = false;

// Boot stance / nominal height. The stance is IK-derived through the current
// calibration at LEG_STANCE_X_CM, so the boot pose and the 'o' preset track
// leg_calib.h and match a tilt-LUT breakpoint. 15.53->14.94 (2026-07-07):
// with x=-0.616 this reproduces the OLD proven stance raws exactly
// (L -42.04/-20.28, R +42.04/+20.28) — quiet hips, eq ~3.4 ('8' A/B result).
static constexpr float STANCE_HEIGHT_CM = 14.94f;

// --- Height-dependent tilt-offset feedforward (Stage 2) -------------------
// Equilibrium tilt shifts with body height: as the legs fold the CoM moves
// horizontally relative to the wheel contact, so the tilt setpoint that holds
// still is different at each height. A constant offset is only right at one
// height; elsewhere the robot creeps and the wheels drive to chase it (the
// backward-creep-during-height-change symptom). This is a small breakpoint
// table, linearly interpolated on the *current* (ramped) height, feeding the
// balance setpoint. Seeded uniformly with TILT_OFFSET_DEG so behavior is
// IDENTICAL to the old constant until trimmed live with '['/']'. RAM-only:
// once captured, copy the values printed by 'i' into TILT_LUT_O and reflash.
// Breakpoints on an integer-cm grid across the low region + the presets, so a
// '3' capture lands exactly on a breakpoint (reach any of them via 'c' then
// u/d in 0.5 cm steps; presets o/g land on 14.94/16.5). Below the first
// breakpoint (10.0) and above the last (18.0) the offset is held flat.
static constexpr int   TILT_LUT_N = 10;
static constexpr float TILT_LUT_H[TILT_LUT_N] =
    { 10.0f, 11.0f, 12.0f, 13.0f, 14.0f, 14.94f, 16.5f, 17.0f, 18.0f, 20.0f };
// Captured on hardware 2026-07-09 ('3' bumpless / '[' ']' trim, x=-0.616 stance).
// Trusted anchors: 12.0=3.316 (settled, -4 dps), 14.0=3.36, 14.94=2.88, 16.5=2.42,
// 18.0=2.318. 10.0/11.0/13.0 are LINEARLY INTER/EXTRAPOLATED from the 12.0-14.0
// line (slope +0.022 deg/cm, region is nearly flat): 10.0=3.272, 11.0=3.294,
// 13.0=3.338. 17.0=2.386 is LINEARLY INTERPOLATED on the 16.5->18.0 line (slope
// -0.068 deg/cm) -- replaces the old 2.86 (from a since-discarded capture) and
// today's 2.951 recapture (rejected, taken at 11.1 dps, not settled). 20.0=2.182
// EXTRAPOLATED continuing that same 16.5->18.0 line -- UNCAPTURED, feeds the
// experimental 'e' extend (~20 cm IK limit, near-singular). Sub-12 cm and 20.0
// are both extrapolated, not measured -- capture live if running there for real.
static float           TILT_LUT_O[TILT_LUT_N] =
    { 3.272f, 3.294f, 3.316f, 3.38f, 3.36f, 2.88f, 2.42f, 2.386f, 2.318f, 2.182f };
static constexpr float TILT_TRIM_STEP = 0.05f;

// --- Ramp riding: slope feedforward (Stage 3, 2026-07-09) -----------------
// PID2's integral already cancels a constant slope (it's just another steady
// disturbance) — but its clamp (integral_limit 300, Ki=0.01 -> ~+/-3 deg,
// see pid2 comment above) was sized for the height-LUT residual, not a real
// incline. On a ramp steep enough to need more than that, PID2 saturates and
// the old backward-creep symptom comes back, now driven by slope instead of
// height.
//
// Fix: don't give PID2 more clamp — bleed its steady output into a separate,
// slower-moving offset instead (same idea as a cascaded/reset-windup
// integrator). Every settled tick, s_slope_offset_deg absorbs a small slice
// of pid2's last output; pid2's own integral then relaxes back toward zero
// on its own (it's driven by the same error it's now sharing the correction
// for), which frees its clamp for transients again while the slope load
// lives in s_slope_offset_deg instead. tilt_sp = LUT(h) + offset - pid2(...),
// so this is strictly additive to the existing height feedforward.
//
// Off by default: 'z' arms it. Disabled, s_slope_offset_deg is pinned at 0
// and tilt_sp is byte-for-byte the pre-ramp-control expression.
static bool  s_slope_ff_enabled  = false;
static float s_slope_offset_deg  = 0.0f;
static constexpr float SLOPE_FF_BLEED_GAIN = 0.002f; // per-tick fraction of pid2 output bled in
static constexpr float SLOPE_FF_MAX_DEG    = 8.0f;   // generous vs. the 20 deg tilt ESTOP
static constexpr float SLOPE_FF_SETTLE_DPS = 5.0f;   // |speed_filt - ride_v| under this = settled

static float tilt_offset_for_height(float h) {
    if (h <= TILT_LUT_H[0])              return TILT_LUT_O[0];
    if (h >= TILT_LUT_H[TILT_LUT_N - 1]) return TILT_LUT_O[TILT_LUT_N - 1];
    for (int i = 1; i < TILT_LUT_N; i++) {
        if (h <= TILT_LUT_H[i]) {
            float f = (h - TILT_LUT_H[i-1]) / (TILT_LUT_H[i] - TILT_LUT_H[i-1]);
            return TILT_LUT_O[i-1] + f * (TILT_LUT_O[i] - TILT_LUT_O[i-1]);
        }
    }
    return TILT_LUT_O[TILT_LUT_N - 1];  // unreachable
}

// Index of the breakpoint nearest a height — the one '['/']' trims.
static int tilt_lut_nearest(float h) {
    int best = 0; float bd = fabsf(h - TILT_LUT_H[0]);
    for (int i = 1; i < TILT_LUT_N; i++) {
        float d = fabsf(h - TILT_LUT_H[i]);
        if (d < bd) { bd = d; best = i; }
    }
    return best;
}

static void control_tick() {
    tick_count++;

    // Temperature ESTOP — one motor per check (~2 ms bus time), every 33 ticks
    // (~165 ms), so the full 6-motor rotation completes in ~1 s (was 6 s at one
    // check/second). Kept as single reads: a batched 6-motor sweep is ~15 ms of
    // blocking bus time inside a 5 ms tick and would stall the balance loop.
    if (tick_count % 33 == 0) {
        if (MotorBus_TempCheck(55)) {
            WheelCtrl_Stop();
            MotorBus_EStop();
            state = STATE_ESTOP;
            s_estop_needs_power_cycle = true;
            return;
        }
    }

    IMU_Update();

    if (s_imu_rate_probe_enabled) {
        uint32_t n = IMU_GetPacketsLastUpdate();
        if (n == 0)      s_imu_probe_zero++;
        else if (n == 1) s_imu_probe_one++;
        else             s_imu_probe_multi++;
        if (++s_imu_probe_ticks >= IMU_PROBE_WINDOW) {
            Serial.print("#PROBE,imu_pkts/tick (of "); Serial.print(s_imu_probe_ticks);
            Serial.print("): 0="); Serial.print(s_imu_probe_zero);
            Serial.print(" 1=");  Serial.print(s_imu_probe_one);
            Serial.print(" 2+="); Serial.println(s_imu_probe_multi);
            s_imu_probe_zero = s_imu_probe_one = s_imu_probe_multi = s_imu_probe_ticks = 0;
        }
    }

    if (s_hip_track_enabled && (tick_count % HIP_TRACK_EVERY_N_TICKS == 0)) {
        if (!HeightCtrl_HasSentHip()) {
            Serial.println("#PROBE,hip,no data yet -- no height command sent this session");
        } else {
            MotorBus_RequestHipAngles();
            double encL = MotorBus_GetLegAngle(1);
            double encR = MotorBus_GetLegAngle(3);
            float  cmdL = HeightCtrl_GetLastSentHipL();
            float  cmdR = HeightCtrl_GetLastSentHipR();
            Serial.print("#PROBE,hip,t_ms="); Serial.print(millis());
            Serial.print(",ramping=");        Serial.print(HeightCtrl_IsRamping() ? 1 : 0);
            Serial.print(",cmdL=");  Serial.print(cmdL, 3);
            Serial.print(",encL=");  Serial.print(encL, 3);
            Serial.print(",errL=");  Serial.print(cmdL - encL, 3);
            Serial.print(",cmdR=");  Serial.print(cmdR, 3);
            Serial.print(",encR=");  Serial.print(encR, 3);
            Serial.print(",errR=");  Serial.println(cmdR - encR, 3);
        }
    }

    if (s_id3_probe_enabled && (tick_count % ID3_PROBE_EVERY_N_TICKS == 0)) {
        MotorBus_RequestID3State();
        Serial.print(millis());                        Serial.print(',');
        Serial.print(MotorBus_GetID3AngleDeg(), 3);     Serial.print(',');
        Serial.print(MotorBus_GetID3SpeedDPS(), 2);     Serial.print(',');
        Serial.println(MotorBus_GetID3CurrentA(), 3);
    }

    // IMU staleness guard: HI229 streams at 200 Hz, so >50 ms with no valid packet
    // means the link is dead. Without this, a mid-balance IMU disconnect left the
    // wheels driving from a frozen tilt reading forever (IsValid latches true).
    if (state == STATE_BALANCE && IMU_MsSinceLastPacket() > 50) {
        WheelCtrl_Stop();
        state = STATE_ESTOP;
        Serial.println("ESTOP: IMU stale >50 ms (link lost?)");
        return;
    }

    if (!IMU_IsValid()) return;

    // Vertical-accel bench stream (G1 gate): hands-free readout for lifting
    // the robot while watching numbers, vs. needing a free hand to press 'p'
    // repeatedly. Runs regardless of state (IDLE/BALANCE/ESTOP) since it's
    // read-only diagnostics -- no actuation gated on it. Decimated to ~10 Hz
    // (every 20 ticks @ 200 Hz) so it's human-readable scrolling by, not a
    // 200 Hz flood.
    if (s_vacc_stream && (tick_count % 20 == 0)) {
        Serial.print("vacc="); Serial.print(IMU_GetVerticalAccel(), 2);
        Serial.print("  vtrim="); Serial.print(HeightCtrl_GetVerticalTrimCm(), 3);
        Serial.print("  acc=("); Serial.print(IMU_GetAccelX(), 2);
        Serial.print(",");       Serial.print(IMU_GetAccelY(), 2);
        Serial.print(",");       Serial.print(IMU_GetAccelZ(), 2);
        Serial.println(")");
    }

    // 'F' pitch-offset capture: state-agnostic, read-only (see declaration
    // comment) -- lets IDLE and BALANCE offsets be measured and compared.
    if (s_pitch_capture_active) {
        s_pitch_capture_sum += IMU_GetPitch();
        if (++s_pitch_capture_n >= SIDE_TILT_CAPTURE_SAMPLES) {
            s_pitch_capture_active = false;
            Serial.print("pitch capture [");
            Serial.print(state == STATE_BALANCE ? "BALANCE" :
                         state == STATE_ESTOP   ? "ESTOP"   : "IDLE");
            Serial.print("]: avg pitch = ");
            Serial.print(s_pitch_capture_sum / SIDE_TILT_CAPTURE_SAMPLES, 3);
            Serial.print(" deg (IDLE hardcoded offset: ");
            Serial.print(SIDE_TILT_OFFSET_DEG, 2);
            Serial.println(") -- compare BALANCE vs IDLE captures for a load-dependent gap.");
        }
    }

    float tilt = GET_TILT() * TILT_SIGN;

    // On state transition: send wheel stop immediately; any state change also
    // freezes the height ramp (it must never resume by surprise later).
    if (state != prev_state) {
        if (state == STATE_IDLE || state == STATE_ESTOP) {
            WheelCtrl_Stop();
            ride_clear();
        }
        HeightCtrl_Freeze();
        // Hard-reset the vertical CoM loop across ANY state change: Freeze()
        // zeroes HeightCtrl's trim, but without this, main re-applies
        // ComCtrl's still-loaded internal trim/v_z next tick -- e.g. the
        // set-down accel spike right before 'b' would carry a stale trim
        // into balance. It re-arms clean next tick if still gated on.
        ComCtrl_SetEnabled(false);
        // Deliberate: freeze (not level) any active lean -- auto-moving legs
        // mid-transition is worse. But say so, so nobody balances tilted
        // without knowing (e.g. side-tilt loop was correcting when 'b' hit).
        if (fabsf(HeightCtrl_GetDiffCm()) > 0.05f) {
            Serial.print("WARNING: lean frozen at diff(R-L)=");
            Serial.print(HeightCtrl_GetDiffCm(), 2);
            Serial.println(" cm across state change ('/' to level)");
        }
        prev_state = state;
    }

    if (state == STATE_BALANCE) {
        if (fabsf(tilt) > 20.0f) {
            WheelCtrl_Stop();
            state = STATE_ESTOP;
            Serial.println("ESTOP: tilt >20 deg");
            return;
        }

        WheelCtrl_UpdateFeedback();
        float avg_speed = WheelCtrl_GetAvgSpeedDPS();
        s_speed_filt   += 0.05f * (avg_speed - s_speed_filt);

        // Ride: expire the dead-man window, then slew the live values toward targets.
        uint32_t now_ms = millis();
        if ((int32_t)(now_ms - s_ride_deadline) >= 0) s_ride_target = 0.0f;
        if ((int32_t)(now_ms - s_turn_deadline) >= 0) s_turn_target = 0.0f;
        float d = s_ride_target - s_ride_v;
        if (d >  RIDE_SLEW_PER_TICK) d =  RIDE_SLEW_PER_TICK;
        if (d < -RIDE_SLEW_PER_TICK) d = -RIDE_SLEW_PER_TICK;
        s_ride_v += d;
        d = s_turn_target - s_turn_v;
        if (d >  TURN_SLEW_PER_TICK) d =  TURN_SLEW_PER_TICK;
        if (d < -TURN_SLEW_PER_TICK) d = -TURN_SLEW_PER_TICK;
        s_turn_v += d;

        // SIGN: on this robot, leaning tilt-positive drives travel in the NEGATIVE
        // wheel-speed direction (confirmed across every 2026-07-03 run: sustained
        // +tilt <-> speed ramping negative). Braking a negative-speed drift therefore
        // requires lowering tilt_sp, so pid2's output must be SUBTRACTED. With it
        // added (the original wiring) the outer loop was positive feedback — every
        // run diverged one-way with tilt_sp chasing the fall into its clamp.
        // Riding = PID2 regulates wheel speed to s_ride_v instead of 0.
        if (s_slope_ff_enabled) {
            bool settled = !HeightCtrl_IsRamping()
                        && fabsf(s_speed_filt - s_ride_v) < SLOPE_FF_SETTLE_DPS;
            if (settled) {
                s_slope_offset_deg -= SLOPE_FF_BLEED_GAIN * pid2.GetLastOutput();
                if (s_slope_offset_deg >  SLOPE_FF_MAX_DEG) s_slope_offset_deg =  SLOPE_FF_MAX_DEG;
                if (s_slope_offset_deg < -SLOPE_FF_MAX_DEG) s_slope_offset_deg = -SLOPE_FF_MAX_DEG;
            }
        } else {
            s_slope_offset_deg = 0.0f;
        }
        // Height for the equilibrium LUT includes the vertical trim: the trim
        // physically changes body height, so leaving it out makes every trim
        // excursion a ~0.3-0.5 deg/cm equilibrium error the balance loop must
        // chase with wheel motion -- one of the two couplings behind the G4
        // first-attempt oscillation (2026-07-12 log).
        float tilt_sp   = tilt_offset_for_height(HeightCtrl_GetCurrentCm()
                                                 + HeightCtrl_GetVerticalTrimCm())
                          + s_slope_offset_deg
                          - pid2.Update(s_ride_v, s_speed_filt);
        float wheel_cmd = pid1.Update(tilt_sp, tilt);
        // Turn: differential offset, right wheel slower / left faster = turn right.
        WheelCtrl_SetSpeed(wheel_cmd - s_turn_v, wheel_cmd + s_turn_v);

        if (csv_stream) {
            // CSV v2 (Task 5): column order is FROZEN -- the Python pipeline
            // (programming/analysis/imulog.py) parses by position against the
            // #HDR line printed when '1' toggles on. Add columns at the END
            // only, and update both the #HDR string and imulog.py together.
            Serial.print(millis());        Serial.print(',');
            Serial.print(tilt, 3);         Serial.print(',');
            Serial.print(tilt_sp, 3);      Serial.print(',');
            Serial.print(wheel_cmd, 1);    Serial.print(',');
            Serial.print(avg_speed, 1);    Serial.print(',');
            Serial.print(s_speed_filt, 1); Serial.print(',');
            Serial.print(s_slope_offset_deg, 3);              Serial.print(',');
            Serial.print(IMU_GetPitch() - SIDE_TILT_OFFSET_DEG, 3); Serial.print(',');
            Serial.print(HeightCtrl_GetCurrentCm(), 2);       Serial.print(',');
            Serial.print(HeightCtrl_GetDiffCm(), 2);          Serial.print(',');
            Serial.print(HeightCtrl_GetVerticalTrimCm(), 3);  Serial.print(',');
            Serial.println(IMU_GetVerticalAccel(), 2);
        }
    }

    // Side-tilt (roll) closed loop: IDLE always, plus BALANCE behind the '4'
    // posture flag (Task 4b step 3, 2026-07-16 -- was IDLE-only). Sets
    // HeightCtrl's diff target every tick; HeightCtrl_Tick() below does the
    // actual leg motion (diff trapezoid). Same '4' flag as the vertical-CoM
    // loop -- each posture loop still needs its own arm toggle ('y' here), so
    // '4' alone doesn't start it. State-selected offset: the BALANCE load
    // shifts the level-body reading ~0.33 deg vs on-stand IDLE.
    if (s_side_tilt_enabled &&
        (state == STATE_IDLE ||
         (state == STATE_BALANCE && s_posture_in_balance))) {
        float offset = (state == STATE_BALANCE) ? s_side_tilt_offset_bal
                                                : SIDE_TILT_OFFSET_DEG;
        float side_tilt = IMU_GetPitch() - offset;

        // Busy-gate (2026-07-17, post near-fall CSV): while the fore/aft loop
        // is actively fighting, side_tilt is BOTH contaminated (fore/aft tilt
        // couples into the sideways IMU reading at larger angles) and MOST
        // dangerous to act on (a diff correction now is what fed the fore/aft
        // axis into pid2 saturation last time). Freeze -- hold the last
        // commanded diff via HeightCtrl (no SetDiff call), re-seed the filter
        // to the raw value (no stale-filter jump on resume), and reset the PID
        // (no stored integral/derivative state carried across the pause).
        if (balance_is_busy()) {
            s_side_tilt_filt = side_tilt;
            side_pid.Reset();
        } else {
            s_side_tilt_filt += SIDE_TILT_FILT_ALPHA * (side_tilt - s_side_tilt_filt);
            // Deadband HOLDS the last commanded diff rather than zeroing the
            // input: a P-only loop needs persistent error to hold its output, so
            // feeding it 0 inside the band would collapse diff_cmd to 0, release
            // the correction via SetDiff(0), let the tilt grow past the band, and
            // seesaw forever at ~0.1 cm amplitude (with bus traffic) -- the exact
            // at-rest case the deadband exists to quiet. Skipping the update
            // holds the pose AND keeps the bus silent; the deadband also freezes
            // the integral inside the band when Ki != 0 (desired).
            if (fabsf(s_side_tilt_filt) >= SIDE_TILT_DEADBAND_DEG) {
                // side_pid.Update(setpoint=0 "level", measurement=s_side_tilt_filt)
                // gives error = -s_side_tilt_filt, so the raw output points the
                // wrong way to correct the tilt (same class of sign flip pid2
                // needs above) -- negate it. Derivation: diff = h_right - h_left
                // (height_ctrl.cpp), so diff>0 makes the RIGHT leg longer, which
                // raises the right hip and leans the body LEFT (see the ',' lean
                // comment below). A positive side_tilt means the bot is falling
                // to ITS RIGHT, which needs exactly that correction (right leg
                // extends, pulls the right side back up) -- so diff_cmd must be
                // POSITIVE when side_tilt is POSITIVE, i.e. diff_cmd = +Kp*side_tilt.
                float diff_cmd = -side_pid.Update(0.0f, s_side_tilt_filt);
                // Only re-issue the target when it actually moved -- otherwise noise
                // keeps HeightCtrl "ramping" and the bus never goes quiet at rest.
                if (fabsf(diff_cmd - HeightCtrl_GetDiffTargetCm()) > SIDE_TILT_DIFF_HYST_CM) {
                    HeightCtrl_SetDiff(diff_cmd);
                }
            }
        }

        if (s_side_tilt_capturing) {
            s_side_tilt_capture_sum += IMU_GetPitch();
            if (++s_side_tilt_capture_n >= SIDE_TILT_CAPTURE_SAMPLES) {
                s_side_tilt_capturing = false;
                Serial.print("side-tilt offset capture: suggested SIDE_TILT_OFFSET_DEG = ");
                Serial.print(s_side_tilt_capture_sum / SIDE_TILT_CAPTURE_SAMPLES, 3);
                Serial.print(" (current hardcoded: ");
                Serial.print(SIDE_TILT_OFFSET_DEG, 2);
                Serial.println(") -- ONLY valid if the bot sat level+still this past 1s"
                               " (legs may have been correcting). Not auto-adopted.");
            }
        }
    } else {
        side_pid.Reset();
        s_side_tilt_capturing = false;
    }

    // Offset calibration ('V'): BALANCE only, runs independent of s_side_tilt_enabled
    // (in fact it REQUIRES 'y' off -- refused at the command site -- so it measures the
    // natural uncorrected lean, not a loop-driven one). SETTLE phase waits for
    // balance_is_busy() to clear for OFFSET_CAL_SETTLE_TICKS straight; a busy tick
    // during AVERAGE aborts back to SETTLE rather than polluting the sum -- no partial/
    // contaminated result is ever committed.
    if (s_offset_cal_active && state == STATE_BALANCE) {
        // Total-time budget covers both "never settles" and "jerks too often to
        // ever hold a continuous window". Increments every active tick.
        if (++s_offset_cal_wait_n >= OFFSET_CAL_MAX_WAIT_TICKS) {
            s_offset_cal_active = false;
            Serial.println("offset cal: ABORTED -- couldn't hold 20s continuous calm within 90s. Robot isn't settling; calm pid1 / re-level and retry.");
        } else if (s_offset_cal_settling) {
            // Wait for a continuous calm stretch before starting the average.
            if (balance_is_busy()) {
                s_offset_cal_settle_n = 0;
            } else if (++s_offset_cal_settle_n >= OFFSET_CAL_SETTLE_TICKS) {
                s_offset_cal_settling   = false;   // -> AVERAGE, fresh
                s_offset_cal_sample_n   = 0;
                s_offset_cal_sum        = 0.0f;
                s_offset_cal_progress_n = 0;
                Serial.println("offset cal: settled, averaging 15s CONTINUOUS -- hold still, don't touch (a jerk restarts it)...");
            }
        } else {
            if (balance_is_busy()) {
                // Jerk mid-average: DISCARD this attempt. Pre-jerk samples may be
                // at a different equilibrium than post-jerk (2.05 vs 1.8) -- keeping
                // them would blend two values into a wrong offset. Only a fully
                // uninterrupted window counts.
                s_offset_cal_settling = true;
                s_offset_cal_settle_n = 0;
                Serial.println("offset cal: jerk -- DISCARDING this average, need 20s clean in a row (samples must be one equilibrium).");
            } else {
                s_offset_cal_sum += IMU_GetPitch();
                if (++s_offset_cal_sample_n >= OFFSET_CAL_WINDOW_TICKS) {
                    s_side_tilt_offset_bal = s_offset_cal_sum / (float)s_offset_cal_sample_n;
                    s_offset_cal_active = false;
                    Serial.print("calibration done. pitch offset= ");
                    Serial.print(s_side_tilt_offset_bal, 3);
                    Serial.println(" deg (adopted this session; auto-reruns on next 'b')");
                } else if (++s_offset_cal_progress_n >= 5 * 200) {  // every 5s of continuous data
                    s_offset_cal_progress_n = 0;
                    Serial.print("offset cal: ");
                    Serial.print(s_offset_cal_sample_n / 200);
                    Serial.println("s / 15s continuous...");
                }
            }
        }
    } else if (s_offset_cal_active) {
        // Left BALANCE mid-calibration -- abort, don't silently keep waiting forever.
        s_offset_cal_active = false;
        Serial.println("offset cal: ABORTED -- left BALANCE state.");
    }

    // Vertical CoM regulator (com_ctrl): IDLE always (Task 3); BALANCE only
    // behind the '4' posture flag (Task 4). Must run BEFORE HeightCtrl_Tick()
    // below so the trim it sets is fresh for this tick's send.
    // HeightCtrl_SetVerticalTrim() is called every tick regardless of
    // active/inactive so a disable (or a state change) actually propagates
    // trim->0 into HeightCtrl instead of leaving it stuck at whatever was
    // last commanded.
    if (s_com_ctrl_enabled &&
        (state == STATE_IDLE ||
         (state == STATE_BALANCE && s_posture_in_balance))) {
        // Balance-busy gate (G4 attempt 3 lesson: rocking contaminates vertical
        // accel; see balance_is_busy() -- shared with the 'V' offset calibration).
        // A real bump still gets through: its vertical impulse loads v_z ~100 ms
        // before the tilt reaction closes this gate.
        bool busy = balance_is_busy();
        ComCtrl_SetEnabled(true);   // idempotent re-arm; only false resets internal state
        ComCtrl_Tick(0.005f, busy);
    } else {
        ComCtrl_SetEnabled(false);
    }
    HeightCtrl_SetVerticalTrim(ComCtrl_GetTrimCm());

    // Height ramp: active in IDLE (Stage 1); in BALANCE behind the '2' flag
    // (manual height/lean) OR the '4' posture flag (Tick must run for the
    // trim to reach the motors -- note manual height/lean COMMANDS are still
    // rejected at the command site unless '2' is on, so '4' alone does not
    // open up u/d/presets during balance, only the autonomous trim path).
    if (state == STATE_IDLE ||
        (state == STATE_BALANCE && (s_height_in_balance || s_posture_in_balance))) {
        HeightCtrl_Tick();
    }
}

static void handle_serial_cmd(char c) {
    switch (c) {
        case 'b':
            if (state == STATE_BALANCE) break;
            if (state == STATE_ESTOP) {
                // Temp-ESTOP paused ALL motors (legs included) with no software
                // re-arm — refuse rather than balance on limp legs. Only a power
                // cycle clears this flag.
                if (s_estop_needs_power_cycle) {
                    Serial.println("Cannot start: temp-ESTOP -- power-cycle the robot first.");
                    break;
                }
                // Tilt/IMU-stale ESTOP: legs stayed armed, but require near-upright
                // before re-engaging so recovery can't fire from a big lean.
                float tilt = fabsf(GET_TILT() * TILT_SIGN);
                if (tilt >= 5.0f) {
                    Serial.print("Cannot start from ESTOP: |tilt|=");
                    Serial.print(tilt, 1);
                    Serial.println(" deg (need <5 -- stand it up first)");
                    break;
                }
            }
            pid1.Reset();
            pid2.Reset();
            s_speed_filt = 0.0f;
            s_slope_offset_deg = 0.0f;
            ride_clear();
            state = STATE_BALANCE;
            Serial.println("-> BALANCE");
            // Auto-start pitch-offset calibration (2026-07-17, user-requested):
            // the user always presses 'b' with the bot on the ground and about
            // to balance, so grab the natural offset automatically. The settle
            // phase waits ~2s for the balance to calm before averaging, so this
            // effectively "starts ~2s after 'b'". Skips if 'y' is on or the legs
            // are leaned (either would make the reading a driven/commanded lean,
            // not the natural offset) -- 'V' is still there to run it manually.
            if (!s_side_tilt_enabled && fabsf(HeightCtrl_GetDiffCm()) <= 0.1f) {
                arm_offset_cal();
                Serial.println("auto pitch-offset cal: armed -- starts once balance settles (~2s calm), 15s hold. Hold still; 'V' cancels.");
            } else {
                Serial.println("auto pitch-offset cal: SKIPPED (y on or legs leaned) -- run 'V' when level with y off.");
            }
            break;
        case 's':
            state = STATE_IDLE;
            WheelCtrl_Stop();
            ride_clear();
            Serial.println("-> IDLE");
            break;
        case 'x':
            // Manual e-stop: pauses torque on all 6 motors immediately (legs go
            // limp — this is not a graceful stop). For wire-tangle / short-risk
            // situations where waiting for the next control tick is too slow.
            // No GPIO-controlled relay exists on this build, so this is the
            // strongest stop firmware can issue; it does not cut battery power.
            MotorBus_EStop();
            state = STATE_ESTOP;
            s_estop_needs_power_cycle = true;
            ride_clear();
            HeightCtrl_Freeze();
            Serial.println("MANUAL E-STOP -- all motors paused, legs limp. Power-cycle required to resume.");
            break;
        case 'p':
            Serial.print("roll=");  Serial.print(IMU_GetRoll(), 2);
            Serial.print(" pitch="); Serial.print(IMU_GetPitch(), 2);
            Serial.print(" yaw=");  Serial.print(IMU_GetYaw(), 2);
            Serial.print(" tilt="); Serial.print(GET_TILT() * TILT_SIGN, 2);
            Serial.print(" acc=("); Serial.print(IMU_GetAccelX(), 2);
            Serial.print(",");      Serial.print(IMU_GetAccelY(), 2);
            Serial.print(",");      Serial.print(IMU_GetAccelZ(), 2);
            Serial.print(")");
            Serial.print(" vacc="); Serial.print(IMU_GetVerticalAccel(), 2);
            Serial.print(" vtrim="); Serial.print(HeightCtrl_GetVerticalTrimCm(), 3);
            Serial.print(" spd=");  Serial.print(WheelCtrl_GetAvgSpeedDPS(), 1);
            Serial.print(" state=");
            Serial.println(state == STATE_BALANCE ? "BAL" :
                           state == STATE_ESTOP   ? "ESTOP" : "IDLE");
            break;
        case 't':
            // Spin both wheels forward at +200 dps (sign-corrected) — both should go forward
            WheelCtrl_SetSpeed(200.0, 200.0);
            Serial.println("Wheels -> +200 dps forward (sign-corrected).");
            // Manually poll feedback for 2 s since UpdateFeedback() normally only
            // runs inside STATE_BALANCE — 't' alone would otherwise show stale/0.0 speed.
            for (int i = 0; i < 10; i++) {
                delay(200);
                WheelCtrl_UpdateFeedback();
                Serial.print("  readback avg_speed="); Serial.println(WheelCtrl_GetAvgSpeedDPS(), 1);
            }
            Serial.println("Press 's' to stop.");
            break;
        case '1':
            csv_stream = !csv_stream;
            if (csv_stream) {
                // Machine-readable header -- imulog.py keys on "#HDR,".
                Serial.println("#HDR,t_ms,tilt,tilt_sp,wheel_cmd,avg_speed,speed_filt,slope_offset,side_tilt,h_current,h_diff,vtrim,vacc");
            } else {
                Serial.println("CSV off");
            }
            break;
        case 'M': {
            // Trial marker for the analysis pipeline: emitted into the CSV
            // stream so Python can split one long capture into N trials.
            // Works whether or not CSV is on (harmless comment line either way).
            static uint16_t s_mark_n = 0;
            Serial.print("#MARK,"); Serial.print(++s_mark_n);
            Serial.print(",");      Serial.println(millis());
            break;
        }
        case 'E':
            s_vacc_stream = !s_vacc_stream;
            Serial.println(s_vacc_stream
                ? "vacc stream ON (~10 Hz, any state) -- G1: watch for ~0 at rest, +/- spike on lift/set-down"
                : "vacc stream off");
            break;
        case 'T':
            s_tick_timing_enabled = !s_tick_timing_enabled;
            s_tick_dur_min = 0xFFFFFFFFUL; s_tick_dur_max = 0; s_tick_dur_sum = 0; s_tick_dur_n = 0;
            Serial.println(s_tick_timing_enabled
                ? "tick timing probe ON -- prints min/avg/max control_tick() us every ~1s"
                : "tick timing probe off");
            break;
        case 'I':
            s_imu_rate_probe_enabled = !s_imu_rate_probe_enabled;
            s_imu_probe_zero = s_imu_probe_one = s_imu_probe_multi = s_imu_probe_ticks = 0;
            Serial.println(s_imu_rate_probe_enabled
                ? "IMU packet-rate probe ON -- prints packets/tick histogram every ~1s"
                : "IMU packet-rate probe off");
            break;
        case 'H':
            s_hip_track_enabled = !s_hip_track_enabled;
            Serial.println(s_hip_track_enabled
                ? "hip tracking probe ON -- streams cmd/encoder/error for ID1+ID3 at 50Hz during/after ramps"
                : "hip tracking probe off");
            break;
        case 'F':
            s_pitch_capture_active = true;
            s_pitch_capture_n      = 0;
            s_pitch_capture_sum    = 0.0f;
            Serial.println("pitch capture started (~1s, works in IDLE or BALANCE) -- hold/ride steady, result prints when done");
            break;
        case 'V':
            if (s_offset_cal_active) {
                s_offset_cal_active = false;
                Serial.println("offset cal: cancelled.");
                break;
            }
            if (state != STATE_BALANCE) {
                Serial.println("offset cal: must be in BALANCE first ('b'), at 'h' stance, nothing else active.");
                break;
            }
            if (s_side_tilt_enabled) {
                Serial.println("offset cal: refused -- disable 'y' first (need the NATURAL uncorrected lean, not a loop-driven one).");
                break;
            }
            if (fabsf(HeightCtrl_GetDiffCm()) > 0.1f) {
                // A leaned diff physically tilts the body, so IMU_GetPitch() would
                // measure the commanded lean, not the natural offset. Legs must be level.
                Serial.print("offset cal: refused -- legs leaned (diff=");
                Serial.print(HeightCtrl_GetDiffCm(), 2);
                Serial.println(" cm). Level them first ('/'), then retry 'V'.");
                break;
            }
            arm_offset_cal();
            Serial.println("offset cal: started -- waiting for balance to settle, then 15s hold. Press 'V' again to cancel.");
            break;
        case 'h': {
            float t1, t2, hipL, kneeL, hipR, kneeR;
            if (!LegIK_FromXHCm(LEG_STANCE_X_CM, STANCE_HEIGHT_CM, t1, t2)) {
                Serial.println("home: stance IK failed");
                break;
            }
            Calib_RawFromModel(LEG_LEFT,  t1, t2, hipL, kneeL);
            Calib_RawFromModel(LEG_RIGHT, t1, t2, hipR, kneeR);
            MotorBus_SetLegAngle(1, hipL,  80.0);
            MotorBus_SetLegAngle(2, kneeL, 80.0);
            MotorBus_SetLegAngle(3, hipR,  80.0);
            MotorBus_SetLegAngle(4, kneeR, 80.0);
            HeightCtrl_Init(STANCE_HEIGHT_CM);  // re-sync ramp to the stance
            Serial.print("Legs -> standing stance 14.94 cm (L:");
            Serial.print(hipL, 2);  Serial.print("/");  Serial.print(kneeL, 2);
            Serial.print("  R:");   Serial.print(hipR, 2);
            Serial.print("/");      Serial.print(kneeR, 2);  Serial.println(")");
            break;
        }
        case 'f':
            MotorBus_FreeLegMotors();
            Serial.println("Legs free — move by hand. Press 'a' to read angles, 'h' to return to nominal.");
            break;
        case 'a':
            MotorBus_PrintLegAngles();
            break;
        case 'm':
            MotorBus_DiagMotors();
            break;
        case 'n':
            MotorBus_ScanIDs();
            break;
        case 'q':
            // Free right side (ID3+ID4) for manual positioning / calibration
            MotorBus_RightSideFree();
            Serial.println("ID3+ID4 free — move right legs by hand. Press 'w' to read angles.");
            break;
        case 'w':
            MotorBus_RightSidePrintAngles();
            break;
        case 'v':
            MotorBus_PrintAllTemps();
            break;
        // --- Height control (Stage 1: IDLE on stand; BALANCE only via '2') ---
        case 'u': case 'd': case 'c': case 'o': case 'g': case 'l': {
            if (state == STATE_ESTOP) { Serial.println("height: rejected (ESTOP)"); break; }
            if (state == STATE_BALANCE && !s_height_in_balance) {
                Serial.println("height: rejected in BALANCE (toggle with '2' -- Stage 2 only)");
                break;
            }
            if      (c == 'u') HeightCtrl_StepUp();
            else if (c == 'd') HeightCtrl_StepDown();
            else if (c == 'c') HeightCtrl_Preset_Crouch();
            else if (c == 'o') HeightCtrl_Preset_Nominal();
            else if (c == 'g') HeightCtrl_Preset_Tall();
            else               HeightCtrl_Preset_Extra();   // l
            Serial.print("height target: ");
            Serial.print(HeightCtrl_GetTargetCm(), 2);
            Serial.print(" cm (now ");
            Serial.print(HeightCtrl_GetCurrentCm(), 2);
            Serial.print(" cm, ramping at 2 cm/s)");
            if (c == 'o') Serial.print("  [HOME: leveling lean to 0]");
            Serial.println();
            break;
        }
        case ',': case '.': case '/': {
            // Differential leg height (side lean). ',' = lean LEFT (left leg
            // shorter / right longer), '.' = lean RIGHT, '/' = level.
            if (state == STATE_ESTOP) { Serial.println("lean: rejected (ESTOP)"); break; }
            if (state == STATE_BALANCE && !s_height_in_balance) {
                Serial.println("lean: rejected in BALANCE (toggle with '2')");
                break;
            }
            if      (c == ',') HeightCtrl_DiffStep(+HEIGHT_DIFF_STEP_CM);
            else if (c == '.') HeightCtrl_DiffStep(-HEIGHT_DIFF_STEP_CM);
            else               HeightCtrl_DiffLevel();
            Serial.print("lean: diff(R-L) target ");
            Serial.print(HeightCtrl_GetDiffTargetCm(), 2);
            Serial.print(" cm (now ");
            Serial.print(HeightCtrl_GetDiffCm(), 2);
            Serial.print(", L=");
            Serial.print(HeightCtrl_GetLeftCm(), 2);
            Serial.print(" R=");
            Serial.print(HeightCtrl_GetRightCm(), 2);
            Serial.println(" cm)");
            break;
        }
        case '9': case '0': {
            // Full-split preset: REVERTED 2026-07-09 to the original 07-07 fixed
            // pose (L=14.0/R=17.0, balanced on hardware that day) -- hardcoded
            // rather than tracking the live 'c'/'l' presets, so it stays 14/17
            // even after today's c=14.0/l=18.0 changes (or any future ones).
            // center = mean(14.0,17.0) = 15.5; diff = 3.0, within
            // HEIGHT_DIFF_MAX_CM (7.0). '9' = right high, '0' = left high.
            // '/' or presets restore level.
            static constexpr float SPLIT_LOW_CM  = 14.0f;
            static constexpr float SPLIT_HIGH_CM = 17.0f;
            if (state == STATE_ESTOP) { Serial.println("split: rejected (ESTOP)"); break; }
            if (state == STATE_BALANCE && !s_height_in_balance) {
                Serial.println("split: rejected in BALANCE (toggle with '2')");
                break;
            }
            HeightCtrl_SetTarget(0.5f * (SPLIT_LOW_CM + SPLIT_HIGH_CM));   // 15.5
            HeightCtrl_SetDiff(c == '9' ? +(SPLIT_HIGH_CM - SPLIT_LOW_CM)
                                        : -(SPLIT_HIGH_CM - SPLIT_LOW_CM));
            Serial.print("SPLIT: target L=");
            Serial.print(c == '9' ? SPLIT_LOW_CM : SPLIT_HIGH_CM, 1);
            Serial.print(" R=");
            Serial.print(c == '9' ? SPLIT_HIGH_CM : SPLIT_LOW_CM, 1);
            Serial.println(" cm, ramping ('/' = level)");
            break;
        }
        case 'e': {
            // EXPERIMENTAL: extend to the linkage's true geometric IK limit.
            // Numerically verified 2026-07-09 at LEG_STANCE_X_CM: the five-bar
            // stops being solvable past ~20.0 cm -- NOT 21 as visually estimated.
            // Uses HeightCtrl_SetTargetExperimental, which is the only path that
            // can push past HEIGHT_MAX_CM (18); every other height/lean/split
            // command resets the ceiling back to 18 on its next press.
            // Opened to BALANCE 2026-07-09 (IDLE-tested OK) behind the same
            // Stage-2 gate as height/lean/split. Still near-singular at 20 cm
            // (high joint torque, low margin, no tilt-LUT breakpoint above
            // 18.0 -- feedforward holds flat at the 18.0 value up there) --
            // spotter, watch joint temps/torque, be ready with 's'/'x'.
            if (state == STATE_ESTOP) { Serial.println("extend (e): rejected (ESTOP)"); break; }
            if (state == STATE_BALANCE && !s_height_in_balance) {
                Serial.println("extend (e): rejected in BALANCE (toggle with '2' -- Stage 2 only)");
                break;
            }
            HeightCtrl_SetTargetExperimental(HEIGHT_EXPERIMENTAL_MAX_CM);
            Serial.print("EXPERIMENTAL EXTEND -> ");
            Serial.print(HeightCtrl_GetTargetCm(), 2);
            Serial.println(" cm (IK geometric limit, ramping -- watch joint temps/torque; 'o' or 'h' to return, resets ceiling to 18)");
            break;
        }
        case 'r': case 'k': {
            // Combo: ride + gradual height change. 'r' = forward + rise (-> 'l'
            // tall preset), 'k' = backward + crouch (-> 'c' crouch preset). Ride
            // is a dead-man (repeat/hold to sustain, like arrow A/B); height is a
            // one-shot target that ramps at its own 2 cm/s rate underneath it.
            if (state != STATE_BALANCE) { Serial.println("ride+height: rejected (BALANCE only)"); break; }
            if (!s_height_in_balance)   { Serial.println("ride+height: rejected (toggle '2' -- Stage 2 only)"); break; }
            uint32_t now = millis();
            if (c == 'r') {
                s_ride_target = +RIDE_SPEED_FWD_DPS;
                HeightCtrl_Preset_Extra();   // ramp -> 18.0
            } else {
                s_ride_target = -RIDE_SPEED_BACK_DPS;
                HeightCtrl_Preset_Crouch();  // ramp -> 13.0
            }
            s_ride_deadline = now + RIDE_DEADMAN_MS;
            Serial.print(c == 'r' ? "RIDE FWD + RISE" : "RIDE BACK + CROUCH");
            Serial.print(" -> height target ");
            Serial.print(HeightCtrl_GetTargetCm(), 2);
            Serial.println(" cm (hold key to sustain ride)");
            break;
        }
        case 'i': {
            Serial.print("height: target=");  Serial.print(HeightCtrl_GetTargetCm(), 2);
            Serial.print("  current=");       Serial.print(HeightCtrl_GetCurrentCm(), 2);
            Serial.print("  diff(R-L)=");     Serial.print(HeightCtrl_GetDiffCm(), 2);
            Serial.print(" (L=");             Serial.print(HeightCtrl_GetLeftCm(), 2);
            Serial.print(" R=");              Serial.print(HeightCtrl_GetRightCm(), 2);
            Serial.print(")");
            Serial.print(HeightCtrl_IsRamping() ? "  RAMPING" : "  holding");
            // Live encoder cross-check: raw -> calib -> FK per side
            MotorBus_RequestLegAngles();
            float t1, t2;
            Calib_ModelFromRaw(LEG_LEFT,
                               (float)MotorBus_GetLegAngle(1),
                               (float)MotorBus_GetLegAngle(2), t1, t2);
            Serial.print("  |  FK L=");
            Serial.print(MotorBus_LegReplied(1) && MotorBus_LegReplied(2)
                         ? LegFK_HeightCm(t1, t2) : -1.0f, 2);
            Calib_ModelFromRaw(LEG_RIGHT,
                               (float)MotorBus_GetLegAngle(3),
                               (float)MotorBus_GetLegAngle(4), t1, t2);
            Serial.print("  R=");
            Serial.print(MotorBus_LegReplied(3) && MotorBus_LegReplied(4)
                         ? LegFK_HeightCm(t1, t2) : -1.0f, 2);
            Serial.println(" cm");
            Serial.print("  tilt-LUT (h:offset):");
            for (int k = 0; k < TILT_LUT_N; k++) {
                Serial.print(' ');  Serial.print(TILT_LUT_H[k], 1);
                Serial.print(':');  Serial.print(TILT_LUT_O[k], 2);
            }
            Serial.println();
            Serial.print("  slope feedforward: ");
            Serial.print(s_slope_ff_enabled ? "ENABLED" : "disabled");
            Serial.print("  offset=");
            Serial.print(s_slope_offset_deg, 3);
            Serial.println(" deg");
            break;
        }
        case '[': case ']': {
            int idx = tilt_lut_nearest(HeightCtrl_GetCurrentCm());
            TILT_LUT_O[idx] += (c == ']') ? +TILT_TRIM_STEP : -TILT_TRIM_STEP;
            // true_eq is what '3' would save right now: the actual equilibrium
            // the robot is holding = feedforward - PID2's converged output. Show
            // it (plus spd, so settling is visible) instead of the raw offset.
            float true_eq = tilt_offset_for_height(HeightCtrl_GetCurrentCm())
                            - pid2.GetLastOutput();
            Serial.print("tilt trim @ h=");   Serial.print(TILT_LUT_H[idx], 2);
            Serial.print(" cm -> true_eq ");  Serial.print(true_eq, 3);
            Serial.print(" deg (offset ");    Serial.print(TILT_LUT_O[idx], 2);
            Serial.print(", spd ");           Serial.print(WheelCtrl_GetAvgSpeedDPS(), 1);
            Serial.println(" dps)");
            break;
        }
        case '3': {
            // Save the true equilibrium tilt for the current height. The setpoint
            // the robot is actually holding is (feedforward - PID2's output); PID2's
            // integrator has already converged to the residual, so this captures the
            // real equilibrium even if the '['/']' trim was imperfect. Baking it into
            // the table and resetting PID2 is a bumpless handoff — the feedforward now
            // supplies exactly what the integrator was, so the setpoint doesn't move.
            if (state != STATE_BALANCE) {
                Serial.println("save (3): only while balancing ('b' first)");
                break;
            }
            if (HeightCtrl_IsRamping()) {
                Serial.println("save (3): still ramping -- wait for arrival, let it settle");
                break;
            }
            if (fabsf(s_ride_v) > 0.5f || fabsf(s_turn_v) > 0.5f) {
                Serial.println("save (3): ride active -- release arrows, wait a second");
                break;
            }
            float h   = HeightCtrl_GetCurrentCm();
            int   idx = tilt_lut_nearest(h);
            if (fabsf(h - TILT_LUT_H[idx]) > 0.25f) {
                Serial.print("save (3): height "); Serial.print(h, 2);
                Serial.print(" is between breakpoints (nearest ");
                Serial.print(TILT_LUT_H[idx], 2);
                Serial.println(") -- use presets c/o/g/l to capture");
                break;
            }
            float eq  = tilt_offset_for_height(h) - pid2.GetLastOutput();
            TILT_LUT_O[idx] = eq;
            pid2.Reset();
            Serial.print("SAVED tilt LUT[");    Serial.print(TILT_LUT_H[idx], 2);
            Serial.print("cm] = ");             Serial.print(eq, 3);
            Serial.print(" deg  (measured at "); Serial.print(h, 2);
            Serial.print("cm, avg_speed ");     Serial.print(WheelCtrl_GetAvgSpeedDPS(), 1);
            Serial.println(" dps -- near 0 = well settled)");
            break;
        }
        // Old commands '4' (temp-poll toggle), '5' (free knees), '6'/'7'
        // (hold-mode A/B), '8' (old-stance A/B) deleted 2026-07-12 with user
        // approval -- all were one-off diagnostics for the 07-07 jitter/pose
        // investigation, which is settled (pose was the cause; x=-0.616
        // adopted). Temp polling is now unconditionally ON ('4' only existed
        // to disable protection during bus-silence diagnosis). Recover any of
        // them from git history if a future investigation needs them.
        // '4' was then reused (Task 4) as the posture-in-balance gate below.
        case '4':
            s_posture_in_balance = !s_posture_in_balance;
            Serial.print("posture-in-BALANCE ('4'): ");
            Serial.println(s_posture_in_balance
                ? "ENABLED -- posture loops allowed in BALANCE: side-tilt ('y') and/or vertical-CoM ('j'), whichever is armed (G4/G4b: spotter + tether!)"
                : "disabled (posture loops IDLE-only again)");
            break;
        case '2':
            s_height_in_balance = !s_height_in_balance;
            Serial.print("height-in-BALANCE: ");
            Serial.println(s_height_in_balance
                           ? "ENABLED (Stage 2 -- small steps, spotter!)" : "disabled");
            break;
        case 'G':
            s_id3_probe_enabled = !s_id3_probe_enabled;
            Serial.print("ID3 angle/speed/current probe ('G'): ");
            if (s_id3_probe_enabled) {
                Serial.println("ENABLED (50Hz)");
                Serial.println("#HDR3,t_ms,angle_deg,speed_dps,current_a");
            } else {
                Serial.println("disabled");
            }
            break;
        case 'z':
            s_slope_ff_enabled = !s_slope_ff_enabled;
            s_slope_offset_deg = 0.0f;  // start every arm/disarm from zero
            Serial.print("slope feedforward: ");
            Serial.println(s_slope_ff_enabled
                           ? "ENABLED (Stage 3 -- ramp riding, spotter!)" : "disabled");
            break;
        case 'y':
            s_side_tilt_enabled = !s_side_tilt_enabled;
            side_pid.Reset();
            if (s_side_tilt_enabled) {
                s_side_tilt_capturing   = true;
                s_side_tilt_capture_n   = 0;
                s_side_tilt_capture_sum = 0.0f;
                Serial.println("side-tilt (roll) closed loop: ENABLED (IDLE always; BALANCE only if '4' also on -- spotter+tether! Kp=1.5/Ki=0.3, +/-3.5cm)");
            } else {
                s_side_tilt_capturing = false;
                HeightCtrl_DiffLevel();  // don't leave the legs stuck in the last commanded lean
                Serial.println("side-tilt (roll) closed loop: disabled -- leveling diff back to 0");
            }
            break;
        case 'j':
            s_com_ctrl_enabled = !s_com_ctrl_enabled;
            Serial.print("vertical CoM regulator: ");
            Serial.println(s_com_ctrl_enabled
                ? "ENABLED (KA=0/KV=2.0 slow+gated, +/-1.0cm; IDLE always, BALANCE only if '4' on)"
                : "disabled -- trim releasing back to 0");
            break;
        default:
            break;
    }
}

// Arrow keys arrive as ESC [ A/B/C/D. Each press (and each OS auto-repeat while
// held) refreshes a dead-man window; expiry coasts the target back to zero.
static void handle_arrow(char c) {
    if (state != STATE_BALANCE) return;  // ride only while balancing
    uint32_t now = millis();
    switch (c) {
        case 'A': s_ride_target = +RIDE_SPEED_FWD_DPS;  s_ride_deadline = now + RIDE_DEADMAN_MS; break; // up   (fwd)
        case 'B': s_ride_target = -RIDE_SPEED_BACK_DPS; s_ride_deadline = now + RIDE_DEADMAN_MS; break; // down (back)
        case 'C': s_turn_target = +TURN_DPS;       s_turn_deadline = now + RIDE_DEADMAN_MS; break; // right
        case 'D': s_turn_target = -TURN_DPS;       s_turn_deadline = now + RIDE_DEADMAN_MS; break; // left
        default: break;
    }
}

void setup() {
    Serial.begin(115200);
    while (!Serial && millis() < 3000) {}
    print_commands();

    IMU_Init();
    LegFK_Init();
    MotorBus_Init(true);   // all 6 motors: legs + wheels
    WheelCtrl_Init(true);

    // Soft engage: read current angles and command them first so motors
    // lock on without jerking, then move slowly to standing stance.
    // Guard: only engage motors that actually replied — GetLegAngle defaults to
    // 0.0 for a silent motor, and commanding 0 deg is a violent full-force swing.
    MotorBus_PrintLegAngles();  // populates motor_angle_multi on ID1-4
    for (int id = 1; id <= 4; id++) {
        if (!MotorBus_LegReplied(id)) {
            Serial.print("WARNING: leg ID"); Serial.print(id);
            Serial.println(" gave no encoder reply — NOT engaging it (check RS485/power).");
            continue;
        }
        MotorBus_SetLegAngle(id, MotorBus_GetLegAngle(id), 30.0);
    }
    delay(300);  // let motors engage at current position

    // Stance from IK through the live calibration (replaces the old hardcoded
    // raw angles, which were frozen in the pre-rezero ID2 frame).
    float st1, st2, s_hipL, s_kneeL, s_hipR, s_kneeR;
    if (LegIK_FromXHCm(LEG_STANCE_X_CM, STANCE_HEIGHT_CM, st1, st2)) {
        Calib_RawFromModel(LEG_LEFT,  st1, st2, s_hipL, s_kneeL);
        Calib_RawFromModel(LEG_RIGHT, st1, st2, s_hipR, s_kneeR);
        const double stance[4][2] = { {1, s_hipL}, {2, s_kneeL}, {3, s_hipR}, {4, s_kneeR} };
        for (int i = 0; i < 4; i++) {
            int id = (int)stance[i][0];
            if (MotorBus_LegReplied(id)) MotorBus_SetLegAngle(id, stance[i][1], 40.0);
        }
    } else {
        Serial.println("WARNING: stance IK failed — legs hold soft-engage pose.");
    }

    HeightCtrl_Init(STANCE_HEIGHT_CM);  // nominal = 14.94

    state = STATE_IDLE;
    controlTimer.begin(onTick, 5000);  // 5000 µs = 200 Hz
    Serial.println("Ready.");
}

static void print_commands() {
    Serial.println("=== iRobot Phase B ===");
    Serial.println("b=balance (clears tilt/IMU estop if <5 deg; temp-estop needs power cycle)  s=idle  p=print  1=csv");
    Serial.println("x=MANUAL E-STOP (legs go limp -- power-cycle required to resume)");
    Serial.println("RIDE (hold, BALANCE only): up/down=fwd/back  left/right=turn");
    Serial.println("COMBO (hold, BALANCE + Stage2 '2' only): r=ride fwd + rise to 18.0  k=ride back + crouch to 14.0");
    Serial.println("HEIGHT (IDLE): u/d=+-0.5cm  c=14.0  o=HOME(14.94 + levels lean)  g=16.5  l=18.0  i=status  2=allow-in-balance");
    Serial.println("EXPERIMENTAL: e=extend to IK limit ~20.0cm -- near-singular, no tilt-LUT past 18.0 (same gates as HEIGHT)");
    Serial.println("LEAN: ,=left  .=right (+-0.25cm/press, max 7.0)  /=level  9/0=full split R/L-high  (same gates as HEIGHT)");
    Serial.println("TILT TRIM (BALANCE): [ / ] = -/+ 0.05 deg (prints live true_eq + spd)  3=save equilibrium here  i=show table");
    Serial.println("RAMP (BALANCE, EXPERIMENTAL): z=toggle slope feedforward (Stage 3 -- off by default, spotter!)  i=show offset");
    Serial.println("SIDE-TILT (IDLE only, EXPERIMENTAL): y=toggle closed-loop roll-leveling via diff-height (Kp=0.3 P-only, +/-2.0cm clamp)");
    Serial.println("VERTICAL CoM (EXPERIMENTAL): j=toggle vertical-accel trim (KA=0/KV=2.0, gated, +/-1.0cm)  4=allow-in-BALANCE (G4: spotter!)");
    Serial.println("BENCH: G=toggle ID3 angle/speed/current probe (50Hz CSV, #HDR3 on enable) -- for live-graphing while tuning on the LK tool");
    Serial.println("BENCH: E=toggle vacc stream (~10 Hz, any state, hands-free -- for the G1 vertical-accel sanity check)");
    Serial.println("BENCH: F=capture avg pitch over ~1s (IDLE or BALANCE, read-only) -- compare load-dependent side-tilt offset");
    Serial.println("BENCH: pitch-offset cal auto-runs ~2s after 'b' (15s hold, needs y off + legs level). V=re-run manually / cancel.");
    Serial.println("PROBE: T=control_tick() min/avg/max us every ~1s (read-only)  I=IMU packets/tick histogram every ~1s (read-only)");
    Serial.println("PROBE: H=hip ID1/ID3 cmd-vs-encoder tracking stream at 50Hz, during+after ramps (read-only, adds bus traffic)");
    Serial.println("DATA: 1=CSV v2 stream (BALANCE, #HDR on enable)  M=trial marker (#MARK,n,ms -- splits trials in analysis)");
    Serial.println("h=home  f=free_L  a=angles_L  q=free_R  w=angles_R");
    Serial.println("t=test_wheels  m=diag  n=scan  v=temps");
}

void loop() {
    // Reprint commands whenever the serial monitor is (re)opened
    static bool s_serial_prev = false;
    bool s_serial_now = (bool)Serial;
    if (s_serial_now && !s_serial_prev) {
        print_commands();
    }
    s_serial_prev = s_serial_now;

    while (Serial.available()) {
        char c = (char)Serial.read();
        // Escape-sequence parser for arrow keys (ESC [ A/B/C/D).
        // A stray/lone ESC must NOT swallow the next real command byte (e.g. 's'
        // arriving after an accidental Escape keypress) — bug found 2026-07-03:
        // the old version dropped that byte via a bare `continue`. A timeout also
        // stops a lone ESC from latching indefinitely waiting for a '[' that may
        // never come.
        static uint8_t  esc    = 0;  // 0=idle, 1=got ESC, 2=got ESC[
        static uint32_t esc_t0 = 0;
        if (esc != 0 && (millis() - esc_t0) > 50) esc = 0;  // abandon stale sequence

        if (esc == 0 && c == 0x1B) { esc = 1; esc_t0 = millis(); continue; }
        if (esc == 2) { esc = 0; handle_arrow(c); continue; }
        if (esc == 1) {
            if (c == '[') { esc = 2; continue; }
            esc = 0;  // not an arrow sequence — fall through, process c normally
        }
        Serial.print("["); Serial.print(c); Serial.print("]");
        handle_serial_cmd(c);
    }

    // Heartbeat every 2 s
    static uint32_t last_hb = 0;
    if (millis() - last_hb > 1000) {
        last_hb = millis();
        Serial.print("tick="); Serial.print(tick_count);
        Serial.print(" tilt="); Serial.print(GET_TILT() * TILT_SIGN, 2);
        Serial.print(" spd="); Serial.print(WheelCtrl_GetAvgSpeedDPS(), 1);
        Serial.print(" maxT="); Serial.print((int)MotorBus_GetMaxTempC()); Serial.println("C");
    }

    if (tick_flag) {
        tick_flag = false;
        if (s_tick_timing_enabled) {
            uint32_t t0 = micros();
            control_tick();
            uint32_t dt = micros() - t0;
            if (dt < s_tick_dur_min) s_tick_dur_min = dt;
            if (dt > s_tick_dur_max) s_tick_dur_max = dt;
            s_tick_dur_sum += dt;
            if (++s_tick_dur_n >= TICK_TIMING_WINDOW) {
                Serial.print("#PROBE,tick_us (of "); Serial.print(s_tick_dur_n);
                Serial.print("): min="); Serial.print(s_tick_dur_min);
                Serial.print(" avg="); Serial.print((float)s_tick_dur_sum / s_tick_dur_n, 1);
                Serial.print(" max="); Serial.println(s_tick_dur_max);
                s_tick_dur_min = 0xFFFFFFFFUL; s_tick_dur_max = 0; s_tick_dur_sum = 0; s_tick_dur_n = 0;
            }
        } else {
            control_tick();
        }
    }
}
