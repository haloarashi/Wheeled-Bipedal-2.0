#include "legs/height_ctrl.h"
#include <math.h>
#include "legs/leg_ik.h"
#include "leg_calib.h"
#include "motor_bus.h"

// Ramp: trapezoidal velocity profile (2026-07-07, was constant-rate). The old
// ramp stepped velocity 0 -> 2 cm/s instantly at start and back at arrival;
// those velocity steps are jerk kicks the balance loop must absorb (observed:
// wobble at every transition). Same 2 cm/s cruise, but accelerate over ~0.3 s
// and decelerate into the target (v = sqrt(2*a*d_remaining)) -> smooth ends.
// Joint speed at cruise is ~13 dps worst case (~6.3 deg/cm), well under the
// 60 dps per-command cap.
static constexpr float RAMP_MAX_V = 17.0f / 200.0f;             // cm per tick (... -> 5.0 -> 17.0 cm/s,
                                                                // 2026-07-15, user-directed). PURPOSE: after
                                                                // re-tuning ID1/ID3 internal gains on the LK
                                                                // tool, the user found sub-1-degree commanded
                                                                // steps don't close accurately but >=1 degree
                                                                // steps do. At 50Hz (SEND_EVERY_N_TICKS=4,
                                                                // 20ms window) and the slow joint (knee, ~3
                                                                // deg/cm), 17 cm/s gives a knee step of
                                                                // 17*0.02*3 ~= 1.02 deg -- just clears the 1-deg
                                                                // floor. Hip (~6.3 deg/cm) steps ~2.14 deg,
                                                                // needing ~107 dps, under the raised 150 cap.
                                                                // CAVEAT: RAMP_ACCEL reaches cruise in 0.3s, but
                                                                // a typical 3cm move takes only ~0.18s, so it
                                                                // BARELY reaches 17 cm/s mid-move -- during the
                                                                // accel/decel portions steps are <1 deg. If the
                                                                // 1-deg guarantee needs to hold across the whole
                                                                // ramp, RAMP_ACCEL must also be raised (faster
                                                                // time-to-cruise). knee deg/cm is from ONE
                                                                // transition's kinematics + position-dependent.
static constexpr float RAMP_ACCEL = RAMP_MAX_V / (0.1f * 200.0f); // reach cruise in 0.1 s (was 0.3s), 2026-07-15
                                                                 // user-directed: the 0.3s accel meant a ~3cm
                                                                 // move never reached the 17 cm/s cruise, so its
                                                                 // decel tail sat below the 1-deg/step threshold
                                                                 // and oscillated (confirmed on hardware -- mid-
                                                                 // move clean, end oscillated). 0.1s compresses
                                                                 // the accel/decel distance (~0.85cm each vs
                                                                 // ~2.5cm) so a real cruise segment exists and the
                                                                 // sub-1-deg region shrinks toward the target
                                                                 // (where the one-shot arrival send lands the
                                                                 // exact final pose). TRADEOFF: RAMP_ACCEL is also
                                                                 // the per-tick jerk limit the trapezoid added to
                                                                 // kill velocity-step wobble -- 3x faster = 3x the
                                                                 // start/end jerk. If jerk wobble reappears at the
                                                                 // ramp START, back off toward 0.15-0.2s.
// End-of-ramp early-snap distance (2026-07-15). The decel-into-target tail
// of the trapezoid has instantaneous speed < RAMP_MAX_V by construction --
// meaning every step in that tail is BELOW the 1-deg/step floor RAMP_MAX_V
// was tuned to guarantee (see its comment), regardless of how fast RAMP_ACCEL
// is. Raising RAMP_ACCEL only shortens the tail, it can't remove it (root
// cause: the tail's whole DEFINITION is "speed below the 1-deg threshold").
// Root fix: stop ramping once the remaining distance is small enough that
// the rest of the move would be sub-1-deg anyway, and hand it to the
// existing one-shot arrival send instead -- ONE clean, fast, single move at
// LEG_CMD_SPEED_DPS to the exact final target, the same reason full-degree
// ramp steps track cleanly. Derived from RAMP_MAX_V/RAMP_ACCEL (the same
// constants that define the threshold and the tail shape), not a separate
// magic number -- stays correct if either is retuned. v^2 = 2*a*d -> d = v^2/2a.
static constexpr float RAMP_TAIL_DISTANCE_CM = (RAMP_MAX_V * RAMP_MAX_V) / (2.0f * RAMP_ACCEL);
static constexpr float LEG_CMD_SPEED_DPS = 150.0f;  // 60 -> 150, 2026-07-15: raised so the hip (~107 dps at
                                                    // the new 17 cm/s ramp) isn't clamped. Not a hardware
                                                    // limit (jump-rated, hundreds of dps); == TRIM_CMD_SPEED_DPS
                                                    // now. In matched-speed this is the per-motor speed CEILING,
                                                    // not the commanded value -- steps that need less still get
                                                    // less.
// MATCHED arrival (2026-07-15, replaces the flat ARRIVAL_CMD_SPEED_DPS).
// History: the one-shot arrival send (the last ~0.85cm, RAMP_TAIL_DISTANCE_CM)
// used to be flat-speed -- at a flat cap, hip (~5.4deg remaining) and knee
// (~2.5deg) finish at DIFFERENT times, and during that asynchrony window the
// wheel-mount x-position transiently shifts; since the balance loop is
// velocity-only (pid2), it can't correct that shift back to zero, leaving an
// occasional small post-ramp drift -- worse the gentler/slower the flat cap
// was (confirmed on hardware: drift appeared after softening 150->80->70).
// Fix: make the arrival MATCHED too -- per-joint speed proportioned to each
// joint's own remaining distance so hip and knee land TOGETHER at a chosen
// duration (same step_speed_dps() math as the periodic ramp send, just fed
// ARRIVAL_DURATION_S instead of SEND_INTERVAL_S). This removes the
// asynchrony window entirely while keeping the soft landing -- no more
// trading gentleness against drift.
static constexpr float ARRIVAL_DURATION_S = 0.08f;  // TUNE-ON-BENCH: target landing time for
                                                    // BOTH hip and knee (was ~77ms at flat 70dps
                                                    // for the hip alone -- similar feel, now
                                                    // synchronized). Lower = faster/firmer,
                                                    // higher = gentler/slower.
static constexpr float ARRIVAL_SPEED_CEILING_DPS = 150.0f;  // safety cap only -- normal tails
                                                            // (hip ~5.4deg/0.08s=~68dps, knee
                                                            // ~2.5deg/0.08s=~31dps) sit well
                                                            // under this; it just bounds an
                                                            // unusually large residual.
static constexpr float ARRIVAL_MARGIN = 1.0f;  // no headroom scaling -- unlike the periodic
                                               // send's margin (needed because insufficient
                                               // margin compounds drift over MANY repeated
                                               // cycles), this is a single one-shot move with
                                               // nothing to accumulate over. Raise slightly
                                               // (e.g. 1.1-1.2) only if bench testing shows the
                                               // arrival doesn't fully land within
                                               // ARRIVAL_DURATION_S (real motor accel lag).
// Task 4: trim-driven sends get a higher motor speed cap than ramp sends --
// disturbance absorption is only useful if the legs beat the disturbance,
// and 60 dps (~0.2 s for a 1 cm trim) eats most of the reaction budget.
// Hardware is jump-rated (hundreds of dps); watch temps ('v') during G4.
static constexpr float TRIM_CMD_SPEED_DPS = 150.0f;
static constexpr int   SEND_EVERY_N_TICKS = 4;   // 50 Hz (20ms window). 2026-07-15, user-directed. Paired
                                                 // with RAMP_MAX_V=17 cm/s + LEG_CMD_SPEED_DPS=150 to
                                                 // guarantee the knee step clears ~1 deg/window -- the
                                                 // accuracy floor found after the LK-tool ID1/ID3 gain
                                                 // re-tune (sub-1-deg steps don't close, >=1-deg do). Lower
                                                 // frequency = fewer, larger steps at a fixed cruise speed;
                                                 // combined with the higher speed here, each step is well
                                                 // above 1 deg (except the accel/decel tails -- see
                                                 // RAMP_MAX_V caveat). Spotter required, watch both
                                                 // directions.

// Differential slew: 2x RAMP_MAX_V (2026-07-16, user-directed test -- was 1.15
// cm/s). The 2x is deliberate, NOT an error: send_height() splits diff +-half
// per leg (hL = h - diff/2, hR = h + diff/2), so a given d(diff)/dt moves each
// leg at HALF that rate. 2*RAMP_MAX_V makes each leg's physical speed equal to
// a height ramp's per-leg speed (17 cm/s) -- which is what keeps the knee step
// per 20ms window at/above the ~1-deg accuracy floor (a 1x setting would put it
// at ~0.5 deg, in the sub-1-deg "doesn't close" zone found after the ID1/ID3
// retune). Still a plain rate limit, NOT the height trapezoid: the flat limiter
// now has a real start/end velocity STEP (jerk) at this speed -- if the bench
// shows start/stop wobble on a split, that's the signal to add a diff
// trapezoid. If it shows net creep after settling, that's the station-keeping
// loop's job, not the ramp shape.
static constexpr float DIFF_SLEW_PER_TICK = 3.0f * RAMP_MAX_V;  // 1.15/200 -> 2x -> 3x RAMP_MAX_V (2026-07-16,
                                                               // user: "too sluggish, don't worry about jerk").
                                                               // 3x -> 25.5 cm/s per leg; knee step ~1.5 deg/window,
                                                               // clear of the 1-deg floor. Speeds up BOTH manual
                                                               // splits and side-tilt leveling (same channel).
// Diff trapezoid (2026-07-16): gives the diff channel the same accel/decel +
// early-snap treatment as height, mirroring RAMP_ACCEL / RAMP_TAIL_DISTANCE_CM.
// Motivation: at the raised DIFF_SLEW the old plain rate limiter had a hard
// velocity stop at arrival (17 cm/s/leg -> 0 in one tick) = a jerk kick that
// produced a small end-of-split oscillation. The decel ramp removes the hard
// stop; the early-snap hands the sub-1-deg decel tail to a matched arrival
// (same reason height needs it). NO glide analogue: tilt_sp never reads diff,
// so there is no LUT-timing mismatch to hide -- diff snaps to target the
// instant it enters the tail.
static constexpr float DIFF_ACCEL = DIFF_SLEW_PER_TICK / (0.1f * 200.0f);  // reach cruise in 0.1 s (mirrors RAMP_ACCEL)
static constexpr float DIFF_TAIL_DISTANCE_CM = (DIFF_SLEW_PER_TICK * DIFF_SLEW_PER_TICK) / (2.0f * DIFF_ACCEL);

static float s_target  = 0.0f;
static float s_current = 0.0f;
static float s_nominal = 0.0f;
static float s_vel     = 0.0f;   // ramp velocity, cm per tick (signed)
static float s_diff    = 0.0f;   // current diff (h_R - h_L), cm
static float s_diff_t  = 0.0f;   // diff target
static float s_diff_vel = 0.0f;  // diff trapezoid velocity, cm per tick (signed)
static bool  s_inited  = false;
static int   s_tick_ct = 0;

// Arrival glide state (2026-07-15). The old behavior snapped s_current to
// s_target INSTANTLY at the tail threshold, then fired a one-shot arrival
// send that took ARRIVAL_DURATION_S to physically complete -- meaning
// tilt_offset_for_height(GetCurrentCm()) (the LUT equilibrium lookup) jumped
// to the FINAL height's value immediately, ~80ms before the real body got
// there. That model/reality mismatch is a real, if occasional, disturbance
// seed (confirmed on hardware: rare l->o post-arrival oscillation, pid2
// pinned at its +5 clamp -- the same cascade mechanism from prior incidents).
// Fix: glide s_current toward s_target over the SAME ARRIVAL_DURATION_S the
// physical arrival send actually takes, instead of jumping instantly -- the
// LUT lookup now tracks where the body really is.
static bool  s_arrival_active     = false;
static float s_arrival_start_cm   = 0.0f;
static int   s_arrival_ticks_left = 0;
static constexpr int ARRIVAL_DURATION_TICKS = (int)(0.5f + ARRIVAL_DURATION_S * 200.0f);

// Matched-speed sends (2026-07-13): diagnosed with the user that periodic
// ramp sends were "sprint then park" -- a 20 ms window's real step (e.g.
// ~0.38 deg at 3 cm/s cruise, using the ~6.3 deg/cm figure above) takes only
// ~6 ms at the flat 60 dps cap, so the motor sits frozen for the other
// ~14 ms, then jerks again next send. Four stiff servos doing this in sync
// at 50 Hz excites the compliant chassis (the 07-07 coupled-pair finding).
// Fix: track each motor's last-SENT angle and request only the speed THIS
// step actually needs (with headroom), capped at the existing flat value as
// a ceiling -- never faster than before, just not needlessly fast. Only
// applied to the periodic/ramping send; the one-shot "arrival" send (a
// single clean move to the exact final target, not a repeated small step)
// keeps its original flat-speed behavior unchanged.
static float s_last_hipL = 0.0f, s_last_kneeL = 0.0f;
static float s_last_hipR = 0.0f, s_last_kneeR = 0.0f;
static bool  s_have_last = false;
static constexpr float SEND_INTERVAL_S    = SEND_EVERY_N_TICKS / 200.0f; // 0.02 s
// Margin search history (all at 50 Hz, o->l only): 1.4->1.2->1.4->3.0->2.5->
// 2.0->1.7->2.0(closed). Full result table: 3.0=clean, 2.5=clean, 2.0=CLEAN
// (confirmed), 1.7=slight jitter + slight drift, 1.2-1.4=drift (worse, ran
// away to a fall). 2.0 is the lowest o->l-CONFIRMED-clean value. The 1.7
// result (small/bounded, not runaway) shows the effect is GRADED with
// margin, not a cliff edge -- trading jitter against drift (more margin =
// less drift, more jitter). Cause: too little margin means the motor never
// fully catches up to the moving target each send window, so a sustained
// hip-vs-knee tracking-lag difference accumulates into drift instead of
// being corrected every cycle (hip/knee rotate OPPOSITE directions for
// height changes, so any relative lag shows as x-drift).
//
// 2026-07-14: EVERY margin value above was only ever tested on o->l -- l->o
// was never separately validated at any margin (only at margin=2.0+100Hz,
// which failed for a DIFFERENT, frequency-linked reason, see
// SEND_EVERY_N_TICKS). User's hypothesis: o->l fights gravity (extending),
// l->o is assisted by it (contracting) -- the two directions may have
// genuinely different motor tracking dynamics and may need different
// margins. Split into direction-specific constants, chosen by the height
// ramp's sign (s_vel) in HeightCtrl_Tick, NOT per-joint sign (hip/knee
// counter-rotate for the same height change, so per-joint sign would pick
// the wrong constant for one of the two motors every time).
static constexpr float STEP_SPEED_MARGIN_EXTEND   = 3.0f;  // 2.0->4.0->2.0->1.4->2.0->3.0, 2026-07-15,
                                                             // user-directed. RESULT at 2.0 (100Hz, post bus-
                                                             // split + reduced ACK-drain, T-probe confirmed all
                                                             // ticks under the 5ms budget): l->o STILL spiked to
                                                             // tilt=8.6deg, spd=-583.7dps almost immediately after
                                                             // starting the ramp -- second l->o data point at this
                                                             // margin, both showing real excursions, now with the
                                                             // bus-timing mechanism confirmed NOT at fault (timing
                                                             // was clean when it happened). Used for BOTH o->l and
                                                             // l->o (direction-split still bypassed, matched=true
                                                             // unconditionally, see HeightCtrl_Tick's send-site
                                                             // comment). 3.0 was o->l-clean in the original search;
                                                             // l->o at 3.0 is untested.
static constexpr float STEP_SPEED_MARGIN_CONTRACT = 1.7f;  // NOT CURRENTLY READ anywhere -- direction-split is
                                                             // bypassed (see above). Left in place, inert, for
                                                             // whenever the split is reinstated.
static constexpr float STEP_SPEED_MIN_DPS = 3.0f;  // floor -- avoids a near-zero speed arg on a tiny/no-op step

// interval_s made explicit (2026-07-15, was hardcoded to SEND_INTERVAL_S) so
// this same per-motor-speed math can serve two different callers: the
// periodic ramp send (interval_s=SEND_INTERVAL_S, the repeating send period)
// and the one-shot MATCHED arrival (interval_s=ARRIVAL_DURATION_S, a chosen
// target landing time) -- see send_height().
static float step_speed_dps(float last, float now, float ceiling_dps, float margin, float interval_s) {
    float need = fabsf(now - last) / interval_s * margin;
    if (need < STEP_SPEED_MIN_DPS) return STEP_SPEED_MIN_DPS;
    if (need > ceiling_dps)        return ceiling_dps;
    return need;
}

// Vertical CoM trim (common-mode, cm) -- see header comment. No target/
// current split: the value IS the instantaneous command. s_vtrim_last_sent
// tracks what was last actually transmitted to the motors, so Tick() can
// tell "trim changed since we last sent" apart from "trim is static" without
// re-sending identical commands every tick.
static float s_vtrim           = 0.0f;
static float s_vtrim_last_sent = 0.0f;
static constexpr float VTRIM_SEND_THRESHOLD_CM = 0.02f;

// Runtime ceiling send_height() clamps against. Defaults to the normal
// HEIGHT_MAX_CM; only HeightCtrl_SetTargetExperimental() raises it, and only
// for that one call — every other setter resets it first (see below), so the
// experimental ceiling can never silently persist into a later normal command.
static float s_ceiling_cm = HEIGHT_MAX_CM;

static float clamp_h(float h) {
    return h < HEIGHT_MIN_CM ? HEIGHT_MIN_CM : (h > s_ceiling_cm ? s_ceiling_cm : h);
}

// IK -> calib -> raw commands, per leg at h -/+ diff/2 + vtrim (both at
// stance x). vtrim is common-mode (added to both legs identically) so it
// composes with diff rather than fighting it. Only commands motors that
// answered on the bus (same guard as soft-engage).
//
// matched=true (periodic ramp sends, and now the matched arrival too):
// per-motor speed is capped at what THIS step actually needs, not a flat
// ceiling -- see step_speed_dps(). `interval_s` is the caller's TARGET
// completion time (SEND_INTERVAL_S for periodic sends; ARRIVAL_DURATION_S
// for the one-shot arrival) -- every motor lands at roughly interval_s/margin
// regardless of its own step size, which is what makes hip and knee finish
// TOGETHER instead of at a flat speed's different times per joint.
// matched=false: flat speed_dps for all four (interval_s/margin unused).
// `margin` selects EXTEND vs CONTRACT per the caller's ramp direction for
// periodic sends; the arrival call uses its own margin (see call site).
static void send_height(float h_cm, float diff_cm, float speed_dps, bool matched, float margin, float interval_s) {
    float t1L, t2L, t1R, t2R;
    float hL = clamp_h(h_cm - 0.5f * diff_cm + s_vtrim);
    float hR = clamp_h(h_cm + 0.5f * diff_cm + s_vtrim);
    if (!LegIK_FromXHCm(LEG_STANCE_X_CM, hL, t1L, t2L)) return;  // unreachable: hold last sent
    if (!LegIK_FromXHCm(LEG_STANCE_X_CM, hR, t1R, t2R)) return;

    float hipL, kneeL, hipR, kneeR;
    Calib_RawFromModel(LEG_LEFT,  t1L, t2L, hipL, kneeL);
    Calib_RawFromModel(LEG_RIGHT, t1R, t2R, hipR, kneeR);

    float sHipL = speed_dps, sKneeL = speed_dps, sHipR = speed_dps, sKneeR = speed_dps;
    if (matched && s_have_last) {
        sHipL  = step_speed_dps(s_last_hipL,  hipL,  speed_dps, margin, interval_s);
        sKneeL = step_speed_dps(s_last_kneeL, kneeL, speed_dps, margin, interval_s);
        sHipR  = step_speed_dps(s_last_hipR,  hipR,  speed_dps, margin, interval_s);
        sKneeR = step_speed_dps(s_last_kneeR, kneeR, speed_dps, margin, interval_s);
    }

    if (MotorBus_LegReplied(1)) MotorBus_SetLegAngle(1, hipL,  sHipL);
    if (MotorBus_LegReplied(2)) MotorBus_SetLegAngle(2, kneeL, sKneeL);
    if (MotorBus_LegReplied(3)) MotorBus_SetLegAngle(3, hipR,  sHipR);
    if (MotorBus_LegReplied(4)) MotorBus_SetLegAngle(4, kneeR, sKneeR);

    s_last_hipL = hipL; s_last_kneeL = kneeL;
    s_last_hipR = hipR; s_last_kneeR = kneeR;
    s_have_last = true;
    s_vtrim_last_sent = s_vtrim;  // track what was actually transmitted
}

void HeightCtrl_Init(float h0_cm) {
    s_target     = h0_cm;
    s_current    = h0_cm;
    s_nominal    = h0_cm;
    s_diff       = 0.0f;
    s_diff_t     = 0.0f;
    s_vel        = 0.0f;
    s_diff_vel   = 0.0f;
    s_ceiling_cm = HEIGHT_MAX_CM;
    s_vtrim           = 0.0f;
    s_vtrim_last_sent = 0.0f;
    // Whatever called Init() (boot, or 'h') just positioned the motors
    // directly outside of send_height() -- invalidate the last-sent-angle
    // tracking so the next matched-speed send doesn't diff against a stale
    // reference from before this reset.
    s_have_last  = false;
    // If this fires mid-glide, s_arrival_start_cm/s_arrival_ticks_left are
    // now stale (2026-07-15 fix) -- without this, the NEXT ramp's first tick
    // could wrongly take the "glide continues" branch using garbage state.
    s_arrival_active = false;
    s_inited     = true;
}

bool HeightCtrl_SetDiff(float d_cm) {
    float clamped = d_cm < -HEIGHT_DIFF_MAX_CM ? -HEIGHT_DIFF_MAX_CM
                  : d_cm >  HEIGHT_DIFF_MAX_CM ?  HEIGHT_DIFF_MAX_CM : d_cm;
    s_diff_t = clamped;
    return clamped == d_cm;
}

bool HeightCtrl_DiffStep(float d_cm) { return HeightCtrl_SetDiff(s_diff_t + d_cm); }

void  HeightCtrl_DiffLevel()  { s_diff_t = 0.0f; }

// Last-COMMANDED raw hip angle (degrees, motor frame) -- diagnostic getters
// (2026-07-15) for tracking commanded-vs-actual hip tracking error. Only
// meaningful once s_have_last is true (first send has gone out); before that
// s_last_hipL/R read 0, which a caller comparing against a real encoder
// reading would misread as a huge error. HeightCtrl_HasSentHip() lets the
// caller gate on this explicitly instead of silently printing garbage.
bool  HeightCtrl_HasSentHip()      { return s_have_last; }
float HeightCtrl_GetLastSentHipL() { return s_last_hipL; }
float HeightCtrl_GetLastSentHipR() { return s_last_hipR; }

bool HeightCtrl_SetVerticalTrim(float cm) {
    float clamped = cm < -VTRIM_MAX_CM ? -VTRIM_MAX_CM
                  : cm >  VTRIM_MAX_CM ?  VTRIM_MAX_CM : cm;
    s_vtrim = clamped;
    return clamped == cm;
}
float HeightCtrl_GetVerticalTrimCm() { return s_vtrim; }

float HeightCtrl_GetDiffCm()  { return s_diff; }
float HeightCtrl_GetDiffTargetCm() { return s_diff_t; }
float HeightCtrl_GetLeftCm()  { return s_current - 0.5f * s_diff; }
float HeightCtrl_GetRightCm() { return s_current + 0.5f * s_diff; }

bool HeightCtrl_SetTarget(float h_cm) {
    s_ceiling_cm = HEIGHT_MAX_CM;  // normal path always resets the ceiling
    float clamped = h_cm < HEIGHT_MIN_CM ? HEIGHT_MIN_CM
                  : h_cm > HEIGHT_MAX_CM ? HEIGHT_MAX_CM : h_cm;
    s_target = clamped;
    return clamped == h_cm;
}

bool HeightCtrl_SetTargetExperimental(float h_cm) {
    s_ceiling_cm = HEIGHT_EXPERIMENTAL_MAX_CM;
    float clamped = h_cm < HEIGHT_MIN_CM ? HEIGHT_MIN_CM
                  : h_cm > HEIGHT_EXPERIMENTAL_MAX_CM ? HEIGHT_EXPERIMENTAL_MAX_CM : h_cm;
    s_target = clamped;
    return clamped == h_cm;
}

void HeightCtrl_StepUp()   { HeightCtrl_SetTarget(s_target + HEIGHT_STEP_CM); }
void HeightCtrl_StepDown() { HeightCtrl_SetTarget(s_target - HEIGHT_STEP_CM); }
void HeightCtrl_Preset_Crouch()  { HeightCtrl_SetTarget(HEIGHT_CROUCH_CM); }
void HeightCtrl_Preset_Nominal() { HeightCtrl_SetTarget(s_nominal); HeightCtrl_DiffLevel(); }  // 'o' = TRUE HOME: nominal height AND level lean (clears any split/lean)
void HeightCtrl_Preset_Tall()    { HeightCtrl_SetTarget(HEIGHT_TALL_CM); }
void HeightCtrl_Preset_Extra()   { HeightCtrl_SetTarget(HEIGHT_EXTRA_CM); }

void HeightCtrl_Freeze() {
    s_target = s_current; s_diff_t = s_diff; s_vel = 0.0f;
    s_diff_vel = 0.0f;
    s_arrival_active = false;  // stale glide state must not survive a freeze (2026-07-15 fix)
    // ESTOP/any state change must not preserve a trim -- the vertical-CoM
    // loop is IDLE-only for now (Task 3), so leaving IDLE should zero it.
    // Also zero s_vtrim_last_sent (not just s_vtrim) so this doesn't itself
    // trigger a "trim changed" send next tick -- Freeze() means go quiet,
    // not command one more correction. (ESTOP separately cuts motor torque
    // via MotorBus_EStop(), so the legs go physically limp regardless; this
    // is purely about not leaving stale internal state behind.)
    s_vtrim = 0.0f;
    s_vtrim_last_sent = 0.0f;
}

bool  HeightCtrl_IsRamping()   { return fabsf(s_target - s_current) > 1e-4f
                                     || fabsf(s_diff_t - s_diff)    > 1e-4f; }
float HeightCtrl_GetTargetCm() { return s_target; }
float HeightCtrl_GetCurrentCm(){ return s_current; }

void HeightCtrl_Tick() {
    if (!s_inited) { s_vel = 0.0f; return; }

    bool was_ramping  = HeightCtrl_IsRamping();
    // Trim is a live control-loop output (com_ctrl), not a human setpoint --
    // it can move on ANY tick with no ramp in progress at all. Without this,
    // send_height() (which now folds s_vtrim in) would never fire once
    // height/diff settle, and a vertical-trim command would silently never
    // reach the motors. Threshold avoids re-sending on sub-noise trim wiggle.
    bool trim_pending = fabsf(s_vtrim - s_vtrim_last_sent) > VTRIM_SEND_THRESHOLD_CM;
    if (!was_ramping && !trim_pending) { s_vel = 0.0f; return; }

    if (was_ramping) {
        if (s_arrival_active) {
            // Glide s_current toward s_target over the SAME duration the
            // physical one-shot arrival send (fired below, once, at glide
            // entry) actually takes -- so tilt_offset_for_height()'s LUT
            // lookup tracks where the body really is, instead of jumping to
            // the final value ~ARRIVAL_DURATION_S before the legs get there.
            // Deterministic tick countdown (not repeated small increments)
            // avoids float drift across the glide.
            if (--s_arrival_ticks_left <= 0) {
                s_current = s_target;
                s_vel = 0.0f;
                s_arrival_active = false;
            } else {
                float frac = 1.0f - (float)s_arrival_ticks_left / (float)ARRIVAL_DURATION_TICKS;
                s_current = s_arrival_start_cm + (s_target - s_arrival_start_cm) * frac;
            }
            // Diff already converged before glide entry (see the guard
            // below) -- nothing to slew here.
            return;  // one-shot arrival send already fired at glide entry;
                     // no periodic send while it's physically in flight.
        }

        // Height: trapezoidal profile, restored 2026-07-15 (was briefly
        // swapped for a constant-rate ramp during testing). Desired speed
        // is cruise, capped by the decelerate-into-target limit
        // v = sqrt(2*a*d), toward the target.
        float d = s_target - s_current;
        // Early-snap (2026-07-15): end the ramp as soon as the REMAINING
        // distance is within the sub-1-deg tail (RAMP_TAIL_DISTANCE_CM), not
        // just within one tick's step (the old fmaxf(fabsf(s_vel),
        // RAMP_ACCEL) threshold) -- hands the whole tail to the one-shot
        // arrival send instead of dribbling through it in sub-1-deg ramp
        // steps (the end-of-ramp oscillation source). Still falls back to
        // the tiny one-tick threshold too, in case RAMP_TAIL_DISTANCE_CM is
        // ever smaller (e.g. a future low-accel/low-speed retune).
        //
        // SCOPE (2026-07-15): the smooth arrival glide below only applies to
        // a PURE height move (diff already converged, `fabsf(dd) small`) --
        // a combined height+split ramp falls back to the old instant snap,
        // since diff/split ramping has its own known asymmetric-drift issue
        // parked for a separate fix (not this one).
        float dd_now = s_diff_t - s_diff;
        bool diff_settled = fabsf(dd_now) <= 1e-4f;
        if (fabsf(d) <= fmaxf(fmaxf(fabsf(s_vel), RAMP_ACCEL), RAMP_TAIL_DISTANCE_CM)) {
            if (diff_settled) {
                // Enter the glide: fire the one-shot MATCHED arrival send NOW
                // (real motors start moving toward s_target this instant),
                // then glide the MODEL toward s_target over the same
                // ARRIVAL_DURATION_S instead of jumping there instantly.
                send_height(s_target, s_diff, ARRIVAL_SPEED_CEILING_DPS, true, ARRIVAL_MARGIN, ARRIVAL_DURATION_S);
                s_arrival_start_cm   = s_current;
                s_arrival_ticks_left = ARRIVAL_DURATION_TICKS;
                s_arrival_active     = true;
                s_vel   = 0.0f;
                s_tick_ct = 0;
                return;  // periodic send below must not also fire this tick
            }
            // Diff still mid-ramp: old behavior, instant snap (out of scope
            // for the smooth-glide fix; see comment above).
            s_current = s_target;
            s_vel = 0.0f;
        } else {
            float v_stop = sqrtf(2.0f * RAMP_ACCEL * fabsf(d));
            float v_des  = fminf(RAMP_MAX_V, v_stop) * (d > 0 ? 1.0f : -1.0f);
            // Slew actual velocity toward desired (bounds jerk at start and on
            // mid-ramp target changes, including direction reversals).
            float dv = v_des - s_vel;
            if (dv >  RAMP_ACCEL) dv =  RAMP_ACCEL;
            if (dv < -RAMP_ACCEL) dv = -RAMP_ACCEL;
            s_vel += dv;
            s_current += s_vel;
        }

        // Differential: trapezoid (2026-07-16, was a plain rate limit). Accel
        // toward cruise, decel into the target via v = sqrt(2*a*d), early-snap
        // the sub-1-deg tail. The matched arrival send at the bottom lands the
        // snapped tail with synchronized legs (no async landing). No glide --
        // diff doesn't feed the tilt LUT (see DIFF_ACCEL comment).
        float dd = s_diff_t - s_diff;
        if (fabsf(dd) <= fmaxf(fmaxf(fabsf(s_diff_vel), DIFF_ACCEL), DIFF_TAIL_DISTANCE_CM)) {
            s_diff = s_diff_t;      // early-snap; matched arrival handles the last leg motion
            s_diff_vel = 0.0f;
        } else {
            float vd_stop = sqrtf(2.0f * DIFF_ACCEL * fabsf(dd));
            float vd_des  = fminf(DIFF_SLEW_PER_TICK, vd_stop) * (dd > 0 ? 1.0f : -1.0f);
            float dvd = vd_des - s_diff_vel;
            if (dvd >  DIFF_ACCEL) dvd =  DIFF_ACCEL;
            if (dvd < -DIFF_ACCEL) dvd = -DIFF_ACCEL;
            s_diff_vel += dvd;
            s_diff += s_diff_vel;
        }
    }

    // Arrival on height/diff THIS tick (the instant-snap fallback path, or
    // trim-only), with no trim in flight: send the exact final pose
    // immediately and go silent. If trim is still moving, fall through to
    // the decimated send below instead (still needs to keep transmitting),
    // and the outer gate keeps this function alive next tick via
    // trim_pending regardless of ramp state.
    if (was_ramping && !HeightCtrl_IsRamping() && !trim_pending) {
        // Final arrival (diff-involved completions + the combined height instant-
        // snap path). MATCHED (2026-07-16, was flat): lands both legs together so
        // a split's last step doesn't finish asynchronously (async landing was
        // part of the end-of-split oscillation). Pure-height arrivals never reach
        // here -- they take the glide branch above.
        send_height(s_current, s_diff, ARRIVAL_SPEED_CEILING_DPS, true, ARRIVAL_MARGIN, ARRIVAL_DURATION_S);
        s_tick_ct = 0;
        return;
    }
    if (++s_tick_ct >= SEND_EVERY_N_TICKS) {
        s_tick_ct = 0;
        // 2026-07-15, user-directed: matched-speed back ON, both directions,
        // margin=1.4 (STEP_SPEED_MARGIN_EXTEND -- FOR THE RECORD, this is in
        // the 1.2-1.4 range the original o->l search found caused drift that
        // ran away to a fall). Send rate 100Hz (SEND_EVERY_N_TICKS=2), Kp1=65
        // and ramp=5.0cm/s trapezoidal unchanged from the prior flat-60 test.
        bool  matched   = true;
        float margin    = STEP_SPEED_MARGIN_EXTEND;  // 1.4, used for BOTH directions
        send_height(s_current, s_diff,
                    trim_pending ? TRIM_CMD_SPEED_DPS : LEG_CMD_SPEED_DPS, matched, margin,
                    SEND_INTERVAL_S);
    }
}
