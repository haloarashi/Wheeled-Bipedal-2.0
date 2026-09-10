#include "motor_bus.h"
#include <LKM_Motor.h>
#include <LKM_Motor_Receive.h>
#include <Arduino.h>

// 2026-07-15 hardware rewire: legs (IDs 1-4) stay on Serial5 (RS485_1 --
// hip-L/knee-L via ASR Lab PCB port 1-1, hip-R/knee-R via port 1-2). Wheels
// (IDs 5-6) moved to Serial3 (RS485_2 -- wheel-L via port 2-1, wheel-R via
// port 2-2), off the shared Serial5 daisy chain, so leg-height traffic and
// wheel commands no longer compete for the same bus/blocking budget (the
// tick-timing pressure found via the 'T' probe). Two physically separate
// buses -- confirmed from LKM_Motor.cpp: Serial5.transmitterEnable(2) for
// serial_port==5, Serial3.transmitterEnable(13) for serial_port==3. Do not
// manually control either DE/RE pin, the library owns them.
static LKM_Motor hip_left(1,    8, 5);
static LKM_Motor knee_left(2,   8, 5);
static LKM_Motor hip_right(3,   8, 5);
static LKM_Motor knee_right(4,  8, 5);
static LKM_Motor wheel_right(5, 10, 3);
static LKM_Motor wheel_left(6,  10, 3);

// One receive router per physical bus -- LKM_Motor_Receive binds to a single
// Stream; a motor registered on the wrong one would have its replies parsed
// against a buffer they never actually arrive on.
static LKM_Motor_Receive MotorBus;   // Serial5 -- legs (IDs 1-4)
static LKM_Motor_Receive WheelBus;   // Serial3 -- wheels (IDs 5-6)
static bool s_wheels_enabled = false;
static bool s_motors_init    = false;

// Post-write ACK-drain delay. A fire-and-forget write (Set_Need_Receive false)
// still draws a ~13-byte motor ACK ~180us later (round-trip figure from the
// ScanIDs comment below). This delay's REAL job is COLLISION AVOIDANCE on the
// half-duplex bus: it holds off the NEXT same-bus command until the current
// command's ACK has finished on the wire -- otherwise the Teensy asserts DE to
// transmit while the motor is still driving the line, corrupting both. It also
// lets the following Serial.clear() actually catch the ACK. 2026-07-15: 500 ->
// 250 after the leg/wheel bus split (motor_bus header) halved per-bus traffic.
// 250 is still ~1.4x the ~180us round-trip -- do NOT drop below ~200us without
// bench validation: a too-short value causes intermittent same-bus command
// collisions, whose symptom is a dropped/garbled motor command == JITTER, which
// would silently confound the very thing this project is chasing. The 'T' probe
// shows the blocking saved; watch for new jitter / read failures as the
// collision signature when tuning lower.
static constexpr uint32_t MOTOR_ACK_DRAIN_US = 250;

static LKM_Motor* leg_motor(int id) {
    switch (id) {
        case 1: return &hip_left;
        case 2: return &knee_left;
        case 3: return &hip_right;
        case 4: return &knee_right;
        default: return nullptr;
    }
}

static LKM_Motor* wheel_motor(int id) {
    if (id == 5) return &wheel_right;
    if (id == 6) return &wheel_left;
    return nullptr;
}

static void init_motor(LKM_Motor &m, HardwareSerialIMXRT &bus) {
    m.debug_mode = false;
    m.Set_Need_Receive(false);  // fire-and-forget: no blocking delay/Receive_All in write fns
    m.Write_Motor_Run();
    delay(10);
    bus.clear();  // discard Write_Motor_Run ACK to prevent framing desync
}

void MotorBus_Init(bool enable_wheels) {
    s_motors_init    = true;
    s_wheels_enabled = enable_wheels;

    MotorBus = LKM_Motor_Receive(5);
    WheelBus = LKM_Motor_Receive(3);
    MotorBus.debug_mode = false;
    WheelBus.debug_mode = false;

    // Serial_Init sets MOTOR_SERIAL on each instance — must be called on all objects.
    // Legs share Serial5, wheels share Serial3 (separate buses since 2026-07-15).
    hip_left.debug_mode    = false;  hip_left.Serial_Init();
    knee_left.debug_mode   = false;  knee_left.Serial_Init();
    hip_right.debug_mode   = false;  hip_right.Serial_Init();
    knee_right.debug_mode  = false;  knee_right.Serial_Init();
    wheel_right.debug_mode = false;  wheel_right.Serial_Init();
    wheel_left.debug_mode  = false;  wheel_left.Serial_Init();

    // Register every motor on ITS bus's receive router. Without this, reads
    // parse whatever 13-byte packet is first in the RX buffer with NO motor-ID
    // check — wheel L/R telemetry gets swapped (confirmed on bench 2026-07-03:
    // avg_speed flipped sign in blocks at constant wheel_cmd). With registration
    // the library routes each reply to the motor whose ID matches -- and now
    // also requires registering on the bus the motor's replies actually arrive on.
    MotorBus.RegisterMotor(&hip_left);
    MotorBus.RegisterMotor(&knee_left);
    MotorBus.RegisterMotor(&hip_right);
    MotorBus.RegisterMotor(&knee_right);
    WheelBus.RegisterMotor(&wheel_right);
    WheelBus.RegisterMotor(&wheel_left);

    // Enable leg motors.
    init_motor(hip_left,  Serial5);
    init_motor(knee_left, Serial5);
#ifndef LEFT_SIDE_ONLY
    init_motor(hip_right,  Serial5);
    init_motor(knee_right, Serial5);
#else
    // Explicitly pause right side so it doesn't hold any previously commanded position.
    hip_right.Write_Motor_Pause();  delay(10);  Serial5.clear();
    knee_right.Write_Motor_Pause(); delay(10);  Serial5.clear();
#endif

    if (enable_wheels) {
#ifndef LEFT_SIDE_ONLY
        init_motor(wheel_right, Serial3);
#endif
        init_motor(wheel_left, Serial3);
    }
}

void MotorBus_SetLegAngle(int id, double angle_deg, double max_speed_dps) {
#ifdef LEFT_SIDE_ONLY
    if (id == 3 || id == 4) return;  // right side not initialized
#endif
    LKM_Motor *m = leg_motor(id);
    if (!m) return;
    m->Write_Angle_MultiRound(angle_deg, max_speed_dps);
    delayMicroseconds(MOTOR_ACK_DRAIN_US);  // collision-avoidance + ACK drain -- see MOTOR_ACK_DRAIN_US
    Serial5.clear();
}

void MotorBus_SetLegAngleUncapped(int id, double angle_deg) {
    // 0xA3 multi-round position WITHOUT the speed-cap field. The 0xA4 cap is a
    // saturation element inside the motor's position loop (pos error -> speed
    // demand -> CLAMP -> speed loop) — a candidate limit-cycle source for the
    // hip jiggle. Only use when the motor is already at/near angle_deg: with
    // no cap, a large step would move at the motor's full internal speed.
#ifdef LEFT_SIDE_ONLY
    if (id == 3 || id == 4) return;
#endif
    LKM_Motor *m = leg_motor(id);
    if (!m) return;
    m->Write_Angle_MultiRound(angle_deg);
    delayMicroseconds(MOTOR_ACK_DRAIN_US);
    Serial5.clear();
}

void MotorBus_RequestLegAngles() {
    if (!s_motors_init) return;
    hip_left.Read_Angle_MultiRound();
    knee_left.Read_Angle_MultiRound();
#ifndef LEFT_SIDE_ONLY
    hip_right.Read_Angle_MultiRound();
    knee_right.Read_Angle_MultiRound();
#endif
}

// Diagnostic (2026-07-15): ID1+ID3 only, not all 4 -- half the extra bus
// traffic of MotorBus_RequestLegAngles(), since this is meant to run
// periodically WHILE a ramp is already sending position commands on the
// same Serial5 bus. Any extra read still adds real bus time on top of the
// existing send traffic -- keep the caller's poll rate modest (a few tens
// of Hz, not every tick) so the measurement doesn't perturb what it's trying
// to observe.
void MotorBus_RequestHipAngles() {
    if (!s_motors_init) return;
    hip_left.Read_Angle_MultiRound();
#ifndef LEFT_SIDE_ONLY
    hip_right.Read_Angle_MultiRound();
#endif
}

// ID3 (hip-R) angle/speed/current probe (2026-07-16), for LK-tool bench
// tuning graphs. One 0x9C request gives all three fields in a single
// round-trip -- cheaper than a separate angle + a separate diag read.
// SPEED SCALE: the library stores motor_speed RAW (no reduction-ratio
// division applied on read -- confirmed in LKM_Motor.cpp, only the WRITE
// path multiplies by _reduction_ratio). The wheels' empirical "/10.0"
// (motor_bus.cpp MotorBus_GetWheelSpeedDPS) is /reduction_ratio for their
// 10:1 gearing, NOT a universal constant -- hip_right is 8:1, so the correct
// output-shaft scale here is /8.0, not /10.0.
void MotorBus_RequestID3State() {
    if (!s_motors_init) return;
    while (Serial5.available()) Serial5.read();
    hip_right.Read_Motor_State_2();
    delayMicroseconds(MOTOR_ACK_DRAIN_US);
    Serial5.clear();
}

double MotorBus_GetID3AngleDeg()  { return hip_right.motor_angle_encoder; }
double MotorBus_GetID3SpeedDPS()  { return hip_right.motor_speed / 8.0; }  // 8:1 ratio, NOT the wheels' /10.0
double MotorBus_GetID3CurrentA()  { return hip_right.motor_iq; }

double MotorBus_GetLegAngle(int id) {
    LKM_Motor *m = leg_motor(id);
    return m ? m->motor_angle_multi : 0.0;
}

bool MotorBus_LegReplied(int id) {
    // motor_id is 0 at boot and only set from a parsed reply, so this tells us
    // whether the motor has ever actually answered on the bus this session.
    LKM_Motor *m = leg_motor(id);
    return m && m->motor_id == id;
}


void MotorBus_SetWheelSpeed(double right_dps, double left_dps) {
    if (!s_wheels_enabled) return;
#ifndef LEFT_SIDE_ONLY
    wheel_right.Write_Speed(right_dps);
#endif
    wheel_left.Write_Speed(left_dps);
    // Motors ACK every 0xA2 with 13 bytes even in fire-and-forget mode. Left
    // undrained they accumulate (+2 pkts/tick) and later reads parse stale ACKs
    // as replies. Same pattern as SetLegAngle: let ACKs land, then drop them.
    // Wheels are on Serial3, not Serial5, since the 2026-07-15 bus split.
    delayMicroseconds(MOTOR_ACK_DRAIN_US);
    Serial3.clear();
}

void MotorBus_RequestWheelState() {
    if (!s_wheels_enabled) return;
    // Defensive drain (2026-07-15): with the reduced MOTOR_ACK_DRAIN_US, a
    // wheel Write_Speed ACK from the end of the previous tick could still be
    // sitting in the Serial3 buffer. Clear it before the read so Receive_All
    // parses the fresh 0x9C reply, not a stale 0xA2 write-ACK. (The ~5ms
    // read->write->read tick spacing means it has long since arrived; this is
    // just belt-and-suspenders now that we don't over-wait after the write.)
    Serial3.clear();
#ifndef LEFT_SIDE_ONLY
    wheel_right.Read_Motor_State_2();
#endif
    wheel_left.Read_Motor_State_2();
}

double MotorBus_GetWheelSpeedDPS(int id) {
    LKM_Motor *m = wheel_motor(id);
    if (!m || !s_wheels_enabled) return 0.0;
    // motor_speed raw units verified empirically 2026-07-03: commanded 200 dps -> raw/100.0
    // read back ~20.0 (10x too low). Correct scale is /10.0 (raw/100.0 read back matched
    // commanded/10 exactly across 9 samples, ~20.0 +/- 0.2).
    return m->motor_speed / 10.0;
}

void MotorBus_StopWheels() {
    if (!s_wheels_enabled) return;
#ifndef LEFT_SIDE_ONLY
    wheel_right.Write_Speed(0.0);
#endif
    wheel_left.Write_Speed(0.0);
}

void MotorBus_FreeLegMotors() {
    // Zero torque: motors are backdrivable, encoder still tracks position
    auto free_one = [](LKM_Motor &m) {
        m.Write_Torque_Current(0.0);
        delayMicroseconds(MOTOR_ACK_DRAIN_US);
        Serial5.clear();
    };
    free_one(hip_left);
    free_one(knee_left);
#ifndef LEFT_SIDE_ONLY
    free_one(hip_right);
    free_one(knee_right);
#endif
}

void MotorBus_FreeKnees() {
    // Knees only: hip loops keep holding while the knee motors go compliant.
    // Isolates the hip<->knee coupled-servo interaction (knee motor rides on
    // the hip arm, so two stiff position loops are mechanically coupled).
    auto free_one = [](LKM_Motor &m) {
        m.Write_Torque_Current(0.0);
        delayMicroseconds(MOTOR_ACK_DRAIN_US);
        Serial5.clear();
    };
    free_one(knee_left);
#ifndef LEFT_SIDE_ONLY
    free_one(knee_right);
#endif
}

void MotorBus_PrintLegAngles() {
    auto read_one = [](LKM_Motor &m, const char* label) {
        while (Serial5.available()) Serial5.read();
        m.Read_Angle_MultiRound();
        Serial.print(label);
        Serial.print(m.motor_angle_multi, 2);
        Serial.println(" deg");
        while (Serial5.available()) Serial5.read();
    };
    read_one(hip_left,   "hip_L  ID1: ");
    read_one(knee_left,  "knee_L ID2: ");
#ifndef LEFT_SIDE_ONLY
    read_one(hip_right,  "hip_R  ID3: ");
    read_one(knee_right, "knee_R ID4: ");
#endif
}

// --- Right-side calibration helpers (usable even with LEFT_SIDE_ONLY) ---

void MotorBus_RightSideFree() {
    // Enable then immediately zero-torque ID3+ID4 so they can be moved by hand
    auto free_one = [](LKM_Motor &m) {
        m.Write_Motor_Run();
        delayMicroseconds(MOTOR_ACK_DRAIN_US);
        Serial5.clear();
        m.Write_Torque_Current(0.0);
        delayMicroseconds(MOTOR_ACK_DRAIN_US);
        Serial5.clear();
    };
    free_one(hip_right);
    free_one(knee_right);
}

void MotorBus_RightSidePrintAngles() {
    auto read_one = [](LKM_Motor &m, const char* label) {
        while (Serial5.available()) Serial5.read();
        m.Read_Angle_MultiRound();
        Serial.print(label);
        Serial.print(m.motor_angle_multi, 2);
        Serial.println(" deg");
        while (Serial5.available()) Serial5.read();
    };
    read_one(hip_right,  "hip_R  ID3: ");
    read_one(knee_right, "knee_R ID4: ");
}

void MotorBus_DiagMotors() {
    auto print_one = [](LKM_Motor &m, const char* label) {
        while (Serial5.available()) Serial5.read();
        m.debug_mode = true;
        m.Read_Motor_State_2();
        m.debug_mode = false;
        Serial.println(label);
        Serial.print("  resp_id=");  Serial.print(m.motor_id);
        Serial.print(" temp=");      Serial.print(m.motor_temperature);
        Serial.print("C iq=");       Serial.print(m.motor_iq, 2);
        Serial.print("A spd=");      Serial.print(m.motor_speed);
        Serial.print(" enc=");       Serial.println(m.motor_encoder);
        delay(5);
        while (Serial5.available()) Serial5.read();
    };

    print_one(hip_left,   "-- ID1 (hip-L) --");
    print_one(knee_left,  "-- ID2 (knee-L) --");
    print_one(hip_right,  "-- ID3 (hip-R) --");
    print_one(knee_right, "-- ID4 (knee-R) --");
}

void MotorBus_ScanIDs() {
    Serial.println("Scanning IDs 1-6...");
    for (int id = 1; id <= 6; id++) {
        // Legs (1-4) are on Serial5, wheels (5-6) on Serial3 since the 2026-07-15 bus split.
        HardwareSerialIMXRT &bus = (id <= 4) ? Serial5 : Serial3;
        while (bus.available()) bus.read();  // drain

        // Raw Read_Motor_State_2 (0x9C) packet to arbitrary ID
        uint8_t pkt[5];
        pkt[0] = 0x3E;
        pkt[1] = 0x9C;
        pkt[2] = (uint8_t)id;
        pkt[3] = 0x00;
        pkt[4] = pkt[0] + pkt[1] + pkt[2] + pkt[3];
        bus.write(pkt, 5);

        delay(5);  // generous: 5ms >> 180µs round-trip at 4Mbps

        // Collect up to 13 bytes after 0x3E header
        uint8_t resp[13] = {};
        int n = 0;
        bool hdr = false;
        uint32_t t0 = millis();
        while (millis() - t0 < 10) {
            if (!bus.available()) continue;
            uint8_t b = bus.read();
            if (!hdr && b == 0x3E) { resp[0] = b; n = 1; hdr = true; }
            else if (hdr && n < 13) { resp[n++] = b; }
            if (n == 13) break;
        }

        Serial.print("  ID"); Serial.print(id); Serial.print(": ");
        if (n >= 13) {
            Serial.print("FOUND  motor_id="); Serial.print((int)resp[2]);
            Serial.print("  temp=");          Serial.print((int8_t)resp[5]);
            Serial.print("C  spd=");
            Serial.println((int16_t)((resp[9] << 8) | resp[8]));
        } else {
            Serial.print("no response (got "); Serial.print(n); Serial.println(" bytes)");
        }
        delay(10);
    }
    Serial.println("Scan done.");
}

int8_t MotorBus_GetMaxTempC() {
    LKM_Motor* motors[] = { &hip_left, &knee_left, &hip_right, &knee_right, &wheel_right, &wheel_left };
    int8_t mx = motors[0]->motor_temperature;
    for (int i = 1; i < 6; i++)
        if (motors[i]->motor_temperature > mx) mx = motors[i]->motor_temperature;
    return mx;
}

void MotorBus_PrintAllTemps() {
    LKM_Motor* motors[] = { &hip_left, &knee_left, &hip_right, &knee_right, &wheel_right, &wheel_left };
    // Legs (0-3) reply on Serial5, wheels (4-5) on Serial3 since the 2026-07-15 bus split.
    HardwareSerialIMXRT* buses[] = { &Serial5, &Serial5, &Serial5, &Serial5, &Serial3, &Serial3 };
    const char* labels[] = { "ID1 hip-L ", "ID2 knee-L", "ID3 hip-R ", "ID4 knee-R", "ID5 whl-R ", "ID6 whl-L " };
    Serial.println("-- Motor Temperatures --");
    for (int i = 0; i < 6; i++) {
        HardwareSerialIMXRT &bus = *buses[i];
        while (bus.available()) bus.read();
        motors[i]->Read_Motor_State_2();
        delayMicroseconds(3000);
        while (bus.available()) bus.read();
        Serial.print("  "); Serial.print(labels[i]);
        Serial.print(": "); Serial.print((int)motors[i]->motor_temperature);
        Serial.println(" C");
    }
    Serial.println("------------------------");
}

// Overtemp guard, hardened after two false ESTOPs (ID2 "77C" 2026-07-06, ID3
// "124C" 2026-07-07 — both motors cool to the touch):
//   1. STALENESS: motor_temperature only updates on a checksum-valid reply
//      (LKM _Unpack); a failed read leaves the OLD value in place. Sentinel the
//      field before reading — if unchanged after, the read failed: restore the
//      last value (keeps maxT display sane) and never judge stale data.
//   2. DEBOUNCE: require 3 consecutive over-threshold reads of the SAME motor
//      before ESTOP. Real overheat persists (thermal mass); a corrupted frame
//      (weak 8-bit checksum passes garbage ~1/256, e.g. ID3 "124C" while cool)
//      doesn't repeat. At the ~1 s/motor rotation a real trip is delayed ~2 s.
//      No upper plausibility cut — an extreme reading (short) still counts
//      toward the streak and trips as fast as any other overheat.
// Failed reads leave the streak UNCHANGED (neither count nor reset), so
// interleaved read failures can't stall a genuine overheat trip.
static constexpr int8_t  TEMP_SENTINEL     = -128;
static constexpr uint8_t TEMP_ESTOP_STREAK = 3;

bool MotorBus_TempCheck(int8_t threshold_c) {
    static uint8_t s_idx = 0;
    static uint8_t s_over[6] = {0};   // consecutive over-threshold reads per motor
    LKM_Motor* motors[] = { &hip_left, &knee_left, &hip_right, &knee_right, &wheel_right, &wheel_left };
    // Legs (0-3) reply on Serial5, wheels (4-5) on Serial3 since the 2026-07-15 bus split.
    HardwareSerialIMXRT* buses[] = { &Serial5, &Serial5, &Serial5, &Serial5, &Serial3, &Serial3 };
    const char* labels[] = { "ID1(hip-L)", "ID2(knee-L)", "ID3(hip-R)", "ID4(knee-R)", "ID5(whl-R)", "ID6(whl-L)" };

    uint8_t i = s_idx;
    LKM_Motor& m = *motors[i];
    HardwareSerialIMXRT& bus = *buses[i];
    const char* label = labels[i];
    s_idx = (s_idx + 1) % 6;

    int8_t prev = m.motor_temperature;
    m.motor_temperature = TEMP_SENTINEL;

    while (bus.available()) bus.read();
    m.Read_Motor_State_2();
    delayMicroseconds(2000);
    while (bus.available()) bus.read();

    int8_t t = m.motor_temperature;
    if (t == TEMP_SENTINEL) {
        m.motor_temperature = prev;   // failed read: keep last-known
        return false;
    }

    if (t > threshold_c) {
        if (++s_over[i] >= TEMP_ESTOP_STREAK) {
            Serial.print("OVERTEMP "); Serial.print(label);
            Serial.print(": "); Serial.print((int)t);
            Serial.println("C -> ESTOP");
            s_over[i] = 0;
            return true;
        }
        Serial.print("temp warn "); Serial.print(label);
        Serial.print(": "); Serial.print((int)t);
        Serial.print("C ("); Serial.print(s_over[i]);
        Serial.print("/"); Serial.print(TEMP_ESTOP_STREAK); Serial.println(")");
    } else {
        s_over[i] = 0;
    }
    return false;
}

void MotorBus_EStop() {
    MotorBus_StopWheels();
    hip_left.Write_Motor_Pause();
    knee_left.Write_Motor_Pause();
#ifndef LEFT_SIDE_ONLY
    hip_right.Write_Motor_Pause();
    knee_right.Write_Motor_Pause();
#endif
    if (s_wheels_enabled) {
#ifndef LEFT_SIDE_ONLY
        wheel_right.Write_Motor_Pause();
#endif
        wheel_left.Write_Motor_Pause();
    }
}
