#include "imu.h"
#include <string.h>
#include <math.h>

// HI229 CH Serial Protocol (HI229 datasheet Rev 1.0, page 13-18)
// Frame: [0x5A][0xA5][LEN_lo][LEN_hi][CRC_lo][CRC_hi][DATA...]
// DATA = concatenated sub-packets, each: [tag][payload_bytes...]
// CRC16/CCITT poly 0x1021, seed 0 (NOT 0xFFFF), covers 0x5A+0xA5+LEN+DATA
//
// Default HI229 output (AT+SETPTL=90,A0,B0,C0,D0,F0), total DATA = 35 bytes:
//   0x90: 2B  — user ID
//   0xA0: 7B  — accel XYZ  (int16 x3, 0.001G)
//   0xB0: 7B  — gyro XYZ   (int16 x3, 0.1 deg/s)
//   0xC0: 7B  — mag XYZ    (int16 x3, 0.001 Gauss)
//   0xD0: 7B  — Euler: [pitch int16 x0.01deg][roll int16 x0.01deg][yaw int16 x0.1deg]
//   0xF0: 5B  — pressure (float, Pa)

static constexpr uint8_t  SYNC0       = 0x5A;
static constexpr uint8_t  SYNC1       = 0xA5;
static constexpr uint8_t  TAG_ACCEL   = 0xA0;
static constexpr uint8_t  TAG_EULER   = 0xD0;
static constexpr uint16_t MAX_DATA_LEN = 128;
static constexpr float    G_MS2       = 9.80665f;

enum ParseState : uint8_t {
    WAIT_SYNC0, WAIT_SYNC1,
    READ_LEN_LO, READ_LEN_HI,
    READ_CRC_LO, READ_CRC_HI,
    READ_DATA
};

static ParseState s_state    = WAIT_SYNC0;
static uint16_t   s_len      = 0;
static uint16_t   s_crc_recv = 0;
static uint16_t   s_data_idx = 0;
static uint8_t    s_data[MAX_DATA_LEN];

static float s_pitch = 0.0f;
static float s_roll  = 0.0f;
static float s_yaw   = 0.0f;
static bool  s_valid = false;
static uint32_t s_last_pkt_ms = 0;  // millis() of last CRC-valid packet

// Diagnostic probe (2026-07-15): how many complete, CRC-valid packets does a
// single IMU_Update() call actually parse? Answers a real contradiction in
// this codebase's own comments -- pid.cpp claims new samples arrive only
// every 2nd control tick (~100Hz effective), while the staleness guard and
// s_vacc_filt's alpha comment both assume a clean 200Hz stream. Whichever is
// true changes whether pid1's derivative term is fed a fresh sample every
// tick or a stale repeat -- directly relevant to any control-rate change.
// Read-only counter, no behavior change.
static uint32_t s_pkts_last_update = 0;

// Body-frame specific force from tag 0xA0 (m/s^2, includes gravity).
static float s_ax = 0.0f, s_ay = 0.0f, s_az = 0.0f;
// World-vertical acceleration, gravity removed: rotate body accel to world
// using the IMU's OWN Euler angles (vertical is frame-independent, so the
// device's mounting rotation on the robot doesn't matter here), take z,
// subtract g. At rest this should read ~0 -- that is the Stage-1 bench gate;
// if the assumed ZYX/NWU convention is wrong, the rest check exposes it.
static float s_vacc_raw  = 0.0f;
static float s_vacc_filt = 0.0f;   // EMA alpha 0.3 per packet (~14 Hz at 200 Hz stream)

// Known sub-packet sizes (including the tag byte itself).
static uint8_t sub_packet_size(uint8_t tag) {
    switch (tag) {
        case 0x90: return 2;
        case 0xA0: return 7;
        case 0xB0: return 7;
        case 0xC0: return 7;
        case 0xD0: return 7;
        case 0xD1: return 17;
        case 0xF0: return 5;
        case 0x91: return 76;
        default:   return 0;
    }
}

static bool validate_and_extract() {
    // CRC covers [5A A5 LEN_lo LEN_hi DATA...], seed = 0 (datasheet p.13)
    uint16_t crc = 0;
    uint8_t hdr[4] = { SYNC0, SYNC1,
                       (uint8_t)(s_len & 0xFF),
                       (uint8_t)(s_len >> 8) };
    for (int i = 0; i < 4; i++) {
        crc ^= ((uint16_t)hdr[i]) << 8;
        for (int b = 0; b < 8; b++)
            crc = (crc & 0x8000) ? ((crc << 1) ^ 0x1021) : (crc << 1);
    }
    for (uint16_t i = 0; i < s_len; i++) {
        crc ^= ((uint16_t)s_data[i]) << 8;
        for (int b = 0; b < 8; b++)
            crc = (crc & 0x8000) ? ((crc << 1) ^ 0x1021) : (crc << 1);
    }
    if (crc != s_crc_recv) return false;

    // Walk sub-packets; extract 0xA0 accel (arrives before 0xD0 in the frame,
    // so by the time we reach Euler below, s_ax/ay/az are already fresh for
    // this same packet) and 0xD0 Euler angles.
    uint16_t i = 0;
    while (i < s_len) {
        uint8_t tag  = s_data[i];
        uint8_t size = sub_packet_size(tag);
        if (size == 0 || i + size > s_len) break;

        if (tag == TAG_ACCEL) {
            // 0xA0: [A0][ax_lo][ax_hi][ay_lo][ay_hi][az_lo][az_hi], 0.001 G/LSB
            int16_t ax_raw, ay_raw, az_raw;
            memcpy(&ax_raw, &s_data[i + 1], 2);
            memcpy(&ay_raw, &s_data[i + 3], 2);
            memcpy(&az_raw, &s_data[i + 5], 2);
            s_ax = ax_raw * 0.001f * G_MS2;
            s_ay = ay_raw * 0.001f * G_MS2;
            s_az = az_raw * 0.001f * G_MS2;
            // no return -- keep walking to reach 0xD0 in this same packet
        } else if (tag == TAG_EULER) {
            // 0xD0: [D0][pitch_lo][pitch_hi][roll_lo][roll_hi][yaw_lo][yaw_hi]
            int16_t pitch_raw, roll_raw, yaw_raw;
            memcpy(&pitch_raw, &s_data[i + 1], 2);
            memcpy(&roll_raw,  &s_data[i + 3], 2);
            memcpy(&yaw_raw,   &s_data[i + 5], 2);
            s_pitch = pitch_raw / 100.0f;  // 0.01 deg/LSB
            s_roll  = roll_raw  / 100.0f;  // 0.01 deg/LSB
            s_yaw   = yaw_raw   / 10.0f;   // 0.1  deg/LSB

            // Tilt-compensated vertical accel, gravity removed. Rotated using
            // the IMU's OWN pitch/roll (ZYX/NWU) -- vertical is frame-
            // independent, so the device's mounting rotation on the robot
            // doesn't matter here. At rest this should read ~0; that check
            // (Stage-1 bench gate) is what validates this convention on
            // actual hardware rather than trusting it by assumption.
            float thp = s_pitch * (float)M_PI / 180.0f;
            float thr = s_roll  * (float)M_PI / 180.0f;
            float a_wz = -sinf(thp) * s_ax
                       +  sinf(thr) * cosf(thp) * s_ay
                       +  cosf(thr) * cosf(thp) * s_az;
            s_vacc_raw   = a_wz - G_MS2;
            s_vacc_filt += 0.3f * (s_vacc_raw - s_vacc_filt);  // EMA, same alpha as _d_filt elsewhere

            s_valid = true;
            return true;
        }
        i += size;
    }
    return false;
}

void IMU_Init() {
    Serial2.begin(115200);
    s_state = WAIT_SYNC0;
    s_valid = false;
    s_last_pkt_ms = millis();  // grace period so staleness isn't triggered at boot
}

bool IMU_Update() {
    bool got_packet = false;
    s_pkts_last_update = 0;
    while (Serial2.available()) {
        uint8_t b = (uint8_t)Serial2.read();
        switch (s_state) {
            case WAIT_SYNC0:
                if (b == SYNC0) s_state = WAIT_SYNC1;
                break;
            case WAIT_SYNC1:
                s_state = (b == SYNC1) ? READ_LEN_LO : WAIT_SYNC0;
                break;
            case READ_LEN_LO:
                s_len   = b;
                s_state = READ_LEN_HI;
                break;
            case READ_LEN_HI:
                s_len  |= ((uint16_t)b << 8);
                s_state = (s_len == 0 || s_len > MAX_DATA_LEN) ? WAIT_SYNC0 : READ_CRC_LO;
                break;
            case READ_CRC_LO:
                s_crc_recv  = b;
                s_state     = READ_CRC_HI;
                break;
            case READ_CRC_HI:
                s_crc_recv |= ((uint16_t)b << 8);
                s_data_idx  = 0;
                s_state     = READ_DATA;
                break;
            case READ_DATA:
                s_data[s_data_idx++] = b;
                if (s_data_idx >= s_len) {
                    if (validate_and_extract()) {
                        got_packet = true;
                        s_last_pkt_ms = millis();
                        s_pkts_last_update++;
                    }
                    s_state = WAIT_SYNC0;
                }
                break;
        }
    }
    return got_packet;
}

float IMU_GetPitch() { return s_pitch; }
float IMU_GetRoll()  { return s_roll;  }
float IMU_GetYaw()   { return s_yaw;   }
bool  IMU_IsValid()  { return s_valid; }
uint32_t IMU_MsSinceLastPacket() { return millis() - s_last_pkt_ms; }
uint32_t IMU_GetPacketsLastUpdate() { return s_pkts_last_update; }

float IMU_GetAccelX() { return s_ax; }
float IMU_GetAccelY() { return s_ay; }
float IMU_GetAccelZ() { return s_az; }
float IMU_GetVerticalAccel()    { return s_vacc_filt; }
float IMU_GetVerticalAccelRaw() { return s_vacc_raw;  }
