#pragma once
#include <Arduino.h>

void  IMU_Init();
bool  IMU_Update();   // call every loop tick; returns true when a new packet decoded
float IMU_GetPitch(); // degrees, NWU frame
float IMU_GetRoll();
float IMU_GetYaw();
bool  IMU_IsValid();  // true after at least one valid packet received (latches — check staleness too)
uint32_t IMU_MsSinceLastPacket(); // ms since last CRC-valid packet (HI229 streams at 200 Hz)
// Diagnostic (2026-07-15): packets parsed by the LAST IMU_Update() call --
// resolves whether new data actually arrives every control tick (200Hz) or
// every other tick (100Hz, per pid.cpp's D-filter comment). 0 = no new
// sample this tick (stale repeat), 1 = normal, 2+ = tick fell behind.
uint32_t IMU_GetPacketsLastUpdate();

// Body-frame specific force, m/s^2 (includes gravity).
float IMU_GetAccelX();
float IMU_GetAccelY();
float IMU_GetAccelZ();
// World-vertical acceleration with gravity removed (m/s^2); ~0 at rest.
// Filtered = EMA'd (~14 Hz); Raw = unfiltered, for bench sanity-checking.
float IMU_GetVerticalAccel();
float IMU_GetVerticalAccelRaw();
