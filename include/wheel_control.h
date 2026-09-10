#pragma once

// Phase A: all functions are stubs (hardware_present=false).
// Phase B: change Init(false) -> Init(true) and implement delegating bodies.
// Sign convention: positive speed = forward for both wheels.
// Left wheel mounting may require sign inversion — set WHEEL_LEFT_SIGN = -1 if needed.

#define WHEEL_RIGHT_SIGN (-1)  // ID5 spins backward with raw +cmd → invert
#define WHEEL_LEFT_SIGN  (+1)  // ID6 spins forward with raw +cmd → keep

void  WheelCtrl_Init(bool hardware_present);
void  WheelCtrl_SetSpeed(float right_dps, float left_dps);
void  WheelCtrl_Stop();
void  WheelCtrl_UpdateFeedback();
float WheelCtrl_GetAvgSpeedDPS(); // returns 0.0f in Phase A
bool  WheelCtrl_IsPresent();
