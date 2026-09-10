#include "wheel_control.h"
#include "motor_bus.h"

static bool s_present = false;
static float s_right_dps = 0.0f;
static float s_left_dps  = 0.0f;

void WheelCtrl_Init(bool hardware_present) {
    s_present = hardware_present;
}

void WheelCtrl_SetSpeed(float right_dps, float left_dps) {
    if (!s_present) return;
    // Phase B: apply per-wheel sign convention, then command
    MotorBus_SetWheelSpeed(right_dps * WHEEL_RIGHT_SIGN,
                           left_dps  * WHEEL_LEFT_SIGN);
}

void WheelCtrl_Stop() {
    if (!s_present) return;
    MotorBus_StopWheels();
    s_right_dps = 0.0f;
    s_left_dps  = 0.0f;
}

void WheelCtrl_UpdateFeedback() {
    if (!s_present) return;
    MotorBus_RequestWheelState();
    s_right_dps = (float)MotorBus_GetWheelSpeedDPS(5) * WHEEL_RIGHT_SIGN;
    s_left_dps  = (float)MotorBus_GetWheelSpeedDPS(6) * WHEEL_LEFT_SIGN;
}

float WheelCtrl_GetAvgSpeedDPS() {
    if (!s_present) return 0.0f;
#ifdef LEFT_SIDE_ONLY
    return s_left_dps;  // only left wheel active
#else
    return (s_right_dps + s_left_dps) * 0.5f;
#endif
}

bool WheelCtrl_IsPresent() {
    return s_present;
}
