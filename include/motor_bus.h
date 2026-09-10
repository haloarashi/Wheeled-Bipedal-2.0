#pragma once
#include <stdint.h>

// Comment out to enable all 6 motors (both sides).
// #define LEFT_SIDE_ONLY

// motor_speed raw field units: output_shaft_dps = motor_speed / 10.0 (verified empirically 2026-07-03)
// Motor IDs: 1=hip-L  2=knee-L  3=hip-R  4=knee-R  5=wheel-R  6=wheel-L

void   MotorBus_Init(bool enable_wheels);

// Leg motors (IDs 1-4)
void   MotorBus_SetLegAngle(int id, double angle_deg, double max_speed_dps);
void   MotorBus_SetLegAngleUncapped(int id, double angle_deg);  // 0xA3: position mode, NO speed cap
void   MotorBus_RequestLegAngles();   // Read_Angle_MultiRound on IDs 1-4
void   MotorBus_RequestHipAngles();   // Read_Angle_MultiRound on IDs 1+3 only (lighter, for periodic polling)

// ID3 (hip-R) angle/speed/current probe (2026-07-16) -- for live-graphing
// during LK-tool bench tuning (0x9C single-request gives all three fields in
// one round-trip). ID3 only, not a general per-motor API -- extend if another
// motor needs this later.
void   MotorBus_RequestID3State();
double MotorBus_GetID3AngleDeg();    // single-round ENCODER angle (motor shaft, pre-reduction, -180..180)
double MotorBus_GetID3SpeedDPS();    // output-shaft dps (raw motor_speed / 10.0, same scale as wheels)
double MotorBus_GetID3CurrentA();    // motor_iq, amps
double MotorBus_GetLegAngle(int id);  // returns motor_angle_multi (degrees)
bool   MotorBus_LegReplied(int id);   // true if this leg motor has answered on the bus this session

// Wheel motors (IDs 5-6) — only valid after enable_wheels=true
void   MotorBus_SetWheelSpeed(double right_dps, double left_dps);
void   MotorBus_RequestWheelState();
double MotorBus_GetWheelSpeedDPS(int id); // id = 5 or 6
void   MotorBus_StopWheels();
void   MotorBus_EStop();
void   MotorBus_FreeLegMotors();      // set torque=0 on active leg motors so they can be moved by hand
void   MotorBus_FreeKnees();          // torque=0 on ID2+ID4 only (hip-jiggle coupling diagnosis)
void   MotorBus_RightSideFree();      // enable ID3+ID4 then zero torque (cal only, LEFT_SIDE_ONLY safe)
void   MotorBus_RightSidePrintAngles(); // read+print ID3+ID4 multi-round angles
void   MotorBus_PrintLegAngles();  // read+print multi-round angles of ID1 and ID2
void   MotorBus_DiagMotors();     // diagnostic: Read_Motor_State_2 on ID1+ID2, prints resp_id (0=no reply)
void   MotorBus_ScanIDs();        // scan IDs 1-6 to find all motors on bus
int8_t MotorBus_GetMaxTempC();                 // max temperature across all 6 motors (cached, no RS485 read)
void   MotorBus_PrintAllTemps();               // read + print temperature of all 6 motors
bool   MotorBus_TempCheck(int8_t threshold_c); // reads one motor per call (rotates 1-6); returns true if any exceed threshold
