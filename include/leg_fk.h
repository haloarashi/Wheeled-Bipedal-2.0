#pragma once

void  LegFK_Init();
float LegFK_HeightCm(float hip_deg, float knee_deg); // -1.0f on unreachable config
float LegFK_XOffsetInches(float hip_deg, float knee_deg);
bool  LegFK_IsReachable(float hip_deg, float knee_deg);
