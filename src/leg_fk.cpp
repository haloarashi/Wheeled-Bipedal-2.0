#include "leg_fk.h"
#include <stdint.h>
#include "leg_geom.h"

// Forward kinematics for the five-bar leg. Link lengths and the shared
// circle-intersection live in leg_geom.h (also used by leg_ik.cpp).
//
// Zero convention (matches physical motor zeroing):
//   theta1 = 0  ->  A in +X (forward horizontal)
//   theta2 = 0  ->  B in -Z (straight down)
//
// Joint positions:
//   A = (R1*cos(t1),  R1*sin(t1))
//   B = (R2*sin(t2), -R2*cos(t2))

static bool fk_xz(float hip_deg, float knee_deg, float &cx, float &cz) {
    float t1 = hip_deg  * LEG_DEG2RAD;
    float t2 = knee_deg * LEG_DEG2RAD;

    float ax = LEG_R1 * cosf(t1);
    float az = LEG_R1 * sinf(t1);

    float bx = LEG_R2 * sinf(t2);
    float bz = -LEG_R2 * cosf(t2);

    return Geom_CircleIntersect(ax, az, LEG_P1, bx, bz, LEG_P2,
                                GEOM_LOWER, cx, cz);
}

void LegFK_Init() {}

float LegFK_HeightCm(float hip_deg, float knee_deg) {
    float cx, cz;
    if (!fk_xz(hip_deg, knee_deg, cx, cz)) return -1.0f;
    return -cz * LEG_IN2CM;
}

float LegFK_XOffsetInches(float hip_deg, float knee_deg) {
    float cx, cz;
    if (!fk_xz(hip_deg, knee_deg, cx, cz)) return 0.0f;
    return cx;
}

bool LegFK_IsReachable(float hip_deg, float knee_deg) {
    float cx, cz;
    return fk_xz(hip_deg, knee_deg, cx, cz);
}
