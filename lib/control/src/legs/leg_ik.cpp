#include "legs/leg_ik.h"
#include "legs/leg_geom.h"

// IK topology (physically confirmed, see leg_kinematics.py):
//   A (motor holder arm, R1) always on +X side (forward)  -> GEOM_RIGHT
//   B (upper arm, R2)        always on -X side (backward) -> GEOM_LEFT
//
//   A = circle(O, R1) ∩ circle(C, P1),  theta1 = atan2(Az, Ax)
//   B = circle(O, R2) ∩ circle(C, P2),  theta2 = atan2(Bx, -Bz)

bool LegIK_FromXHCm(float x_cm, float h_cm, float &theta1_deg, float &theta2_deg) {
    float cx = x_cm / LEG_IN2CM;
    float cz = -h_cm / LEG_IN2CM;   // z = -height (wheel below chassis)

    float ax, az, bx, bz;
    if (!Geom_CircleIntersect(0, 0, LEG_R1, cx, cz, LEG_P1, GEOM_RIGHT, ax, az))
        return false;
    if (!Geom_CircleIntersect(0, 0, LEG_R2, cx, cz, LEG_P2, GEOM_LEFT, bx, bz))
        return false;

    theta1_deg = atan2f(az, ax)  * LEG_RAD2DEG;
    theta2_deg = atan2f(bx, -bz) * LEG_RAD2DEG;
    return true;
}

bool LegIK_FromHeightCm(float h_cm, float &theta1_deg, float &theta2_deg) {
    return LegIK_FromXHCm(0.0f, h_cm, theta1_deg, theta2_deg);
}
