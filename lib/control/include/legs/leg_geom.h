#pragma once
#include <cstdint>
#include <math.h>

// leg_geom.h — shared five-bar geometry for FK (leg_fk.cpp) and IK (leg_ik.cpp)
// Reference: programming/kinematics/leg_kinematics.py

// Link lengths (inches)
static constexpr float LEG_R1 = 3.376f;  // OA: motor holder arm  (hip motor)
static constexpr float LEG_R2 = 5.257f;  // OB: upper arm         (knee motor)
static constexpr float LEG_P1 = 4.507f;  // AC: small passive link
static constexpr float LEG_P2 = 5.652f;  // BC: passive link

static constexpr float LEG_DEG2RAD = 0.017453292519943295f;
static constexpr float LEG_RAD2DEG = 57.29577951308232f;
static constexpr float LEG_IN2CM   = 2.54f;

// Which of the two circle-circle intersections to take.
enum GeomSide : uint8_t {
    GEOM_LOWER,  // smaller z (wheel hanging below chassis) — FK
    GEOM_LEFT,   // smaller x (arm tip backward)             — IK, upper arm B
    GEOM_RIGHT   // larger  x (arm tip forward)              — IK, holder arm A
};

// Intersect circle(c1,r1) with circle(c2,r2) in the X-Z plane; select one
// intersection by `side`. Returns false if the circles don't intersect.
static inline bool Geom_CircleIntersect(
    float c1x, float c1z, float r1,
    float c2x, float c2z, float r2,
    GeomSide side, float &ox, float &oz)
{
    float dx = c2x - c1x;
    float dz = c2z - c1z;
    float d  = sqrtf(dx*dx + dz*dz);

    if (d < 1e-6f)                    return false;  // coincident centers
    if (d > r1 + r2 + 1e-4f)          return false;  // too far apart
    if (d < fabsf(r1 - r2) - 1e-4f)   return false;  // one inside the other

    float a = (r1*r1 + d*d - r2*r2) / (2.0f * d);
    float h = sqrtf(fmaxf(r1*r1 - a*a, 0.0f));

    float dhx = dx / d, dhz = dz / d;
    float mx = c1x + a * dhx;
    float mz = c1z + a * dhz;

    // perpendicular (90° CCW from d-hat): (-dhz, dhx)
    float px = mx - h * dhz, pz = mz + h * dhx;   // "+" branch
    float nx = mx + h * dhz, nz = mz - h * dhx;   // "-" branch

    bool take_p;
    switch (side) {
        case GEOM_LOWER: take_p = (pz <= nz); break;
        case GEOM_LEFT:  take_p = (px <= nx); break;
        default:         take_p = (px >= nx); break;  // GEOM_RIGHT
    }
    if (take_p) { ox = px; oz = pz; }
    else        { ox = nx; oz = nz; }
    return true;
}
