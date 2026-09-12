#include "mixing.h"

#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/*
 * Mix a body velocity into wheel speeds, scaling both sides if either saturates.
 *
 * When a wheel would exceed wheel_v_max_mm_s, both sides are multiplied by the
 * same factor.  Clipping only the fast side would change the ratio between the
 * wheels — and so the turn the rover actually drives — with nothing in the
 * system able to notice.  Scaling keeps the ratio and gives up speed instead,
 * which the row follower can see and correct for.
 *
 * Front and rear wheel on the same side get the same value; they are wired in
 * parallel anyway.
 */
WheelSpeeds mix(float v_mm_s, float omega_deg_s, float track_width_mm, float wheel_v_max_mm_s)
{
    float omega_rad_s = omega_deg_s * (float)M_PI / 180.0f;
    float half_track_mm = track_width_mm / 2.0f;

    WheelSpeeds wheels;
    wheels.left_mm_s = v_mm_s - omega_rad_s * half_track_mm;
    wheels.right_mm_s = v_mm_s + omega_rad_s * half_track_mm;

    float peak = fmaxf(fabsf(wheels.left_mm_s), fabsf(wheels.right_mm_s));
    if (peak > wheel_v_max_mm_s) {
        float k = wheel_v_max_mm_s / peak;
        wheels.left_mm_s *= k;
        wheels.right_mm_s *= k;
    }

    return wheels;
}
