#ifndef MIXING_H
#define MIXING_H

/*
 * Skid-steer mixing: (v, omega) -> left and right wheel speeds.
 *
 * This formula is implemented twice in places that cannot share code — here,
 * and in Python in controller/rover/mixing.py.  Both sides are held to the same
 * table, config/drive_mixing_vectors.csv, which is the single source of truth.
 *
 * Deliberately free of any Arduino / ESP-IDF include so it compiles in the
 * native environment.  firmware/esp32/README.md: a mixing that needs
 * analogWrite() is not a pure function and cannot be tested across languages.
 */

/*
 * These mirror config/rover.yaml and must not drift from it.  Nothing in C can
 * read the YAML, so tests/unit/test_drive_mixing.py parses the two #defines
 * below and asserts they still equal the config.  The day track width changes,
 * that test fails instead of the firmware quietly mixing for the old chassis.
 */
#define TRACK_WIDTH_MM 120.0f   /* config/rover.yaml rover.track_width_mm */
#define WHEEL_V_MAX_MM_S 202.0f /* config/rover.yaml rover.drive.wheel_v_max_mm_s */

/* Commanded surface speed of each side, mm/s.  Not measured — no encoders. */
typedef struct {
    float left_mm_s;
    float right_mm_s;
} WheelSpeeds;

/*
 * Mix a body velocity into wheel speeds, scaling both sides if either saturates.
 *
 * Sign convention (hardware/mechanical/coordinate-frames.md): v > 0 forward,
 * omega > 0 turns left (CCW, z up), so the right wheel runs faster.
 */
WheelSpeeds mix(float v_mm_s, float omega_deg_s, float track_width_mm, float wheel_v_max_mm_s);

#endif /* MIXING_H */
