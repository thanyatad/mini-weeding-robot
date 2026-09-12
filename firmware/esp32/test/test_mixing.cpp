/*
 * Golden-vector test for the C++ half of the skid-steer mixing.
 *
 * Reads config/drive_mixing_vectors.csv — the same table
 * tests/unit/test_drive_mixing.py reads — and holds src/mixing.cpp to it.  If
 * one side fails, the two implementations have drifted apart.  Never edit the
 * table to make a test pass — fix the implementation.
 *
 *   pio test -e native -d firmware/esp32
 *
 * This file deliberately does NOT contain the mixing formula.  It links against
 * src/mixing.cpp (see build_src_filter in platformio.ini).  A test carrying its
 * own copy of the formula would prove the two languages agree with the test
 * file, not with the firmware — which is the exact drift the table exists to
 * catch.
 */

#include <math.h>
#include <stdio.h>
#include <string.h>

#include <unity.h>

#include "mixing.h"

#define MAX_VECTORS 64
#define TOLERANCE_MM_S 0.01f
#define EXPECTED_VECTORS 12
#define CSV_HEADER "v_mm_s,omega_deg_s,v_left_mm_s,v_right_mm_s"

typedef struct {
    float v_mm_s;
    float omega_deg_s;
    float expect_left_mm_s;
    float expect_right_mm_s;
} GoldenVector;

static GoldenVector g_vectors[MAX_VECTORS];
static int g_count = -1;
static const char *g_csv_path = NULL;

/*
 * Relative to the PlatformIO project directory (firmware/esp32), which is the
 * working directory the test runner starts the binary in.  Measured, not
 * assumed: running `pio test -e native -d firmware/esp32` from the repo root
 * still resolves this path, so the runner has chdir'd into the project.
 */
#define GOLDEN_CSV_PATH "../../config/drive_mixing_vectors.csv"

static int is_skippable(const char *line)
{
    while (*line == ' ' || *line == '\t') {
        line++;
    }
    return *line == '\0' || *line == '\r' || *line == '\n' || *line == '#';
}

/* Parse the table.  Returns the number of data rows, or -1 if it could not be
 * read at all — a table the test cannot find must fail, not pass with 0 rows. */
static int load_golden_vectors(void)
{
    g_csv_path = GOLDEN_CSV_PATH;
    FILE *fp = fopen(g_csv_path, "r");
    if (fp == NULL) {
        return -1;
    }

    char line[512];
    int seen_header = 0;
    int count = 0;

    while (fgets(line, sizeof(line), fp) != NULL) {
        if (is_skippable(line)) {
            continue;
        }
        if (!seen_header) {
            char *nl = strpbrk(line, "\r\n");
            if (nl != NULL) {
                *nl = '\0';
            }
            if (strcmp(line, CSV_HEADER) != 0) {
                fclose(fp);
                return -1;
            }
            seen_header = 1;
            continue;
        }
        if (count >= MAX_VECTORS) {
            fclose(fp);
            return -1;
        }
        GoldenVector *v = &g_vectors[count];
        if (sscanf(line, "%f,%f,%f,%f", &v->v_mm_s, &v->omega_deg_s, &v->expect_left_mm_s,
                   &v->expect_right_mm_s) != 4) {
            fclose(fp);
            return -1;
        }
        count++;
    }

    fclose(fp);
    return seen_header ? count : -1;
}

void setUp(void) {}
void tearDown(void) {}

static void test_the_table_is_actually_read(void)
{
    TEST_ASSERT_MESSAGE(g_count >= 0, "could not read config/drive_mixing_vectors.csv");
    TEST_ASSERT_EQUAL_INT_MESSAGE(EXPECTED_VECTORS, g_count,
                                  "golden table did not have the expected number of rows");
}

static void test_golden_vectors(void)
{
    TEST_ASSERT_EQUAL_INT_MESSAGE(EXPECTED_VECTORS, g_count, "golden table was not loaded");

    for (int i = 0; i < g_count; i++) {
        const GoldenVector *g = &g_vectors[i];
        WheelSpeeds wheels = mix(g->v_mm_s, g->omega_deg_s, TRACK_WIDTH_MM, WHEEL_V_MAX_MM_S);

        char message[160];
        snprintf(message, sizeof(message), "row %d: mix(%.3f, %.3f) left", i + 1, (double)g->v_mm_s,
                 (double)g->omega_deg_s);
        TEST_ASSERT_FLOAT_WITHIN_MESSAGE(TOLERANCE_MM_S, g->expect_left_mm_s, wheels.left_mm_s,
                                         message);

        snprintf(message, sizeof(message), "row %d: mix(%.3f, %.3f) right", i + 1,
                 (double)g->v_mm_s, (double)g->omega_deg_s);
        TEST_ASSERT_FLOAT_WITHIN_MESSAGE(TOLERANCE_MM_S, g->expect_right_mm_s, wheels.right_mm_s,
                                         message);
    }
}

/* The trap the (320, 40) row exists to catch: clipping only the fast wheel
 * silently changes the turn rate.  Scaling both keeps v_right / v_left. */
static void test_saturation_scales_both_sides_instead_of_clipping_one(void)
{
    WheelSpeeds unsaturated = mix(320.0f, 40.0f, TRACK_WIDTH_MM, 1.0e9f);
    WheelSpeeds saturated = mix(320.0f, 40.0f, TRACK_WIDTH_MM, WHEEL_V_MAX_MM_S);

    TEST_ASSERT_FLOAT_WITHIN(TOLERANCE_MM_S, WHEEL_V_MAX_MM_S, saturated.right_mm_s);
    /* A clip would have left the inner wheel at its unsaturated value. */
    TEST_ASSERT_TRUE(saturated.left_mm_s < unsaturated.left_mm_s);
    /* Both sides moved by exactly the same factor. */
    TEST_ASSERT_FLOAT_WITHIN(1.0e-4f, saturated.right_mm_s / unsaturated.right_mm_s,
                             saturated.left_mm_s / unsaturated.left_mm_s);
}

static void test_positive_omega_turns_left(void)
{
    WheelSpeeds wheels = mix(100.0f, 20.0f, TRACK_WIDTH_MM, WHEEL_V_MAX_MM_S);
    TEST_ASSERT_TRUE(wheels.right_mm_s > wheels.left_mm_s);
}

int main(void)
{
    g_count = load_golden_vectors();
    printf("golden table: %s (%d rows)\n", g_csv_path ? g_csv_path : "NOT FOUND", g_count);

    UNITY_BEGIN();
    RUN_TEST(test_the_table_is_actually_read);
    RUN_TEST(test_golden_vectors);
    RUN_TEST(test_saturation_scales_both_sides_instead_of_clipping_one);
    RUN_TEST(test_positive_omega_turns_left);
    return UNITY_END();
}
