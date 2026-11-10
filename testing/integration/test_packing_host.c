/** @file test_packing_host.c
 *
 * @brief PC test for app_program/common/robot_packed.c: sizes, round
 *        trips, rounding and saturation of every packed queue format.
 *
 * Build and run on a PC (not part of the firmware):
 *   gcc -std=c99 -Wall -Wextra -I../../app_program/common \
 *       test_packing_host.c ../../app_program/common/robot_packed.c \
 *       -o test_packing && ./test_packing
 */

#include <assert.h>
#include <math.h>
#include <stdio.h>

#include "robot_packed.h"

#define NEAR(a, b, tol)     (fabsf((a) - (b)) <= (tol))

static void
test_motion (void)
{
    motion_command_t    in  = {0};
    motion_command_t    out = {0};
    motion_cmd_packed_t wire;

    in.type        = MOTION_DRIVE;
    in.left_speed  = 0.153f;
    in.right_speed = -0.0474f;
    assert(pack_motion_cmd(&in, &wire));
    unpack_motion_cmd(&wire, &out);
    assert(MOTION_DRIVE == out.type);
    assert(NEAR(out.left_speed, 0.153f, 0.0006f));
    assert(NEAR(out.right_speed, -0.047f, 0.0006f));

    in.left_speed = 9.0f;                       /* saturates at +2.047 */
    assert(pack_motion_cmd(&in, &wire));
    unpack_motion_cmd(&wire, &out);
    assert(NEAR(out.left_speed, 2.047f, 0.0006f));

    in.type           = MOTION_TURN;
    in.turn_angle_deg = -90.0f;
    in.turn_id        = 31U;
    assert(pack_motion_cmd(&in, &wire));
    unpack_motion_cmd(&wire, &out);
    assert((MOTION_TURN == out.type) && (31U == out.turn_id));
    assert(NEAR(out.turn_angle_deg, -90.0f, 0.13f));
    in.turn_angle_deg = 360.0f;
    assert(pack_motion_cmd(&in, &wire));
    unpack_motion_cmd(&wire, &out);
    assert(NEAR(out.turn_angle_deg, 360.0f, 0.13f));
    in.turn_id = 32U;                           /* does not fit 5 bits */
    assert(!pack_motion_cmd(&in, &wire));
    in.turn_id = 0U;
    assert(!pack_motion_cmd(&in, &wire));

    in.type              = MOTION_MOVE_DISTANCE;
    in.left_speed        = -0.15f;
    in.right_speed       = -0.15f;
    in.target_distance_m = 0.25f;
    assert(pack_motion_cmd(&in, &wire));
    unpack_motion_cmd(&wire, &out);
    assert(NEAR(out.right_speed, -0.15f, 0.0006f));
    assert(NEAR(out.target_distance_m, 0.25f, 0.0006f));
    in.target_distance_m = 5.0f;
    assert(!pack_motion_cmd(&in, &wire));

    in.type = MOTION_BRAKE;
    assert(pack_motion_cmd(&in, &wire));
    unpack_motion_cmd(&wire, &out);
    assert((MOTION_BRAKE == out.type) && (0.0f == out.left_speed));
}

static void
test_line (void)
{
    line_event_t  in  = {0};
    line_event_t  out;
    line_packed_t wire;
    float         e;

    for (e = -1.0f; e <= 1.0f; e += 0.001f)
    {
        in.line_error = e;
        pack_line(&in, &wire);
        unpack_line(&wire, &out);
        assert(NEAR(out.line_error, e, 0.00025f));
    }

    in.line_error         = -1.0f;
    in.b_line_detected    = false;
    in.b_junction         = true;
    in.junction_action    = BARCODE_CMD_UTURN;
    in.junction_char      = 'C';
    in.b_barcode_scanning = true;
    pack_line(&in, &wire);
    unpack_line(&wire, &out);
    assert(!out.b_line_detected && out.b_junction && out.b_barcode_scanning);
    assert((BARCODE_CMD_UTURN == out.junction_action) &&
           ('C' == out.junction_char));
    assert(!out.b_barcode_valid && (0U == out.barcode));

    in.b_junction      = false;
    in.junction_char   = '\0';
    in.b_barcode_valid = true;
    in.barcode         = 'Z';
    pack_line(&in, &wire);
    unpack_line(&wire, &out);
    assert(out.b_barcode_valid && ('Z' == out.barcode) &&
           ('Z' == out.barcode_text[0]) && ('\0' == out.junction_char));
}

static void
test_obstacle (void)
{
    obstacle_event_t  in  = {0};
    obstacle_event_t  out;
    obstacle_packed_t wire;

    in.b_detected          = true;
    in.b_scan_complete     = true;
    in.action              = OBSTACLE_ACTION_TURN_RIGHT;
    in.scan_start_ms       = 100000U;
    in.center_distance_cm  = 27.4f;
    in.closest_distance_cm = 24.6f;
    in.estimated_width_cm  = 18.2f;
    in.left_clearance_cm   = 12.0f;
    in.right_clearance_cm  = 240.0f;
    in.bypass_direction    = 1;
    in.bypass_offset_cm    = -21.6f;
    in.center_angle_deg    = -15.0f;
    pack_obstacle(&in, 102345U, &wire);
    unpack_obstacle(&wire, 102345U, &out);
    assert(OBSTACLE_ACTION_TURN_RIGHT == out.action);
    assert(out.b_detected && out.b_scan_complete && !out.b_side_reading);
    assert(100000U == out.scan_start_ms);
    assert(27.0f == out.center_distance_cm);
    assert(25.0f == out.closest_distance_cm);
    assert(18.0f == out.estimated_width_cm);
    assert(240.0f == out.right_clearance_cm);
    assert((1 == out.bypass_direction) && (-22.0f == out.bypass_offset_cm));
    assert(out.center_angle_deg < 0.0f);

    in.b_side_reading   = true;
    in.side_distance_cm = 400.0f;
    in.scan_start_ms    = 0U;               /* not a scan */
    pack_obstacle(&in, 5U, &wire);
    unpack_obstacle(&wire, 5U, &out);
    assert(out.b_side_reading && (400.0f == out.side_distance_cm));
}

static void
test_terrain (void)
{
    terrain_packed_t wire;
    terrain_event_t  out;

    pack_terrain(true, 0.0456f, &wire);
    unpack_terrain(&wire, &out);
    assert(out.b_shock && NEAR(out.height_m, 0.046f, 0.0001f));
    pack_terrain(false, 0.5f, &wire);       /* saturates at 127 mm */
    unpack_terrain(&wire, &out);
    assert(!out.b_shock && NEAR(out.height_m, 0.127f, 0.0001f));
}

int
main (void)
{
    printf("sizes packed/unpacked: motion %zu/%zu line %zu/%zu "
           "terrain %zu/%zu obstacle %zu/%zu\n",
           sizeof(motion_cmd_packed_t), sizeof(motion_command_t),
           sizeof(line_packed_t), sizeof(line_event_t),
           sizeof(terrain_packed_t), sizeof(terrain_event_t),
           sizeof(obstacle_packed_t), sizeof(obstacle_event_t));
    test_motion();
    test_line();
    test_obstacle();
    test_terrain();
    printf("test_packing_host: all tests passed\n");

    return (0);
}

/*** end of file ***/
