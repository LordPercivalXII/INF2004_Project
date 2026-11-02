/** @file drv_servo.c
 *
 * @brief Servo driver.  Called only from obstacle_task.
 *
 * 125 MHz / 125 = 1 MHz counter, wrap 19999 -> 50 Hz, 1 count = 1 us.
 */

#include "drv_servo.h"
#include "../common/robot_config.h"
#include "../common/drv_board.h"

#define SERVO_PWM_CLKDIV        (125U)
#define SERVO_PWM_WRAP          (19999U)
#define SERVO_PULSE_MIN_US      (500U)
#define SERVO_PULSE_SPAN_US     (2000U)
#define SERVO_FULL_SCALE_DEG    (180U)
#define SERVO_STEP_DEG          (4U)    /* 4 deg per 20 ms = 200 deg/s */
#define SERVO_STEP_MS           (20)

static uint32_t g_servo_angle = ULTRASONIC_COARSE_CENTER_DEG;

#if S5_ULTRASONIC_HW
static void
servo_write (uint32_t angle_deg)
{
    board_pwm_set_level(PIN_SCAN_SERVO, SERVO_PULSE_MIN_US +
                        ((angle_deg * SERVO_PULSE_SPAN_US) /
                         SERVO_FULL_SCALE_DEG));
}
#endif

void
servo_init (void)
{
#if S5_ULTRASONIC_HW
    board_pwm_init(PIN_SCAN_SERVO, SERVO_PWM_CLKDIV, SERVO_PWM_WRAP);
    servo_write(ULTRASONIC_COARSE_CENTER_DEG);
#endif
    g_servo_angle = ULTRASONIC_COARSE_CENTER_DEG;
}

/*!
 * @brief Move the servo at a limited speed, so it does not throw the
 *        sensor; blocks until there.  The angle is limited to the scan
 *        range.
 */
void
servo_set_angle (uint32_t angle_deg)
{
#if S5_ULTRASONIC_HW
    uint32_t bounded = angle_deg;
    uint32_t current = g_servo_angle;

    if (bounded < ULTRASONIC_COARSE_LEFT_DEG)
    {
        bounded = ULTRASONIC_COARSE_LEFT_DEG;
    }
    else if (bounded > ULTRASONIC_COARSE_RIGHT_DEG)
    {
        bounded = ULTRASONIC_COARSE_RIGHT_DEG;
    }
    else
    {
        /* In range. */
    }

    while (current != bounded)
    {
        if (current < bounded)
        {
            current = ((bounded - current) > SERVO_STEP_DEG)
                      ? (current + SERVO_STEP_DEG) : bounded;
        }
        else
        {
            current = ((current - bounded) > SERVO_STEP_DEG)
                      ? (current - SERVO_STEP_DEG) : bounded;
        }

        servo_write(current);
        (void) tk_dly_tsk(SERVO_STEP_MS);
    }

    g_servo_angle = bounded;
#else
    g_servo_angle = angle_deg;
#endif
}

uint32_t
servo_get_angle (void)
{
    return (g_servo_angle);
}

/*** end of file ***/
