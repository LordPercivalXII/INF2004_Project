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

static uint32_t g_servo_angle = ULTRASONIC_COARSE_CENTER_DEG;

void
servo_init (void)
{
#if !ROBOT_SIMULATION
    board_pwm_init(PIN_SCAN_SERVO, SERVO_PWM_CLKDIV, SERVO_PWM_WRAP);
#endif
    servo_set_angle(ULTRASONIC_COARSE_CENTER_DEG);
}

/*!
 * @brief Move the servo; the angle is limited to the scan range.
 */
void
servo_set_angle (uint32_t angle_deg)
{
#if !ROBOT_SIMULATION
    uint32_t bounded = angle_deg;

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

    board_pwm_set_level(PIN_SCAN_SERVO, SERVO_PULSE_MIN_US +
                        ((bounded * SERVO_PULSE_SPAN_US) /
                         SERVO_FULL_SCALE_DEG));
#endif
    g_servo_angle = angle_deg;
}

uint32_t
servo_get_angle (void)
{
    return (g_servo_angle);
}

/*** end of file ***/
