/** @file drv_ultrasonic.c
 *
 * @brief Ultrasonic driver.  The echo callback measures the pulse width
 *        in microseconds and sends it to obstacle_task with
 *        tk_snd_mbf(..., TMO_POL) (was xQueueSendFromISR).
 */

#include "subsystem5.h"
#include "drv_ultrasonic.h"
#include "drv_servo.h"
#include "../common/robot_config.h"
#include "../common/drv_board.h"
#include "../common/drv_gpio_irq.h"

#define US_TRIGGER_SETTLE_US    (2U)
#define US_TRIGGER_PULSE_US     (10U)
#define US_CM_PER_US_ROUND_TRIP (0.0343f * 0.5f)
#define US_SIM_PERIOD           (12U)
#define US_SIM_HIT_CM           (24.0f)
#define US_SIM_LEFT_CM          (90.0f)
#define US_SIM_RIGHT_CM         (60.0f)
#define US_SIM_CENTER_CM        (100.0f)

static volatile uint32_t g_echo_drops         = 0U;

#if !ROBOT_SIMULATION
static volatile uint32_t g_echo_rise_us       = 0U;
static volatile bool     gb_waiting_for_fall  = false;
static volatile bool     gb_measuring         = false;
static uint32_t          g_last_trigger_ms    = 0U;
static bool              gb_trigger_started   = false;

/*!
 * @brief Echo pin callback (interrupt context).
 */
static void
ultrasonic_echo_isr (uint32_t pin, uint32_t events)
{
    uint32_t width_us;

    if ((PIN_ULTRASONIC_ECHO == pin) && gb_measuring)
    {
        if (0UL != (events & GPIO_IRQ_EDGE_RISE))
        {
            g_echo_rise_us      = board_micros();
            gb_waiting_for_fall = true;
        }
        else if ((0UL != (events & GPIO_IRQ_EDGE_FALL)) &&
                 gb_waiting_for_fall)
        {
            width_us = board_micros() - g_echo_rise_us;
            gb_waiting_for_fall = false;

            if (E_OK != tk_snd_mbf(gh_echo_mbf, &width_us,
                                   (INT) sizeof(width_us), TMO_POL))
            {
                g_echo_drops++;
            }
        }
        else
        {
            /* Stray fall without a rise. */
        }
    }
}
#endif /* !ROBOT_SIMULATION */

/*!
 * @brief Configure trigger/echo pins and the echo interrupt.
 *
 * @return true if the echo interrupt was registered.
 */
bool
ultrasonic_init (void)
{
    bool b_ok = true;

#if !ROBOT_SIMULATION
    board_gpio_output(PIN_ULTRASONIC_TRIGGER, false);
    board_gpio_input(PIN_ULTRASONIC_ECHO, BOARD_PULL_NONE);
    b_ok = gpio_irq_register(PIN_ULTRASONIC_ECHO,
                             (GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL),
                             ultrasonic_echo_isr);
#endif

    return (b_ok);
}

#if !ROBOT_SIMULATION
/*!
 * @brief Keep at least ULTRASONIC_MIN_PING_INTERVAL_MS between pings so
 *        old echoes have died away.
 */
static void
ultrasonic_wait_ping_interval (void)
{
    uint32_t elapsed_ms;

    if (gb_trigger_started)
    {
        elapsed_ms = board_millis() - g_last_trigger_ms;

        if (elapsed_ms < ULTRASONIC_MIN_PING_INTERVAL_MS)
        {
            (void) tk_dly_tsk((TMO) (ULTRASONIC_MIN_PING_INTERVAL_MS -
                                     elapsed_ms));
        }
    }

    g_last_trigger_ms  = board_millis();
    gb_trigger_started = true;
}
#endif

/*!
 * @brief Ping once and return the distance, clamped to the sensor range.
 *
 * @param[in] scan_number  Used only by the simulation.
 */
float
ultrasonic_measure_cm (uint32_t scan_number)
{
    float distance_cm = ULTRASONIC_MAX_DISTANCE_CM;

#if ROBOT_SIMULATION
    uint32_t const angle = servo_get_angle();

    if ((0U == (scan_number % US_SIM_PERIOD)) &&
        (ULTRASONIC_COARSE_CENTER_DEG == angle))
    {
        distance_cm = US_SIM_HIT_CM;
    }
    else if (angle < ULTRASONIC_COARSE_CENTER_DEG)
    {
        distance_cm = US_SIM_LEFT_CM;
    }
    else if (angle > ULTRASONIC_COARSE_CENTER_DEG)
    {
        distance_cm = US_SIM_RIGHT_CM;
    }
    else
    {
        distance_cm = US_SIM_CENTER_CM;
    }
#else
    uint32_t echo_width_us = 0U;
    INT      received;

    (void) scan_number;

    /* Flush stale echoes. */
    while (tk_rcv_mbf(gh_echo_mbf, &echo_width_us, TMO_POL) > 0)
    {
        echo_width_us = 0U;
    }

    gb_waiting_for_fall = false;
    ultrasonic_wait_ping_interval();
    gb_measuring = true;

    board_gpio_write(PIN_ULTRASONIC_TRIGGER, false);
    board_delay_us(US_TRIGGER_SETTLE_US);
    board_gpio_write(PIN_ULTRASONIC_TRIGGER, true);
    board_delay_us(US_TRIGGER_PULSE_US);
    board_gpio_write(PIN_ULTRASONIC_TRIGGER, false);

    received = tk_rcv_mbf(gh_echo_mbf, &echo_width_us,
                          (TMO) ULTRASONIC_ECHO_TIMEOUT_MS);
    gb_measuring = false;

    if ((received > 0) && (0U != echo_width_us))
    {
        distance_cm = (float) echo_width_us * US_CM_PER_US_ROUND_TRIP;

        if (distance_cm < ULTRASONIC_MIN_DISTANCE_CM)
        {
            distance_cm = ULTRASONIC_MIN_DISTANCE_CM;
        }
        else if (distance_cm > ULTRASONIC_MAX_DISTANCE_CM)
        {
            distance_cm = ULTRASONIC_MAX_DISTANCE_CM;
        }
        else
        {
            /* In range. */
        }
    }
#endif

    return (distance_cm);
}

/*!
 * @brief Read and clear the count of echoes dropped (buffer full).
 */
uint32_t
ultrasonic_take_drops (void)
{
    uint32_t const drops = g_echo_drops;

    g_echo_drops = 0U;

    return (drops);
}

/*** end of file ***/
