/** @file drv_ir_sampler.c
 *
 * @brief 1 kHz IR sampling.  Replaces the Pico SDK repeating timer
 *        (add_repeating_timer_us) with RP2040 TIMER alarm 3, installed
 *        with tk_def_int().
 *
 * Single producer (the alarm interrupt) / single consumer (line_task),
 * lock-free: only the interrupt writes g_ir_head, only the task writes
 * g_ir_tail.  The interrupt is the only code that touches the ADC once
 * sampling has started, so the kernel's "adca" driver is disabled in
 * config/config_device.h (TEAM.md section 3).
 */

#include "subsystem3.h"
#include "drv_ir_sampler.h"
#include "drv_barcode_sensor.h"
#include "../common/robot_config.h"
#include "../common/drv_board.h"

#if (0U != (IR_RING_SIZE & (IR_RING_SIZE - 1U)))
#error "IR_RING_SIZE must be a power of two"
#endif

/* RP2040 ADC (datasheet 4.9.6). */
#define IR_ADC_BASE             (0x4004C000UL)
#define IR_ADC_CS               (IR_ADC_BASE + 0x00UL)
#define IR_ADC_RESULT           (IR_ADC_BASE + 0x04UL)
#define IR_ADC_CS_EN            (1UL << 0)
#define IR_ADC_CS_START_ONCE    (1UL << 2)
#define IR_ADC_CS_READY         (1UL << 8)
#define IR_ADC_CS_AINSEL_SHIFT  (12U)
#define IR_ADC_RESULT_MASK      (0x0FFFUL)

/* RP2040 TIMER alarm 3 = interrupt 3. */
#define IR_TIMER_INTNO          (3U)
#define IR_TIMER_ALARM_BIT      (1UL << 3)
#define IR_TIMER_INT_PRIORITY   (2)
#define IR_PERIOD_US            (1000000UL / IR_SAMPLE_RATE_HZ)

typedef struct
{
    uint16_t left;
    uint16_t center;
    uint8_t  right_black;
} ir_sample_t;

static ir_sample_t       g_ir_ring[IR_RING_SIZE];
static volatile uint32_t g_ir_head     = 0U;   /* written by ISR only  */
static volatile uint32_t g_ir_tail     = 0U;   /* written by task only */
static volatile uint32_t g_ir_overruns = 0U;
static uint32_t          g_next_alarm_us = 0U;

static uint16_t
ir_adc_read (uint32_t channel)
{
    out_w(IR_ADC_CS, ((channel << IR_ADC_CS_AINSEL_SHIFT) |
                      IR_ADC_CS_EN | IR_ADC_CS_START_ONCE));

    while (0UL == (in_w(IR_ADC_CS) & IR_ADC_CS_READY))
    {
        /* ~2 us conversion at 48 MHz. */
    }

    return ((uint16_t) (in_w(IR_ADC_RESULT) & IR_ADC_RESULT_MASK));
}

/*!
 * @brief TIMER alarm 3 interrupt, every 1 ms.  No waits, ~5 us of work.
 */
static void
ir_sample_isr (UINT intno)
{
    uint32_t const head = g_ir_head;
    uint32_t const now  = board_micros();

    /* Clear, then re-arm at a fixed interval (no drift).  If we fell
       behind, restart from now instead of firing back-to-back. */
    out_w(TIMER_INTR, IR_TIMER_ALARM_BIT);
    g_next_alarm_us += IR_PERIOD_US;

    if ((int32_t) (g_next_alarm_us - now) <= 0)
    {
        g_next_alarm_us = now + IR_PERIOD_US;
    }

    out_w(TIMER_ALARM3, g_next_alarm_us);

    if ((head - g_ir_tail) >= IR_RING_SIZE)
    {
        g_ir_overruns++;            /* full: drop this sample */
    }
    else
    {
        ir_sample_t * const p_sample =
            &g_ir_ring[head & (IR_RING_SIZE - 1U)];

        p_sample->left        = ir_adc_read(ADC_CHANNEL_IR_LEFT);
        p_sample->center      = ir_adc_read(ADC_CHANNEL_IR_CENTER);
        p_sample->right_black = barcode_sensor_is_black();
        board_memory_barrier();     /* sample written before publish */
        g_ir_head = head + 1U;
    }

    ClearInt(intno);
}

/*!
 * @brief Power the ADC, set up the pins and start the 1 kHz alarm.
 *        Call once, from line_task (the ADC's owning task).
 *
 * @return E_OK, or the tk_def_int() error.
 */
ER
ir_sampler_start (void)
{
    T_DINT  dint = {0};
    ER      er;

    board_release_reset(RESETS_RESET_ADC);
    board_gpio_analog(PIN_IR_LEFT);
    board_gpio_analog(PIN_IR_CENTER);
    out_w(IR_ADC_CS, IR_ADC_CS_EN);

    while (0UL == (in_w(IR_ADC_CS) & IR_ADC_CS_READY))
    {
        /* Wait for the ADC to power up. */
    }

    g_ir_head = 0U;
    g_ir_tail = 0U;

    dint.intatr = TA_HLNG;
    dint.inthdr = (FP) ir_sample_isr;
    er = tk_def_int(IR_TIMER_INTNO, &dint);

    if (E_OK == er)
    {
        out_w(TIMER_INTR, IR_TIMER_ALARM_BIT);
        out_w(TIMER_INTE + 0x2000UL, IR_TIMER_ALARM_BIT);  /* atomic SET */
        EnableInt(IR_TIMER_INTNO, IR_TIMER_INT_PRIORITY);
        g_next_alarm_us = board_micros() + IR_PERIOD_US;
        out_w(TIMER_ALARM3, g_next_alarm_us);
    }

    return (er);
}

/*!
 * @brief Move every sample published since the last call into a window.
 */
void
ir_sampler_drain (ir_window_t * p_window)
{
    uint32_t const head = g_ir_head;
    uint32_t       tail;

    p_window->count             = 0U;
    p_window->left_sum          = 0U;
    p_window->center_sum        = 0U;
    p_window->right_black_count = 0U;

    board_memory_barrier();         /* read samples after seeing head */
    tail = g_ir_tail;

    while (tail != head)
    {
        ir_sample_t const * const p_sample =
            &g_ir_ring[tail & (IR_RING_SIZE - 1U)];

        p_window->left_sum          += p_sample->left;
        p_window->center_sum        += p_sample->center;
        p_window->right_black_count += p_sample->right_black;
        p_window->count++;
        tail++;
    }

    board_memory_barrier();
    g_ir_tail = tail;               /* give the slots back to the ISR */
}

uint32_t
ir_sampler_overruns (void)
{
    return (g_ir_overruns);
}

/*** end of file ***/
