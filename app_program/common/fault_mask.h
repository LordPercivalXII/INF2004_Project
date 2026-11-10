/** @file fault_mask.h
 *
 * @brief Shared 5-bit fault bitmask (Tribunal 1 curveball: minimum-bit
 *        fault flags, safe against read-modify-write races).
 *
 * Producers (tasks AND interrupt handlers, on either core) latch bits
 * with fault_mask_set().  The controller takes an atomic snapshot-and-
 * clear with fault_mask_take() once per telemetry publish, so every
 * telemetry packet reports the faults raised since the previous one.
 *
 * Every function is callable from task and from handler context: the
 * update is done inside crit_enter()/crit_exit() (see crit_sect.h).
 *
 *   bit  name               set by
 *   ---  -----------------  ------------------------------------------
 *   0    FAULT_BIT_IMU      S4 imu_task: IMU read / range failure
 *   1    FAULT_BIT_ENCODER  S2 motion_task: encoder stall watchdog
 *   2    FAULT_BIT_LINE     S3 GP28 edge ISR (edge buffer overflow);
 *                           controller (line not reacquired)
 *   3    FAULT_BIT_NO_ECHO  S5 obstacle_task (30 ms echo timeout);
 *                           S5 GP16 echo ISR (echo buffer overflow)
 *   4    FAULT_BIT_COMMS    S1: MQTT disconnect or telemetry buffer full
 */

#ifndef FAULT_MASK_H
#define FAULT_MASK_H

#include <stdint.h>

#define FAULT_MASK_BITS     (5U)
#define FAULT_MASK_ALL      (0x1FU)

#define FAULT_BIT_IMU       (0x01U)
#define FAULT_BIT_ENCODER   (0x02U)
#define FAULT_BIT_LINE      (0x04U)
#define FAULT_BIT_NO_ECHO   (0x08U)
#define FAULT_BIT_COMMS     (0x10U)

void    fault_mask_set(uint8_t bits);
void    fault_mask_clear(uint8_t bits);
uint8_t fault_mask_read(void);
uint8_t fault_mask_take(uint8_t bits);

#endif /* FAULT_MASK_H */

/*** end of file ***/
