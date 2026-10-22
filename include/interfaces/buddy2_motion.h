#ifndef BUDDY2_MOTION_INTERFACE_H
#define BUDDY2_MOTION_INTERFACE_H

#include <stdbool.h>
#include "common_types.h"

void Buddy2Motion_CreateTask(void);
/* Drive speeds are m/s; turns use signed degrees (positive is left) and a nonzero turn_id. */
bool Buddy2Motion_SendCommand(const MotionCommand *command);

#endif