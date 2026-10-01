#ifndef BUDDY2_MOTION_INTERFACE_H
#define BUDDY2_MOTION_INTERFACE_H

#include <stdbool.h>
#include "common_types.h"

void Buddy2Motion_CreateTask(void);
bool Buddy2Motion_SendCommand(const MotionCommand *command);

#endif