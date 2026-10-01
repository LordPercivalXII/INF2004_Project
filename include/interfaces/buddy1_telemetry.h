#ifndef BUDDY1_TELEMETRY_INTERFACE_H
#define BUDDY1_TELEMETRY_INTERFACE_H

#include <stdbool.h>
#include "common_types.h"

void Buddy1Telemetry_CreateTask(void);
bool Buddy1Telemetry_SubmitCommand(const ControlCommand *command);
bool Buddy1Telemetry_Publish(const TelemetryMessage *message);

#endif