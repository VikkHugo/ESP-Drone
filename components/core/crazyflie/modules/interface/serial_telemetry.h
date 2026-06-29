#ifndef SERIAL_TELEMETRY_H_
#define SERIAL_TELEMETRY_H_

#include <stdbool.h>
#include "stabilizer_types.h"

void serialTelemetryInit(void);
bool serialTelemetryIsInit(void);
void serialTelemetryPush(const sensorData_t* sensorData, const state_t* state);

#endif
