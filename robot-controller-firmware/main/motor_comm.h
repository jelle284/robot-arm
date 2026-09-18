#pragma once

#include "motor_comm_api.h"

// Starts the TCP session task and the UDP streaming task. Returns false
// if the queues or tasks could not be created.
bool motor_comm_init(void);

// True once the server has activated us over TCP.
// The control loop must hold the motors still whenever this is false.
bool motor_comm_is_active(void);

// Milliseconds since the last valid setpoint packet arrived, or INT64_MAX
// if none has arrived since activation. The control loop uses this to
// judge the health of the link; it never blocks waiting for packets.
int64_t motor_comm_setpoint_age_ms(void);

// Thread-safe snapshot of the parameters last pushed by the server.
void motor_comm_get_params(ParamsData *out);

// Locally initiated disarm (link timeout, fault). Clears the active flag
// and tells the server, so the UI reflects it. Safe to call repeatedly.
void motor_comm_deactivate(const char *reason);