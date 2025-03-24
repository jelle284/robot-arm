#ifndef MOTION_H
#define MOTION_H
#include <stddef.h>
#include <stdint.h>

typedef struct {
    int16_t steps;
    uint16_t pulse_us;
} motion_instruction_t;

typedef struct motion_axis_t* motion_axis_handle_t;

void motion_system_init();

motion_axis_handle_t motion_axis_create(int pul_pin, int dir_pin);

void motion_axis_load_trajectory(motion_axis_handle_t axis_handle, motion_instruction_t* cmd);

void motion_axis_reset(motion_axis_handle_t axis_handle);

void motion_axis_execute(motion_axis_handle_t axis_handle);

void motion_execute_all();

void motion_axis_stop(motion_axis_handle_t axis_handle);

int motion_event_await();

int16_t motion_axis_get_feedback(motion_axis_handle_t axis_handle);

#endif // MOTION_H