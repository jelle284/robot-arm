#ifndef MOTION_H
#define MOTION_H
#include <stddef.h>
#include <stdint.h>

#define MOTION_BUFFER_SIZE 256
#define MOTION_AXIS_NUM 6

typedef struct {
    int16_t steps;
    uint16_t pulse_us;
    uint16_t level;
} motion_instruction_t;

typedef struct {
    int32_t steps_executed;
    uint8_t current_point;
    uint8_t total_points;
} motion_execution_state_t;

typedef struct motion_axis_t* motion_axis_handle_t;

void motion_system_init();

motion_axis_handle_t motion_axis_create(int pul_pin, int dir_pin);

void motion_axis_load_trajectory(motion_axis_handle_t axis_handle, motion_instruction_t cmd);

void motion_axis_reset(motion_axis_handle_t axis_handle);

void motion_axis_execute(motion_axis_handle_t axis_handle);

void motion_execute_all();

void motion_axis_stop(motion_axis_handle_t axis_handle);

void motion_await_done();

void motion_get_feedback(int32_t *feedback);

#endif // MOTION_H