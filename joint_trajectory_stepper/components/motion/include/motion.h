#ifndef MOTION_H
#define MOTION_H
#include <stddef.h>
#include <stdint.h>

typedef enum {
    MOTION_PROFILE_LINEAR,
    MOTION_PROFILE_SCURVE
} motion_profile_t;

typedef enum {
    MOTION_STATUS_READY,
    MOTION_STATUS_BUSY,
    MOTION_STATUS_ERR
} motion_status_t;

typedef struct {
    int32_t steps;
    uint16_t pulse_us;
} motion_instruction_t;

typedef struct {
    motion_status_t status;
    uint64_t step_count;
} motion_axis_t;

motion_axis_t *motion_axis_create(int pul_pin, int dir_pin);

void motion_axis_reset(motion_axis_t* axis_handle);

void motion_axis_move(motion_axis_t* axis_handle, motion_instruction_t cmd);

void motion_axis_stop(motion_axis_t* axis_handle);

void motion_group_move(motion_axis_t *axis_handle[], motion_instruction_t cmd[], int group_size);

void motion_event_await(motion_axis_t* axis_handle[], size_t group_size);

void motion_system_init();
#endif // MOTION_H