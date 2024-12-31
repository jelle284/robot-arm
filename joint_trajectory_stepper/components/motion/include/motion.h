#ifndef MOTION_H
#define MOTION_H
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
    float angle;
    float duration;
    motion_profile_t profile;
} motion_instruction_t;

typedef struct {
    float output_position;
    float gear_ratio;
    float max_velocity;
    float max_acceleration;
} motion_axis_t;

motion_axis_t *motion_axis_create(int pul_pin, int dir_pin, int resolution);

void motion_axis_move(motion_axis_t* axis_handle, motion_instruction_t cmd);

void motion_axis_stop(motion_axis_t* axis_handle);

void motion_group_move(motion_axis_t *axis_handle[], motion_instruction_t cmd[], int group_size);

#endif // MOTION_H