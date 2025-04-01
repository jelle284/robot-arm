#ifndef MOTION_H
#define MOTION_H
#include <stddef.h>
#include <stdint.h>

#define MOTION_AXIS_NUM 6

typedef enum {
    MS_UNINITIALIZED,
    MS_IDLE,
    MS_LOADING,
    MS_EXECUTING,
    MS_COMPLETE
} motion_state_t;

typedef struct {
    int32_t steps_executed[MOTION_AXIS_NUM];
    uint8_t current_point;
    uint8_t total_points;
} motion_feedback_t;

void motion_system_init(const int* pul_pins, const int* dir_pins);

void motion_load_trajectory(int16_t *steps, uint32_t duration_ms);

void motion_execute();

void motion_stop();

void motion_await_done();

int motion_get_feedback(motion_feedback_t *state);

#endif // MOTION_H