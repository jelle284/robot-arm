#include "pid_controller.h"

void pid_init(pid_state_t *state) {
    state->integral = 0.0f;
    state->prev_error = 0.0f;
}

float pid_update(pid_state_t *state, const pid_param_t *params, float setpoint, float measurement) {
    float error = setpoint - measurement;
    state->integral += error * params->update_period;
    float derivative = (error - state->prev_error) / params->update_period;
    state->prev_error = error;

    float output = params->kp * error
                 + params->ki * state->integral
                 + params->kd * derivative;

    
    // Tracking anti-windup
    float output_clamped = output;
    if (output > params->output_max) output_clamped = params->output_max;
    else if (output < params->output_min) output_clamped = params->output_min;

    float integral_correction = (output - output_clamped) * params->update_period;
    state->integral -= integral_correction;

    return output_clamped;
}