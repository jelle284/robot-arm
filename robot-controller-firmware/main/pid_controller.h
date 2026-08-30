#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

// PID controller parameters (constants)
typedef struct {
    float kp; // Proportional gain
    float ki; // Integral gain
    float kd; // Derivative gain
    float kt; // Tracking gain
    float output_min;
    float output_max;
    float update_period;
} pid_param_t;

// PID controller runtime data
typedef struct {
    float integral;
    float prev_error;
    float prev_measurement;
} pid_state_t;

// Initialize PID state
/**
 * @brief Initializes the PID controller state.
 *
 * This function sets up the internal state of the PID controller,
 * preparing it for use. It should be called before using the PID controller
 * to ensure all state variables are properly initialized.
 *
 * @param state Pointer to a pid_state_t structure that will hold the PID state.
 */
void pid_init(pid_state_t *state);

// Update PID controller
/**
 * @brief Updates the PID controller state and computes the control output.
 *
 * This function calculates the control signal based on the current setpoint,
 * measurement, and PID parameters. It updates the internal state of the PID controller.
 *
 * @param state      Pointer to the PID controller state structure (input/output).
 * @param params     Pointer to the PID parameters structure (input).
 * @param setpoint   The desired target value for the controller.
 * @param measurement The current measured value from the system.
 * @return           The computed control output.
 */
float pid_update(pid_state_t *state, const pid_param_t *params, float setpoint, float measurement);

#endif // PID_CONTROLLER_H