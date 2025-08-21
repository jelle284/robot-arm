#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "driver/gpio.h"
#include "driver/mcpwm_prelude.h"
#include "driver/pulse_cnt.h"

typedef struct stepper_motor stepper_motor_t;
typedef stepper_motor_t* stepper_motor_handle_t;

typedef enum {
    STEPPER_MOTOR_STATUS_UNINITIALIZED = -1,
    STEPPER_MOTOR_STATUS_INITIALIZED = 0,
    STEPPER_MOTOR_STATUS_RUNNING = 1,
    STEPPER_MOTOR_STATUS_STOPPED = 2
} stepper_motor_status_t;
/**
 * @brief Initialize a stepper motor.
 *
 * @param pul_pin GPIO pin number for the pulse (step) signal.
 * @param dir_pin GPIO pin number for the direction signal.
 * @return Handle to the initialized stepper motor, or NULL on failure.
 */
stepper_motor_handle_t stepper_motor_init(gpio_num_t pul_pin, gpio_num_t dir_pin);

/**
 * @brief Start the stepper motor.
 *
 * @param handle Handle to the stepper motor.
 */
void stepper_motor_start(stepper_motor_handle_t handle);

/**
 * @brief Stop the stepper motor.
 *
 * @param handle Handle to the stepper motor.
 */
void stepper_motor_stop(stepper_motor_handle_t handle);

/**
 * @brief Set the speed of the stepper motor.
 *
 * @param handle Handle to the stepper motor.
 * @param speed_hz Desired speed in Hz. Negative values indicate reverse direction.
 */
void stepper_motor_set_speed(stepper_motor_handle_t handle, int speed_hz);

/**
 * @brief Get the current speed of the stepper motor.
 *
 * @param handle Handle to the stepper motor.
 * @return Current speed in Hz. Negative values indicate reverse direction.
 */
int stepper_motor_get_speed(stepper_motor_handle_t handle);

/**
 * @brief Get the current position of the stepper motor (in steps).
 *
 * @param handle Handle to the stepper motor.
 * @return Current position in steps.
 */
int stepper_motor_get_position(stepper_motor_handle_t handle);

/**
 * @brief Reset the position counter of the stepper motor to zero.
 *
 * @param handle Handle to the stepper motor.
 */
void stepper_motor_reset_position(stepper_motor_handle_t handle);
