#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

#include "net_connection.h"
#include "pid_controller.h"
#include "io_expander.h"
#include "motor_comm.h"
#include "stepper_motor.h"

static const char *TAG = "main";

static const int pulse_pins[AXIS_NUM] = {26, 14, 23, 33, 18, 16};
static const int dir_pins[AXIS_NUM]   = {27, 13,  4, 25, 19, 17};

static stepper_motor_handle_t motor_handle[AXIS_NUM];

static void control_loop(void *arg);

void app_main(void)
{
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << GPIO_NUM_34),
        .mode = GPIO_MODE_INPUT,
        .intr_type = GPIO_INTR_NEGEDGE,
    };
    gpio_config(&io_conf);

    xTaskCreate(mcp23017_task, "mcp23017_task", 4096, NULL,
                configMAX_PRIORITIES - 1, &mcp23017_task_handle);
    gpio_install_isr_service(0);
    gpio_isr_handler_add(GPIO_NUM_34, mcp23017_gpio_isr_handler, NULL);

    for (int i = 0; i < AXIS_NUM; i++) {
        motor_handle[i] = stepper_motor_init(pulse_pins[i], dir_pins[i]);
        if (motor_handle[i] == NULL) {
            ESP_LOGE(TAG, "axis %d: init failed (pulse %d, dir %d)",
                     i, pulse_pins[i], dir_pins[i]);
        }
    }

    network_init();

    if (!motor_comm_init()) {
        ESP_LOGE(TAG, "motor_comm_init failed, not starting control loop");
        return;
    }

    xTaskCreate(control_loop, "control_loop", 4096, NULL, 5, NULL);
}

/*
 * The control loop owns motion. It runs on its own fixed clock and never
 * blocks on the network: every queue read below is non-blocking, so a
 * silent link means "no new target", not "no new output". The loop keeps
 * closing the loop on the last target it holds, which brings the axes to
 * rest at that position instead of freezing the last commanded speed.
 */
typedef enum {
    STATE_DISARMED,  // motors held at zero speed
    STATE_RUNNING,   // tracking setpoints from the server
    STATE_STOPPING,  // ramping the commanded speed down, then disarming
} control_state_t;

static void control_loop(void *arg)
{
    pid_state_t states[AXIS_NUM];
    int32_t target_positions[AXIS_NUM] = {0};
    float commanded[AXIS_NUM] = {0};   // last speed written to each axis
    control_state_t state = STATE_DISARMED;
    bool stale_logged = false;
    int64_t stop_elapsed_ms = 0;

    for (int i = 0; i < AXIS_NUM; i++) {
        pid_init(&states[i]);
    }

    TickType_t last_wake = xTaskGetTickCount();

    for (;;) {
        ParamsData cfg;
        motor_comm_get_params(&cfg);
        const pid_param_t params = {
            .kp = cfg.kp,
            .ki = cfg.ki,
            .kd = cfg.kd,
            .kt = 1.0f,
            .output_min = (float)cfg.output_min,
            .output_max = (float)cfg.output_max,
            .update_period = CONTROL_PERIOD_MS / 1000.0f,
        };

        const bool active = motor_comm_is_active();
        const int64_t setpoint_age = motor_comm_setpoint_age_ms();

        // --- state transitions ---------------------------------------
        switch (state) {
        case STATE_DISARMED:
            if (active) {
                // Take the current position as the target so nothing jumps.
                for (int i = 0; i < AXIS_NUM; i++) {
                    pid_init(&states[i]);
                    commanded[i] = 0.0f;
                    target_positions[i] = (motor_handle[i] != NULL)
                                        ? stepper_motor_get_position(motor_handle[i])
                                        : 0;
                }
                stale_logged = false;
                state = STATE_RUNNING;
                ESP_LOGI(TAG, "control running");
            }
            break;

        case STATE_RUNNING:
            if (!active) {
                state = STATE_STOPPING;
                stop_elapsed_ms = 0;
                ESP_LOGI(TAG, "deactivated, ramping to a stop");
            } else if (setpoint_age > SETPOINT_TIMEOUT_MS) {
                // Link is gone. Disarm, but via a ramp, not a cliff.
                motor_comm_deactivate("setpoint timeout");
                state = STATE_STOPPING;
                stop_elapsed_ms = 0;
            } else if (setpoint_age > SETPOINT_STALE_MS && !stale_logged) {
                // Degraded, not lost: keep controlling toward the last
                // target. The PID decelerates as the error closes.
                ESP_LOGW(TAG, "setpoints stale (%lld ms), holding last target",
                         setpoint_age);
                stale_logged = true;
            } else if (setpoint_age <= SETPOINT_STALE_MS) {
                stale_logged = false;
            }
            break;

        case STATE_STOPPING:
            stop_elapsed_ms += CONTROL_PERIOD_MS;
            if (stop_elapsed_ms >= STOP_RAMP_MS) {
                state = STATE_DISARMED;
                ESP_LOGI(TAG, "stopped");
            }
            break;
        }

        // --- fetch the newest target, if one arrived -----------------
        SetpointsMessage msg;
        if (state == STATE_RUNNING &&
            xQueueReceive(xSetpointsQueue, &msg, 0) == pdTRUE) {
            for (int i = 0; i < AXIS_NUM; i++) {
                target_positions[i] = msg.data.positions[i];
            }
        }

        // --- drive the axes ------------------------------------------
        // Ramp step: full scale to zero over STOP_RAMP_MS.
        const float ramp_step = (params.output_max > 0.0f)
                              ? params.output_max * CONTROL_PERIOD_MS / (float)STOP_RAMP_MS
                              : 0.0f;

        FeedbackMessage feedback = {0};

        for (int i = 0; i < AXIS_NUM; i++) {
            if (motor_handle[i] == NULL) {
                continue;
            }
            int32_t position = stepper_motor_get_position(motor_handle[i]);
            feedback.data.positions[i] = position;

            switch (state) {
            case STATE_RUNNING:
                commanded[i] = pid_update(&states[i], &params,
                                          (float)target_positions[i],
                                          (float)position);
                break;

            case STATE_STOPPING:
                if (commanded[i] > ramp_step)       commanded[i] -= ramp_step;
                else if (commanded[i] < -ramp_step) commanded[i] += ramp_step;
                else                                commanded[i] = 0.0f;
                break;

            case STATE_DISARMED:
                commanded[i] = 0.0f;
                break;
            }

            stepper_motor_set_speed(motor_handle[i], (int)commanded[i]);
        }

        // The UDP task adds the header and paces the actual sending, so
        // feedback keeps being produced regardless of link state.
        xQueueOverwrite(xFeedbackQueue, &feedback);

        xTaskDelayUntil(&last_wake, pdMS_TO_TICKS(CONTROL_PERIOD_MS));
    }
}