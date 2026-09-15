#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

#include "net_connection.h"
#include "pid_controller.h"
#include "io_expander.h"
#include "uros_routine.h"


static const char *TAG = "main";

const int pulse_pins[] = {26, 14, 23, 33, 18, 16};
const int dir_pins[] = {27, 13, 4, 25, 19, 17};

stepper_motor_handle_t motor_handle[AXIS_NUM];
int target_positions[AXIS_NUM] = {0};

void control_loop(void* arg);

// Main application
void app_main() 
{
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << GPIO_NUM_34),
        .mode = GPIO_MODE_INPUT,
        .intr_type = GPIO_INTR_NEGEDGE,
    };

    gpio_config(&io_conf);
    xTaskCreate(mcp23017_task, "mcp23017_task", 4096, NULL, configMAX_PRIORITIES - 1, &mcp23017_task_handle);
    gpio_install_isr_service(0);
    gpio_isr_handler_add(GPIO_NUM_34, mcp23017_gpio_isr_handler, NULL);

    for (int i = 0; i < AXIS_NUM; i++) {
        motor_handle[i] = stepper_motor_init(pulse_pins[i], dir_pins[i]);
        if (motor_handle[i] == NULL) {
            ESP_LOGE(TAG, "Failed to initialize stepper motor on pulse pin %d, dir pin %d", pulse_pins[i], dir_pins[i]);
        } else {
            ESP_LOGI(TAG, "Stepper motor initialized on pulse pin %d, dir pin %d", pulse_pins[i], dir_pins[i]);
        }
    }
    network_init();
    xTaskCreate(micro_ros_task, "micro_ros_task", 4096, NULL, 5, NULL);
    xTaskCreate(control_loop, "control_loop", 4096, NULL, 5, NULL);

}

/**/
void control_loop(void* arg) {
    pid_param_t params;
    params.kp = 1.0f;
    params.ki = 0.2f;
    params.kd = 0.0f;
    params.kt = 1.0f;
    params.output_min = -5000.0f;
    params.output_max = 5000.0f;
    params.update_period = 0.01f;
    pid_state_t states[AXIS_NUM];
    for (size_t i = 0; i < AXIS_NUM; ++i) {
        pid_init(&states[i]);
    }
    TickType_t ticks = xTaskGetTickCount();
    for (;;) {
        for (size_t i = 0; i < AXIS_NUM; ++i) {
            if (motor_handle[i] == NULL) {
                continue;
            }
            float measurement = (float)stepper_motor_get_position(motor_handle[i]);
            float setpoint = (float)target_positions[i];
            float control_signal = pid_update(&states[i], &params, setpoint, measurement);
            stepper_motor_set_speed(motor_handle[i], (int)control_signal);
        }
        xTaskDelayUntil(&ticks, pdMS_TO_TICKS(10));
    }
}
