#include "stepper_motor.h"
#include "esp_log.h"

#define PULSE_WIDTH_US 50 // Pulse width in microseconds

static const char *TAG = "stepper_motor";
static size_t handle_count = 0;

static const int group_map[6] = {
    0,0,
    0,1,
    1,1
};

struct stepper_motor {
    stepper_motor_status_t status; // Status of the stepper motor
    int speed, position;
    gpio_num_t pul_pin;  // GPIO pin for step signal
    gpio_num_t dir_pin;   // GPIO pin for direction signal
    pcnt_unit_handle_t pcnt_unit; // Pulse counter unit
    mcpwm_timer_handle_t mcpwm_timer; // MCPWM timer handle
};

stepper_motor_handle_t stepper_motor_init(gpio_num_t pul_pin, gpio_num_t dir_pin) {
    // Create a stepper motor handle
    stepper_motor_handle_t handle = (stepper_motor_handle_t)malloc(sizeof(stepper_motor_t));
    if (handle == NULL) {
        ESP_LOGE(TAG, "Failed to allocate memory for stepper motor handle");
        return NULL;
    }
    handle->status = STEPPER_MOTOR_STATUS_INITIALIZED;
    handle->pul_pin = pul_pin;
    handle->dir_pin = dir_pin;
    handle->pcnt_unit = NULL;
    handle->mcpwm_timer = NULL;

    // Configure dir pin as output
    ESP_LOGI(TAG, "Configuring GPIO");
    gpio_config_t io_config = {
        .pin_bit_mask = ((1ULL << dir_pin) | (1ULL << pul_pin)),
        .mode = GPIO_MODE_INPUT_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io_config);
    gpio_set_level(dir_pin, 0); // Set direction pin low initially
    
    // Configure PCNT for position counting
    ESP_LOGI(TAG, "install pcnt unit");
    pcnt_unit_config_t unit_config = {
        .high_limit = INT16_MAX,
        .low_limit = INT16_MIN,
        .flags.accum_count = 1,
    };
    ESP_ERROR_CHECK(pcnt_new_unit(&unit_config, &handle->pcnt_unit));

    ESP_LOGI(TAG, "set watch points");
    ESP_ERROR_CHECK(pcnt_unit_add_watch_point(handle->pcnt_unit, INT16_MAX));
    ESP_ERROR_CHECK(pcnt_unit_add_watch_point(handle->pcnt_unit, INT16_MIN));
    
    ESP_LOGI(TAG, "set glitch filter");
    pcnt_glitch_filter_config_t filter_config = {
        .max_glitch_ns = 1000,
    };
    ESP_ERROR_CHECK(pcnt_unit_set_glitch_filter(handle->pcnt_unit, &filter_config));

    ESP_LOGI(TAG, "install pcnt channels");
    pcnt_chan_config_t chan_config = {
        .edge_gpio_num = pul_pin,
        .level_gpio_num = dir_pin,
    };
    pcnt_channel_handle_t pcnt_chan = NULL;
    ESP_ERROR_CHECK(pcnt_new_channel(handle->pcnt_unit, &chan_config, &pcnt_chan));

    ESP_LOGI(TAG, "set edge and level actions for pcnt channels");
    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(
        pcnt_chan, 
        PCNT_CHANNEL_EDGE_ACTION_HOLD, PCNT_CHANNEL_EDGE_ACTION_INCREASE)
    );
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(
        pcnt_chan, 
        PCNT_CHANNEL_LEVEL_ACTION_INVERSE, PCNT_CHANNEL_LEVEL_ACTION_KEEP)
    );
    
    ESP_LOGI(TAG, "enable pcnt unit");
    ESP_ERROR_CHECK(pcnt_unit_enable(handle->pcnt_unit));
    stepper_motor_reset_position(handle);
    ESP_ERROR_CHECK(pcnt_unit_start(handle->pcnt_unit));

    // Set up MCPWM for step signal generation
    int group_id = group_map[handle_count];
    ESP_LOGI(TAG, "Create timer and operator in group %d", group_id);
    mcpwm_timer_config_t timer_config = {
        .group_id = group_id,
        .clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT,
        .resolution_hz = 1000000,
        .period_ticks = 1000,
        .count_mode = MCPWM_TIMER_COUNT_MODE_UP,
    };
    ESP_ERROR_CHECK(mcpwm_new_timer(&timer_config, &handle->mcpwm_timer));
    handle_count++;

    mcpwm_oper_handle_t oper = NULL;
    mcpwm_operator_config_t operator_config = {
        .group_id = group_id, // operator must be in the same group to the timer
    };
    ESP_ERROR_CHECK(mcpwm_new_operator(&operator_config, &oper));

    ESP_LOGI(TAG, "Connect timer and operator");
    ESP_ERROR_CHECK(mcpwm_operator_connect_timer(oper, handle->mcpwm_timer));

    ESP_LOGI(TAG, "Create comparator and generator from the operator");
    mcpwm_cmpr_handle_t comparator = NULL;
    mcpwm_comparator_config_t comparator_config = {
        .flags.update_cmp_on_tez = true,
    };
    ESP_ERROR_CHECK(mcpwm_new_comparator(oper, &comparator_config, &comparator));

    mcpwm_gen_handle_t generator = NULL;
    mcpwm_generator_config_t generator_config = {
        .gen_gpio_num = pul_pin,
    };
    ESP_ERROR_CHECK(mcpwm_new_generator(oper, &generator_config, &generator));

    // set the comparator to 20 us pulses
    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator, PULSE_WIDTH_US));

    ESP_LOGI(TAG, "Set generator action on timer and compare event");

    // go high on counter empty
    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_timer_event(
        generator,
        MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH))
    );
    // go low on compare threshold
    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_compare_event(
        generator,
        MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, comparator, MCPWM_GEN_ACTION_LOW))
    );

    // suggested bug remedy, check if needed. this needs to happen prior to starting the modules so move pcnt start to after this
    // ESP_ERROR_CHECK(gpio_set_direction(pul_pin, GPIO_MODE_INPUT_OUTPUT));

    ESP_LOGI(TAG, "Enable and start timer");
    ESP_ERROR_CHECK(mcpwm_timer_enable(handle->mcpwm_timer));
    handle->status = STEPPER_MOTOR_STATUS_INITIALIZED;
    return handle;
}

void stepper_motor_start(stepper_motor_handle_t handle) {
    ESP_ERROR_CHECK(mcpwm_timer_start_stop(handle->mcpwm_timer, MCPWM_TIMER_START_NO_STOP));
    handle->status = STEPPER_MOTOR_STATUS_RUNNING;
}

void stepper_motor_stop(stepper_motor_handle_t handle) {
    ESP_ERROR_CHECK(mcpwm_timer_start_stop(handle->mcpwm_timer, MCPWM_TIMER_STOP_FULL));
    handle->status = STEPPER_MOTOR_STATUS_STOPPED;
}

void stepper_motor_set_speed(stepper_motor_handle_t handle, int speed_hz) {
    if (handle == NULL) {
        ESP_LOGE(TAG, "Stepper motor handle is NULL");
        return;
    }
    if (handle->status == STEPPER_MOTOR_STATUS_UNINITIALIZED) {
        ESP_LOGE(TAG, "Stepper motor is not initialized");
        return;
    }
    handle->speed = speed_hz; // Store the speed in the handle
    gpio_set_level(handle->dir_pin, speed_hz < 0 ? 1 : 0);
    speed_hz = abs(speed_hz);
    if (speed_hz > 1) {
        if (handle->status != STEPPER_MOTOR_STATUS_RUNNING) {
            stepper_motor_start(handle); // Start the motor if it is not already running
        }
        uint32_t period = 1000000 / speed_hz; // Convert speed in Hz to period in microseconds
        mcpwm_timer_set_period(handle->mcpwm_timer, period);
    } else {
        stepper_motor_stop(handle); // Stop the motor if speed is too low
    }
}

int stepper_motor_get_speed(stepper_motor_handle_t handle) {
    return handle->speed; // Return the current speed in Hz
}

int stepper_motor_get_position(stepper_motor_handle_t handle) {
    ESP_ERROR_CHECK(pcnt_unit_get_count(handle->pcnt_unit, &handle->position));
    return handle->position; // Return the current position in steps
}

void stepper_motor_reset_position(stepper_motor_handle_t handle) {
    ESP_ERROR_CHECK(pcnt_unit_clear_count(handle->pcnt_unit));
}
