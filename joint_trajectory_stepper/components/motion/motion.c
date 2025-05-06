#include "motion.h"
#include "driver/gpio.h"
#include "driver/gptimer.h"
#include "driver/rmt_encoder.h"
#include "driver/rmt_tx.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "esp_log.h"

#define FB_RDY_BIT BIT0
#define MOTION_BUFFER_SIZE 128
#define VMIN 1000*1000
/* ===================================================== */
/*                     PRIVATE TYPES                     */
/* ===================================================== */
typedef enum {
    MS_UNINITIALIZED,
    MS_IDLE,
    MS_LOADING,
    MS_EXECUTING,
    MS_COMPLETE
} motion_state_t;

typedef struct {
    int64_t position;
    int64_t velocity;
    int64_t acceleration;
    int64_t time_from_start;
} motion_instruction_t;

typedef struct {
    size_t data_position;
    int64_t time_us;
    int64_t steps_moved;
} stepper_encoder_ctx_t;

typedef struct {
    size_t index;
    gpio_num_t pul_pin;
    gpio_num_t dir_pin;
    rmt_encoder_handle_t encoder;
    stepper_encoder_ctx_t encoder_ctx;
    motion_instruction_t data[MOTION_BUFFER_SIZE];
} motion_axis_t;

/* ===================================================== */
/*                     PRIVATE VARS                      */
/* ===================================================== */
motion_axis_t           axes[MOTION_AXIS_NUM];
rmt_channel_handle_t    tx_channels[MOTION_AXIS_NUM];
motion_state_t          motion_state = MS_UNINITIALIZED;

size_t                  load_index;
size_t                  exec_index;
uint64_t                exec_time_us[MOTION_BUFFER_SIZE];
int32_t                 exec_steps[MOTION_AXIS_NUM];
gptimer_handle_t        exec_timer;

EventGroupHandle_t      motion_event_group;

/* ===================================================== */
/*                  PRIVATE FUNCTIONS                    */
/* ===================================================== */

size_t stepper_encoder_cb(const void *data, size_t data_size,
    size_t symbols_written, size_t symbols_free, rmt_symbol_word_t *symbols,
    bool *done, void *arg)
{
    // retrieve command buffer and values from encoding context
    motion_instruction_t *cmd = (motion_instruction_t*)data;
    stepper_encoder_ctx_t *ctx = (stepper_encoder_ctx_t*)arg;
    size_t idx = ctx->data_position;
    int64_t t = ctx->time_us;
    int64_t z = ctx->steps_moved;

    // Start encoding symbols
    size_t i = 0;
    *done = false;
    while (i < symbols_free) {
        // retrieve values from current command
        const int64_t ts = cmd[idx + 1].time_from_start - cmd[idx].time_from_start;
        const int64_t a0 = cmd[idx].acceleration;
        const int64_t v0 = cmd[idx].velocity;
        const int64_t x0 = cmd[idx].position;

        // encode with current point while time is less than that of next point
        if (t < ts) {
            // calculate equation of motion to current time
            int64_t v = (a0*t)/1000 + v0;
            int64_t x = (a0*t*t)/(2*1000*1000) + (v0*t)/1000 + x0;

            // ensure we stay below the minimum step time
            int64_t vdiv;
            if (v > VMIN || v < -VMIN) {
                vdiv = llabs(v);
            }
            else {
                vdiv = VMIN;
            }
            int64_t dt = (1000*1000*1000)/vdiv;

            // check that we don't exceed time remaining
            if (t + dt > ts) {
                dt = ts - t;
            }

            // determine if we should step now or not
            bool step_forward = x-z > 500*1000 && v > 0;
            bool step_reverse = x-z < -500*1000 && v < 0;
            if (step_forward || step_reverse) {
                symbols[i].level0 = 1;
                z += 1000*1000;
            } else {
                symbols[i].level0 = 0;
            }
            symbols[i].duration0 = dt / 2;
            symbols[i].level1 = 0;
            symbols[i].duration1 = dt / 2;

            // advance to next symbol
            i++;
            t += dt;
            continue;
        }
        // If we are here all steps have been encoded
        // and we move to next data point
        idx++;
        if (idx >= data_size - 1) { 
            *done = true; 
            break;
        }
    }
    ctx->data_position = idx;
    ctx->time_us = t;
    return i;
}

static bool IRAM_ATTR exec_timer_cb(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void *user_data)
{
    BaseType_t high_task_awoken = pdFALSE;
    
    // Submit feedback
    for (int i = 0; i < MOTION_AXIS_NUM; ++i) {
        exec_steps[i] = axes[i].data[exec_index].position;
    }
    xEventGroupSetBitsFromISR(motion_event_group, FB_RDY_BIT, &high_task_awoken);

    // Update direction pins for next instruction
    exec_index++;
    motion_instruction_t instr;
    for (int i = 0; i < MOTION_AXIS_NUM; ++i) {
        instr = axes[i].data[exec_index];
        gpio_set_level(axes[i].dir_pin, (instr.velocity < 0) ? 1 : 0);
    }
    
    // reconfigure alarm value
    if (exec_index < load_index) {
        gptimer_alarm_config_t alarm_config = {
            .alarm_count = exec_time_us[exec_index],
        };
        gptimer_set_alarm_action(timer, &alarm_config);
    } else {
        // TX is finished.
        gptimer_stop(timer);
        gptimer_set_raw_count(exec_timer, 0);
    }
    // return whether we need to yield at the end of ISR
    return (high_task_awoken == pdTRUE);
}

/* ===================================================== */
/*                  PUBLIC FUNCTIONS                     */
/* ===================================================== */

void motion_system_init(const int* pul_pins, const int* dir_pins) {
    // Create event group
    motion_event_group = xEventGroupCreate();
    xEventGroupClearBits(motion_event_group, FB_RDY_BIT);

    // Create GP timer for execution control
    gptimer_config_t timer_config = {
        .clk_src = GPTIMER_CLK_SRC_DEFAULT,
        .direction = GPTIMER_COUNT_UP,
        .resolution_hz = 1000000, // 1 MHz, 1 tick = 1 us
    };
    ESP_ERROR_CHECK(gptimer_new_timer(&timer_config, &exec_timer));
    gptimer_event_callbacks_t cbs = {
        .on_alarm = exec_timer_cb,
    };
    ESP_ERROR_CHECK( gptimer_register_event_callbacks(exec_timer, &cbs, NULL) );
    ESP_ERROR_CHECK( gptimer_enable(exec_timer) );

    // Initialize Axes
    for (size_t i = 0; i < MOTION_AXIS_NUM; ++i){
        
        axes[i].index = i;
        axes[i].pul_pin = pul_pins[i];
        axes[i].dir_pin = dir_pins[i];

        // Init direction pin
        gpio_config_t io_conf = {};
        io_conf.intr_type = GPIO_INTR_DISABLE;
        io_conf.mode = GPIO_MODE_OUTPUT;
        io_conf.pin_bit_mask = ((1UL<<dir_pins[i]));
        io_conf.pull_down_en = 0;
        io_conf.pull_up_en = 0;
        gpio_config(&io_conf);
        gpio_set_level(dir_pins[i], 0);

        // Init RMT channel
        rmt_tx_channel_config_t tx_chan_config = {
            .clk_src = RMT_CLK_SRC_DEFAULT,
            .gpio_num = pul_pins[i],
            .mem_block_symbols = 64,
            .resolution_hz = 1 * 1000 * 1000,
            .trans_queue_depth = 1,
        };
        ESP_ERROR_CHECK(rmt_new_tx_channel(&tx_chan_config, &tx_channels[i]));

        rmt_simple_encoder_config_t encoder_config = {
            .arg = (void*)&axes[i].encoder_ctx,
            .min_chunk_size = 64,
            .callback = stepper_encoder_cb
        };
        ESP_ERROR_CHECK(rmt_new_simple_encoder(&encoder_config, &axes[i].encoder));

        ESP_ERROR_CHECK(rmt_enable(tx_channels[i]));
        ESP_LOGI("motion", "New axis created on index %d", i);
    }
    motion_state = MS_IDLE;
}   

void motion_load_trajectory(int64_t acceleration, int64_t velocity, int64_t position, int64_t time_from_start) {
    switch(motion_state) {
        case MS_UNINITIALIZED:
            ESP_LOGE("motion", "Motion is uninitalized!");
            return;
        case MS_EXECUTING:
            ESP_LOGE("motion", "Can not load points while a trajectory is executing.");
            return;
        case MS_IDLE:
        case MS_COMPLETE:
            load_index = 0;
            motion_state = MS_LOADING;
            break;
        default:
            break;
    }
    if (load_index >= MOTION_BUFFER_SIZE) {
        ESP_LOGE("motion", "Data buffer is full. Cannot load trajectory.");
        return;
    }
    for (size_t i = 0; i < MOTION_AXIS_NUM; ++i) {
        motion_instruction_t cmd = {
            .acceleration = acceleration,
            .velocity = velocity,
            .position = position,
            .time_from_start = time_from_start
        };
        axes[i].data[load_index] = cmd;
    }
    exec_time_us[load_index] = time_from_start;
    load_index++;
}

void motion_execute() {
    // check if we have points to execute
    if (load_index == 0) { 
        ESP_LOGE("motion", "No points in loaded. Failed to execute.");
        return;
    }

    // set motion state
    motion_state = MS_EXECUTING;

    // Prepare for execution
    for (size_t i = 0; i < MOTION_AXIS_NUM; ++i) {
        axes[i].encoder_ctx.data_position = 0;
        axes[i].encoder_ctx.steps_moved = 0;
        axes[i].encoder_ctx.time_us = 0;
        ESP_ERROR_CHECK( rmt_encoder_reset(axes[i].encoder) );
        gpio_set_level(axes[i].dir_pin, (axes[i].data[0].velocity < 0) ? 1 : 0);
        exec_steps[i] = 0;
    }
    exec_index = 0;
    
    // Setup Timer
    gptimer_alarm_config_t alarm_config = {
        .alarm_count = exec_time_us[0],
    };
    ESP_ERROR_CHECK( gptimer_set_alarm_action(exec_timer, &alarm_config) );

    // Start timer
    ESP_ERROR_CHECK( gptimer_start(exec_timer) );

    // start transmitting
    rmt_transmit_config_t tx_config = {
        .loop_count = 0
    };
    for (size_t i = 0; i < MOTION_AXIS_NUM; ++i) {
        ESP_ERROR_CHECK(rmt_transmit(tx_channels[i], axes[i].encoder, axes[i].data, load_index, &tx_config));
    }

    // Wait for execution to finish
    for (size_t i = 0; i < MOTION_AXIS_NUM; ++i) {
        rmt_tx_wait_all_done(tx_channels[i], -1);
    }
    motion_state = MS_COMPLETE;
}

void motion_stop()
{
    for (size_t i = 0; i < MOTION_AXIS_NUM; ++i) {
        ESP_ERROR_CHECK( rmt_disable(tx_channels[i]) );
    }

    for (size_t i = 0; i < MOTION_AXIS_NUM; ++i) {
        ESP_ERROR_CHECK( rmt_enable(tx_channels[i]) );
    }
}

int motion_get_feedback(motion_feedback_t *state) {
    const unsigned int wait_ms = 100;
    EventBits_t bits = xEventGroupWaitBits(motion_event_group, FB_RDY_BIT, pdFALSE, pdTRUE, pdMS_TO_TICKS(wait_ms));
    if (bits & FB_RDY_BIT) {
        for (int i = 0; i < MOTION_AXIS_NUM; ++i) {
            state->steps_executed[i] = exec_steps[i];
        }
        state->current_point = exec_index;
        state->total_points = load_index;
        xEventGroupClearBits(motion_event_group, FB_RDY_BIT);
        return 1;
    }

    return 0;
}