#include "motion.h"
#include "driver/gpio.h"
#include "driver/rmt_encoder.h"
#include "driver/rmt_tx.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "esp_log.h"
#include "esp_timer.h"

#define ALL_TX_DONE_BITS (BIT0 | BIT1 | BIT2 | BIT3 | BIT4 | BIT5)
#define FB_RDY_BIT BIT15

#define TIMER_INTERVAL_US 200

motion_axis_handle_t axes[MOTION_AXIS_NUM];
size_t axis_index_tracker;
EventGroupHandle_t motion_event_group;

typedef struct {
    size_t data_position;
    uint16_t step_count;
} stepper_encoder_ctx_t;

struct motion_axis_t {
    size_t axis_index;
    gpio_num_t pul_pin;
    gpio_num_t dir_pin;
    rmt_channel_handle_t channel;
    rmt_encoder_handle_t encoder;
    stepper_encoder_ctx_t encoder_ctx;
    motion_instruction_t data[MOTION_BUFFER_SIZE];
    size_t data_size;
    volatile size_t execution_index;
    volatile int64_t execution_time;
    volatile int32_t execution_feedback;
    volatile int8_t execution_status;
};

size_t stepper_encoder_cb(const void *data, size_t data_size,
    size_t symbols_written, size_t symbols_free, rmt_symbol_word_t *symbols,
    bool *done, void *arg)
{
    motion_instruction_t *cmd = (motion_instruction_t*)data;
    stepper_encoder_ctx_t *ctx = (stepper_encoder_ctx_t*)arg;
    size_t idx = ctx->data_position;
    size_t cnt = ctx->step_count;
    size_t i = 0;
    *done = false;
    while (i < symbols_free) {
        symbols[i].level0 = cmd[idx].level;
        symbols[i].duration0 = cmd[idx].pulse_us / 2;
        symbols[i].level1 = 0;
        symbols[i].duration1 = cmd[idx].pulse_us / 2;
        i++;
        cnt++;
        if (cnt >= abs(cmd[idx].steps)) {
            idx++;
            cnt = 0;
            if (idx >= data_size) { 
                *done = true; 
                break;
            }
        }
        
    }
    ctx->step_count = cnt;
    ctx->data_position = idx;
    return i;
}

bool axis_tx_done(rmt_channel_handle_t tx_chan, const rmt_tx_done_event_data_t *edata, void* user_ctx) {
    unsigned int* idx = (unsigned int*)user_ctx;
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xEventGroupSetBitsFromISR(motion_event_group, BIT(*idx), &xHigherPriorityTaskWoken);
    if (xHigherPriorityTaskWoken == pdTRUE) {
        portYIELD_FROM_ISR();
    }
    return false;
}

void IRAM_ATTR execution_control_callback(void *arg) {
    motion_instruction_t instr;
    bool fb_rdy = false;
    for (int i = 0; i < axis_index_tracker; ++i) {
        motion_axis_handle_t axis_handle = axes[i];
        if (axis_handle->execution_status == 0) { continue; }
        instr = axis_handle->data[axis_handle->execution_index];
        int64_t now = esp_timer_get_time();
        int64_t elapsed = now - axis_handle->execution_time;
        int64_t duration = abs(instr.steps) * instr.pulse_us;
        if (elapsed >= duration) {
            if (instr.level == 1) {
                axis_handle->execution_feedback += instr.steps;
                fb_rdy = true;
            }
            axis_handle->execution_time += duration;
            axis_handle->execution_index++;
            if (axis_handle->execution_index >= axis_handle->data_size) {
                axis_handle->execution_status = 0;
                continue; 
            }
            instr = axis_handle->data[axis_handle->execution_index];
            gpio_set_level(axis_handle->dir_pin, (instr.steps < 0) ? 1 : 0);
        }
    }
    if (fb_rdy) {
        BaseType_t need_yield = pdFALSE;
        xEventGroupSetBitsFromISR(motion_event_group, FB_RDY_BIT, &need_yield);
        if (need_yield == pdTRUE) { esp_timer_isr_dispatch_need_yield(); }
    }
}

/* ===================================================== */
/*                  PUBLIC FUNCTIONS                     */
/* ===================================================== */

void motion_system_init() {
    axis_index_tracker = 0;
    motion_event_group = xEventGroupCreate();
    xEventGroupClearBits(motion_event_group, ALL_TX_DONE_BITS | FB_RDY_BIT);

    // create timer for execution control
    const esp_timer_create_args_t execution_control_timer_args = {
        .callback = &execution_control_callback,
        /* name is optional, but may help identify the timer when debugging */
        .dispatch_method = ESP_TIMER_ISR,
        .name = "execution control"
    };
    esp_timer_handle_t execution_control_timer;
    ESP_ERROR_CHECK( esp_timer_create(&execution_control_timer_args, &execution_control_timer) );
    ESP_ERROR_CHECK( esp_timer_start_periodic(execution_control_timer, TIMER_INTERVAL_US) );
}   

motion_axis_handle_t motion_axis_create(int pul_pin, int dir_pin) {
    if (axis_index_tracker >= MOTION_AXIS_NUM) {
        ESP_LOGE("motion", "Max axis number reached. Failed to create axis.");
        return NULL;
    }

    motion_axis_handle_t axis_handle = (motion_axis_handle_t)malloc(sizeof(struct motion_axis_t));
    axis_handle->pul_pin = pul_pin;
    axis_handle->dir_pin = dir_pin;
    axis_handle->axis_index = axis_index_tracker;
    axis_handle->data_size = 0;
    axis_handle->execution_time = 0;
    axis_handle->execution_index = 0;
    axis_handle->execution_feedback = 0;
    axis_handle->execution_status = 0;

    gpio_config_t io_conf = {};
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = ((1UL<<dir_pin));
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 0;
    gpio_config(&io_conf);
    gpio_set_level(dir_pin, 0);

    rmt_tx_channel_config_t tx_chan_config = {
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .gpio_num = pul_pin,
        .mem_block_symbols = 64,
        .resolution_hz = 1 * 1000 * 1000,
        .trans_queue_depth = 1,
    };
    ESP_ERROR_CHECK(rmt_new_tx_channel(&tx_chan_config, &axis_handle->channel));

    rmt_tx_event_callbacks_t cbs = {
        .on_trans_done = axis_tx_done
    };
    ESP_ERROR_CHECK(rmt_tx_register_event_callbacks(axis_handle->channel, &cbs, (void*)&axis_handle->axis_index));

    rmt_simple_encoder_config_t encoder_config = {
        .arg = (void*)&axis_handle->encoder_ctx,
        .min_chunk_size = 64,
        .callback = stepper_encoder_cb
    };
    ESP_ERROR_CHECK(rmt_new_simple_encoder(&encoder_config, &axis_handle->encoder));

    ESP_ERROR_CHECK(rmt_enable(axis_handle->channel));

    ESP_LOGI("motion", "New axis created on index %d", axis_index_tracker);
    axes[axis_index_tracker] = axis_handle;
    axis_index_tracker++;
    return axis_handle;
}

void motion_axis_load_trajectory(motion_axis_handle_t axis_handle, motion_instruction_t cmd) {
    if (axis_handle->data_size >= MOTION_BUFFER_SIZE) {
        ESP_LOGE("motion", "Data buffer is full. Cannot load trajectory.");
        return;
    }
    ESP_LOGI("motion", "Axis %u loaded point (%hd, %hu) into buffer at %u",
        axis_handle->axis_index, cmd.steps, cmd.pulse_us, axis_handle->data_size);
    axis_handle->data[axis_handle->data_size] = cmd;
    axis_handle->data_size++;
}

void motion_axis_reset(motion_axis_handle_t axis_handle) {
    stepper_encoder_ctx_t *ctx = &axis_handle->encoder_ctx;

    ESP_ERROR_CHECK(rmt_encoder_reset(axis_handle->encoder));
    gpio_set_level(axis_handle->dir_pin, 0);

    ctx->data_position = 0;
    ctx->step_count = 0;
    axis_handle->data_size = 0;
}

void motion_axis_execute(motion_axis_handle_t axis_handle) {
    if (axis_handle->data_size == 0) { 
        ESP_LOGE("motion", "No points in axis %u, failed to execute.", axis_handle->data_size);
        return;
    }
    rmt_transmit_config_t tx_config = {
        .loop_count = 0
    };
    gpio_set_level(axis_handle->dir_pin, (axis_handle->data[0].steps < 0) ? 1 : 0);
    axis_handle->execution_time = esp_timer_get_time();
    axis_handle->execution_index = 0;
    axis_handle->execution_feedback = 0;
    axis_handle->execution_status = 1;
    ESP_ERROR_CHECK(rmt_transmit(axis_handle->channel, axis_handle->encoder, axis_handle->data, axis_handle->data_size, &tx_config));
    ESP_LOGI("motion", "Transmitting on axis %u", axis_handle->axis_index);
}

void motion_axis_stop(motion_axis_handle_t axis_handle)
{
    ESP_ERROR_CHECK( rmt_disable(axis_handle->channel) );
    ESP_ERROR_CHECK( rmt_enable(axis_handle->channel) );
}

void motion_group_execute(motion_axis_handle_t axis_handle[], size_t group_size)
{
    // TODO: something with sync manager?
}

void motion_await_done()
{
    EventBits_t bits = xEventGroupWaitBits(motion_event_group, ALL_TX_DONE_BITS, pdFALSE, pdTRUE, portMAX_DELAY);
    if (bits == ALL_TX_DONE_BITS) {
        ESP_LOGI("motion", "All transmissions complete");
        xEventGroupClearBits(motion_event_group, ALL_TX_DONE_BITS);
    }
}

void motion_get_feedback(int32_t *feedback) {
    xEventGroupWaitBits(motion_event_group, FB_RDY_BIT, pdFALSE, pdFALSE, portMAX_DELAY);
    for (int i = 0; i < axis_index_tracker; ++i) {
        feedback[i] = axes[i]->execution_feedback;
    }
    xEventGroupClearBits(motion_event_group, FB_RDY_BIT);
}