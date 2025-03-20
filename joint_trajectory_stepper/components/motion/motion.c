#include "motion.h"
#include "driver/gpio.h"
#include "driver/rmt_encoder.h"
#include "driver/rmt_tx.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "esp_log.h"


#define ALL_TX_DONE_BITS (BIT0 | BIT1 | BIT2 | BIT3 | BIT4 | BIT5)
#define MAX_DATA_SIZE 256

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
    motion_instruction_t data[MAX_DATA_SIZE];
    size_t data_size;
};

// TODO:
// -control dir pin. can not be done from encoder, must be timed from execution start

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
        symbols[i].level0 = (cmd[idx].steps == 0) ? 0 : 1;
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

void log_event_bits(EventBits_t bits) 
{
    char binary_str[33] = {0}; // 32 bits + null terminator
    for (int i = 0; i < 32; i++) {
        binary_str[31 - i] = (bits & (1 << i)) ? '1' : '0';
    }
    ESP_LOGI("motion", "Event group bits: %s", binary_str);
}

/* ===================================================== */
/*                  PUBLIC FUNCTIONS                     */
/* ===================================================== */
void motion_system_init()
{
    axis_index_tracker = 0;
    motion_event_group = xEventGroupCreate();
    xEventGroupClearBits(motion_event_group, ALL_TX_DONE_BITS);
}

motion_axis_handle_t motion_axis_create(int pul_pin, int dir_pin)
{
    motion_axis_handle_t axis_handle = (motion_axis_handle_t)malloc(sizeof(struct motion_axis_t));
    axis_handle->pul_pin = pul_pin;
    axis_handle->dir_pin = dir_pin;
    axis_handle->axis_index = axis_index_tracker;
    axis_handle->data_size = 0;

    gpio_config_t io_conf = {};
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = ((1UL<<dir_pin)) ;
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
    ESP_ERROR_CHECK( rmt_new_tx_channel(&tx_chan_config, &axis_handle->channel) );

    rmt_tx_event_callbacks_t cbs = {
        .on_trans_done = axis_tx_done
    };
    ESP_ERROR_CHECK( rmt_tx_register_event_callbacks(axis_handle->channel, &cbs, (void*)&axis_handle->axis_index) );

    rmt_simple_encoder_config_t encoder_config = {
        .arg = (void*)&axis_handle->encoder_ctx,
        .min_chunk_size = 64,
        .callback = stepper_encoder_cb
    };
    ESP_ERROR_CHECK( rmt_new_simple_encoder(&encoder_config, &axis_handle->encoder) );

    ESP_ERROR_CHECK( rmt_enable(axis_handle->channel) );

    ESP_LOGI("motion", "New axis created on index %d", axis_index_tracker);
    axis_index_tracker++;
    return axis_handle;
}

void motion_axis_load_trajectory(motion_axis_handle_t axis_handle, motion_instruction_t* cmd) {
    if (axis_handle->data_size < MAX_DATA_SIZE) {
        ESP_LOGI("motion", "Axis %u loaded point into buffer at %u", axis_handle->axis_index, axis_handle->data_size);
        axis_handle->data[axis_handle->data_size] = *cmd;
        axis_handle->data_size++;
    } else {
        ESP_LOGE("motion", "Data buffer is full. Cannot load trajectory.");
    }
}

void motion_axis_reset(motion_axis_handle_t axis_handle)
{
    stepper_encoder_ctx_t *ctx = &axis_handle->encoder_ctx;

    ESP_ERROR_CHECK( rmt_encoder_reset(axis_handle->encoder) );
    gpio_set_level(axis_handle->dir_pin, 0);

    ctx->data_position = 0;
    ctx->step_count = 0;
    axis_handle->data_size = 0;
}

void motion_axis_execute(motion_axis_handle_t axis_handle) {
    rmt_transmit_config_t tx_config = {
        .loop_count = 0
    };
    ESP_ERROR_CHECK( rmt_transmit(axis_handle->channel, axis_handle->encoder, axis_handle->data, axis_handle->data_size, &tx_config) );
    ESP_LOGI("motion", "Transmitting on axis %u", axis_handle->axis_index);
}

void motion_axis_stop(motion_axis_handle_t axis_handle)
{
    ESP_ERROR_CHECK( rmt_disable(axis_handle->channel) );
    ESP_ERROR_CHECK( rmt_enable(axis_handle->channel) );
}

void motion_group_move(motion_axis_handle_t axis_handle[], motion_instruction_t cmd[], int group_size)
{
    // TODO: something with sync manager?
}

void motion_event_await(motion_axis_handle_t axis_handle[], size_t group_size)
{
    EventBits_t bits = xEventGroupWaitBits(motion_event_group, ALL_TX_DONE_BITS, pdFALSE, pdTRUE, portMAX_DELAY);
    log_event_bits(bits);
    if (bits == ALL_TX_DONE_BITS) {
        // All transmissions are complete
        ESP_LOGI("motion", "All transmissions complete");
        // Reset the event group for future use
        xEventGroupClearBits(motion_event_group, ALL_TX_DONE_BITS);
    }
}

int16_t motion_axis_get_feedback(motion_axis_handle_t axis_handle)
{
    int16_t executed_steps = axis_handle->encoder_ctx.step_count;
    axis_handle->encoder_ctx.step_count = 0;
    return executed_steps;
}