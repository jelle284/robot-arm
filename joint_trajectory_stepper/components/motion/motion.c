#include "motion.h"
#include "driver/gpio.h"
#include "driver/rmt_encoder.h"
#include "driver/rmt_tx.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "esp_log.h"


#define ALL_TX_DONE_BITS (BIT0 | BIT1 | BIT2 | BIT3 | BIT4 | BIT5)
#define MOTION_QUEUE_SIZE 10

size_t axis_index_tracker;
EventGroupHandle_t motion_event_group;

// Queue structure for motion instructions
typedef struct
{
    motion_instruction_t data[MOTION_QUEUE_SIZE];
    size_t head, tail, count;
} motion_queue_t;

typedef struct {
    int time_elapsed_us;
} stepper_encoder_persistence_t;

typedef struct {
    motion_axis_t axis;
    size_t axis_index;
    gpio_num_t pul_pin;
    gpio_num_t dir_pin;
    rmt_channel_handle_t channel;
    rmt_encoder_handle_t encoder;
    stepper_encoder_persistence_t encoder_persistence;
    motion_queue_t queue;
} stepper_driver_t;

size_t stepper_encoder_cb(const void *data, size_t data_size,
                        size_t symbols_written, size_t symbols_free, rmt_symbol_word_t *symbols,
                        bool *done, void *arg)
{
    motion_instruction_t *cmd = (motion_instruction_t*)data;
    size_t steps_to_move = abs(cmd->steps);
    stepper_encoder_persistence_t *persistence = (stepper_encoder_persistence_t*)arg;
    size_t i = 0;
    *done = false;
    while (i < symbols_free) {
        persistence->time_elapsed_us += cmd->pulse_us;
        symbols[i].level0 = 1;
        symbols[i].duration0 = cmd->pulse_us/2;
        symbols[i].level1 = 0;
        symbols[i].duration1 = cmd->pulse_us/2;
        i++;
        if (i + symbols_written >= steps_to_move) {
            *done = true;
            break;
        }
    }
    return i;
}

void axis_enqueue(motion_axis_t* axis_handle, motion_instruction_t* cmd) {
    stepper_driver_t *stepper = __containerof(axis_handle, stepper_driver_t, axis);
    motion_queue_t *queue = &stepper->queue;
    ESP_LOGI("motion", "enqueue on %u", queue->head);
    while (queue->count == MOTION_QUEUE_SIZE) { vTaskDelay(pdMS_TO_TICKS(100)); }
    queue->data[queue->head] = *cmd;
    rmt_transmit_config_t tx_config = {
        .loop_count = 0
    };
    ESP_ERROR_CHECK( rmt_transmit(stepper->channel, stepper->encoder, &queue->data[queue->head], sizeof(motion_instruction_t), &tx_config) );
    queue->head = (queue->head + 1) % MOTION_QUEUE_SIZE;
    queue->count++;
    stepper->axis.status = MOTION_STATUS_BUSY;
}

void axis_dequeue(motion_axis_t *axis_handle)
{
    stepper_driver_t *stepper = __containerof(axis_handle, stepper_driver_t, axis);
    motion_queue_t *queue = &stepper->queue;
    ESP_LOGI("motion", "dequeue on %u", queue->tail);
    if (queue->count > 0) {
        queue->tail = (queue->tail + 1) % MOTION_QUEUE_SIZE;
        queue->count--;
    }
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

/* ===================================================== */
/*                  PUBLIC FUNCTIONS                     */
/* ===================================================== */

motion_axis_t *motion_axis_create(int pul_pin, int dir_pin)
{
    stepper_driver_t* stepper = (stepper_driver_t*)malloc(sizeof(stepper_driver_t));
    stepper->pul_pin = pul_pin;
    stepper->dir_pin = dir_pin;
    stepper->axis.status = MOTION_STATUS_READY;
    stepper->axis.step_count = 0;
    stepper->axis_index = axis_index_tracker;
    
    stepper->queue.head = 0;
    stepper->queue.tail = 0;
    stepper->queue.count = 0;

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
        .trans_queue_depth = MOTION_QUEUE_SIZE,
    };
    ESP_ERROR_CHECK( rmt_new_tx_channel(&tx_chan_config, &stepper->channel) );
    rmt_tx_event_callbacks_t cbs = {
        .on_trans_done = axis_tx_done
    };
    ESP_ERROR_CHECK( rmt_tx_register_event_callbacks(stepper->channel, &cbs, (void*)&stepper->axis_index) );
    ESP_ERROR_CHECK( rmt_enable(stepper->channel) );

    rmt_simple_encoder_config_t encoder_config = {
        .arg = (void*)&stepper->encoder_persistence,
        .min_chunk_size = 64,
        .callback = stepper_encoder_cb
    };

    ESP_ERROR_CHECK( rmt_new_simple_encoder(&encoder_config, &stepper->encoder) );
    ESP_LOGI("motion", "New axis created on index %d", axis_index_tracker);
    axis_index_tracker++;
    return &stepper->axis;
}

void motion_axis_reset(motion_axis_t *axis_handle)
{
    stepper_driver_t *stepper = __containerof(axis_handle, stepper_driver_t, axis);
    ESP_ERROR_CHECK( rmt_encoder_reset(stepper->encoder) );
    stepper->encoder_persistence.time_elapsed_us = 0;
    stepper->axis.status = MOTION_STATUS_READY;
    gpio_set_level(stepper->dir_pin, 0);
}

void motion_axis_move(motion_axis_t* axis_handle, motion_instruction_t cmd)
{
    axis_enqueue(axis_handle, &cmd);
}

void motion_axis_stop(motion_axis_t* axis_handle)
{

}

void motion_group_move(motion_axis_t *axis_handle[], motion_instruction_t cmd[], int group_size)
{
    // TODO: something with sync manager?
}

void log_event_bits(EventBits_t bits) 
{
    char binary_str[33] = {0}; // 32 bits + null terminator
    for (int i = 0; i < 32; i++) {
        binary_str[31 - i] = (bits & (1 << i)) ? '1' : '0';
    }
    ESP_LOGI("motion", "Event group bits: %s", binary_str);
}

void motion_event_await(motion_axis_t* axis_handle[], size_t group_size)
{
    EventBits_t bits = xEventGroupWaitBits(motion_event_group, ALL_TX_DONE_BITS, pdFALSE, pdTRUE, portMAX_DELAY);
    log_event_bits(bits);
    if (bits == ALL_TX_DONE_BITS) {
        // All transmissions are complete
        ESP_LOGI("motion", "All transmissions complete");
        //dequeue items
        for (int i = 0; i < group_size; ++i) {
            axis_dequeue(axis_handle[i]);
        }
        // Reset the event group for future use
        xEventGroupClearBits(motion_event_group, ALL_TX_DONE_BITS);
    }
}

void motion_system_init()
{
    axis_index_tracker = 0;
    motion_event_group = xEventGroupCreate();
    xEventGroupClearBits(motion_event_group, ALL_TX_DONE_BITS);
}