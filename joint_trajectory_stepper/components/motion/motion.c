#include "motion.h"
#include <math.h>
#include "driver/gpio.h"
#include "driver/rmt_encoder.h"
#include "driver/rmt_tx.h"

#define S_TO_US(s) (int)(s*(1000*1000))
#define CURVE_POINTS 100
#define MIN_STEP_US 4000

typedef struct {
    int time_elapsed_us;
} stepper_encoder_persistence_t;

typedef struct {
    int steps_to_move;
    int time_to_move_us;
    int time_interval;
    uint16_t lookup[CURVE_POINTS];
} stepper_table_t;

typedef struct {
    motion_axis_t axis;
    gpio_num_t pul_pin;
    gpio_num_t dir_pin;
    uint16_t resolution;
    rmt_channel_handle_t channel;
    rmt_encoder_handle_t encoder;
    stepper_encoder_persistence_t encoder_persistence;
    stepper_table_t table;
} stepper_driver_t;

void stepper_table_fill(stepper_table_t* table) {
    table->time_interval = table->time_to_move_us/(CURVE_POINTS - 1);
    const float duration = 1e-6f* table->time_to_move_us;
    const float c = (duration / (1e-6f*MIN_STEP_US)) / table->steps_to_move;
    const float a = 2.0f*c - 2.0f;
    const float b = 3.0f - 3.0f*c;
    const float increment = 1.0f / (CURVE_POINTS - 1);
    for (int i = 0; i < CURVE_POINTS; ++i) {
        const float x = increment*i;
        const float f = 3.0f*a*x*x + 2.0f*b*x + c;
        int vs = f*table->steps_to_move / duration;
        const int vmin = 1000000 / MIN_STEP_US;
        if (vs < vmin) {
            vs = vmin;
        }
        table->lookup[i] = 500000 / vs;
    }
}

uint16_t stepper_table_eval(const stepper_table_t *table, int microseconds) {
    int idx = microseconds / table->time_interval;
    if (idx >= CURVE_POINTS) idx = CURVE_POINTS - 1;
    if (idx < 0) idx = 0;
    int interp = 0;
    if (idx < CURVE_POINTS-1) {
        int remainder = microseconds - (table->time_interval*idx);
        int diff = table->lookup[idx+1] - table->lookup[idx];
        interp = diff*remainder / table->time_interval;
    }
    return table->lookup[idx] + interp;
}

size_t stepper_encoder_cb(const void *data, size_t data_size,
                        size_t symbols_written, size_t symbols_free, rmt_symbol_word_t *symbols,
                        bool *done, void *arg)
{
    stepper_table_t *table = (stepper_table_t*)data;
    stepper_encoder_persistence_t *persistence = (stepper_encoder_persistence_t*)arg;
    size_t i = 0;
    *done = false;
    while (i < symbols_free) {
        uint16_t pulse_duration = stepper_table_eval(table, persistence->time_elapsed_us);
        persistence->time_elapsed_us += 2*pulse_duration;
        symbols[i].level0 = 1;
        symbols[i].duration0 = pulse_duration;
        symbols[i].level1 = 0;
        symbols[i].duration1 = pulse_duration;
        i++;
        if (i + symbols_written >= table->steps_to_move) {
            *done = true;
            break;
        }
    }
    return i;
}

void axis_transmit(motion_axis_t* axis_handle, motion_instruction_t* cmd) {
    stepper_driver_t *stepper = __containerof(axis_handle, stepper_driver_t, axis);
    ESP_ERROR_CHECK( rmt_encoder_reset(stepper->encoder) );
    stepper->encoder_persistence.time_elapsed_us = 0;
    stepper->table.steps_to_move = fabsf(cmd->angle) / 360.0f *stepper->resolution * axis_handle->gear_ratio;
    stepper->table.time_to_move_us = S_TO_US(cmd->duration);
    stepper_table_fill(&stepper->table);
    gpio_set_level(stepper->dir_pin, cmd->angle < 0 ? 1 : 0);
    rmt_transmit_config_t tx_config = {
        .loop_count = 0
    };
    ESP_ERROR_CHECK( rmt_transmit(stepper->channel, stepper->encoder, &stepper->table, sizeof(stepper_table_t), &tx_config) );
}

void axis_waitfor(motion_axis_t* axis_handle) {
    stepper_driver_t *stepper = __containerof(axis_handle, stepper_driver_t, axis);
    ESP_ERROR_CHECK( rmt_tx_wait_all_done(stepper->channel, 20*1000) );
}

/* ===================================================== */
/*                  PUBLIC FUNCTIONS                     */
/* ===================================================== */

motion_axis_t *motion_axis_create(int pul_pin, int dir_pin, int resolution)
{
    stepper_driver_t* stepper = (stepper_driver_t*)malloc(sizeof(stepper_driver_t));
    stepper->pul_pin = pul_pin;
    stepper->dir_pin = dir_pin;
    stepper->resolution = resolution;

    stepper->axis.gear_ratio = 1;
    stepper->axis.max_acceleration = 0;
    stepper->axis.max_velocity = 0;
    stepper->axis.output_position = 0;

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
        .trans_queue_depth = 10,
    };
    ESP_ERROR_CHECK( rmt_new_tx_channel(&tx_chan_config, &stepper->channel) );

    ESP_ERROR_CHECK( rmt_enable(stepper->channel) );

    rmt_simple_encoder_config_t encoder_config = {
        .arg = (void*)&stepper->encoder_persistence,
        .min_chunk_size = 64,
        .callback = stepper_encoder_cb
    };

    ESP_ERROR_CHECK( rmt_new_simple_encoder(&encoder_config, &stepper->encoder) );
    return &stepper->axis;
}

void motion_axis_move(motion_axis_t* axis_handle, motion_instruction_t cmd)
{
    axis_transmit(axis_handle, &cmd);
    //axis_waitfor(axis_handle);
}

void motion_axis_stop(motion_axis_t* axis_handle)
{

}

void motion_group_move(motion_axis_t *axis_handle[], motion_instruction_t cmd[], int group_size)
{
    // TODO: something with sync manager?
    for (int i = 0; i < group_size; i++) {
        axis_transmit(axis_handle[i], &cmd[i]);
    }
    // for (int i = 0; i < group_size; i++) {
    //     axis_waitfor(axis_handle[i]);
    // }
}