#include "io_expander.h"

void IRAM_ATTR mcp23017_gpio_isr_handler(void *arg)
{
    BaseType_t higher_priority_task_woken = pdFALSE;
    vTaskNotifyGiveFromISR(mcp23017_task_handle, &higher_priority_task_woken);
    if (higher_priority_task_woken == pdTRUE) {
        portYIELD_FROM_ISR();
    }
}

void mcp23017_task(void *arg)
{
    i2c_master_bus_config_t i2c_mst_config = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = I2C_NUM_0,
        .scl_io_num = 22,
        .sda_io_num = 21,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    i2c_master_bus_handle_t bus_handle;

    ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_mst_config, &bus_handle));

    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = MCP23017_I2C_ADDR,
        .scl_speed_hz = 100000,
    };

    i2c_master_dev_handle_t dev_handle;
    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus_handle, &dev_cfg, &dev_handle));

    uint8_t write_buf[2];
    
    write_buf[0] = MCP23017_IODIRA; write_buf[1] = 0xFF;
    ESP_ERROR_CHECK(i2c_master_transmit(dev_handle, write_buf, sizeof(write_buf), I2C_MASTER_TIMEOUT_MS));

    write_buf[0] = MCP23017_IODIRB; write_buf[1] = 0xFF;
    ESP_ERROR_CHECK(i2c_master_transmit(dev_handle, write_buf, sizeof(write_buf), I2C_MASTER_TIMEOUT_MS));

    write_buf[0] = MCP23017_GPPUA; write_buf[1] = 0xFF;
    ESP_ERROR_CHECK(i2c_master_transmit(dev_handle, write_buf, 2, I2C_MASTER_TIMEOUT_MS));

    write_buf[0] = MCP23017_GPPUB; write_buf[1] = 0xFF;
    ESP_ERROR_CHECK(i2c_master_transmit(dev_handle, write_buf, 2, I2C_MASTER_TIMEOUT_MS));

    write_buf[0] = MCP23017_IOCON; write_buf[1] = 0x40;
    ESP_ERROR_CHECK(i2c_master_transmit(dev_handle, write_buf, sizeof(write_buf), I2C_MASTER_TIMEOUT_MS));

    write_buf[0] = MCP23017_GPINTENA; write_buf[1] = 0xFF;
    ESP_ERROR_CHECK(i2c_master_transmit(dev_handle, write_buf, sizeof(write_buf), I2C_MASTER_TIMEOUT_MS));

    write_buf[0] = MCP23017_GPINTENB; write_buf[1] = 0xFF;
    ESP_ERROR_CHECK(i2c_master_transmit(dev_handle, write_buf, sizeof(write_buf), I2C_MASTER_TIMEOUT_MS));
    
    write_buf[0] = MCP23017_IPOLA; write_buf[1] = 0xFF;
    ESP_ERROR_CHECK(i2c_master_transmit(dev_handle, write_buf, sizeof(write_buf), I2C_MASTER_TIMEOUT_MS));

    write_buf[0] = MCP23017_IPOLB; write_buf[1] = 0xFF;
    ESP_ERROR_CHECK(i2c_master_transmit(dev_handle, write_buf, sizeof(write_buf), I2C_MASTER_TIMEOUT_MS));

    uint8_t port_a = 0;
    uint8_t port_b = 0;
    uint8_t read_register;
    for (;;) {
        BaseType_t status = xTaskNotifyWait(0x00, ULONG_MAX, NULL, pdMS_TO_TICKS(500));
        read_register = MCP23017_GPIOA;
        ESP_ERROR_CHECK(i2c_master_transmit_receive(dev_handle, &read_register, 1, &port_a, 1, I2C_MASTER_TIMEOUT_MS));
        read_register = MCP23017_GPIOB;
        ESP_ERROR_CHECK(i2c_master_transmit_receive(dev_handle, &read_register, 1, &port_b, 1, I2C_MASTER_TIMEOUT_MS));
        uint8_t turn_switch = (port_b & BIT0) ? 1 : 0;
        uint8_t green_btn = (port_b & BIT1) ? 1 : 0;
        uint8_t red_btn = (port_b & BIT2) ? 1 : 0;
        uint8_t limit_switch = (port_b & BIT1) ? 1 : 0;
        if (status) {
            ESP_LOGI("I/O", "Interrupted!\nlimit switch: %d", limit_switch);
        } else {
            ESP_LOGI("I/O", "A: %02x, B: %02x", port_a, port_b);
        }
            
    }
}