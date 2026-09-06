#ifndef IO_EXPANDER_H
#define IO_EXPANDER_H
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

#include "driver/i2c_master.h"
#include "driver/gpio.h"

#define MCP23017_I2C_ADDR   0x20
#define MCP23017_IODIRA     0x00
#define MCP23017_IODIRB     0x01
#define MCP23017_GPIOA      0x12
#define MCP23017_GPIOB      0x13
#define MCP23017_IOCON      0x0A
#define MCP23017_GPINTENA   0x04
#define MCP23017_GPINTENB   0x05
#define MCP23017_GPPUA      0x0C
#define MCP23017_GPPUB      0x0D
#define MCP23017_IPOLA      0x02
#define MCP23017_IPOLB      0x03
#define I2C_MASTER_TIMEOUT_MS       1000

static TaskHandle_t mcp23017_task_handle = NULL;

void mcp23017_task(void *arg);
void mcp23017_gpio_isr_handler(void *arg);
#endif //IO_EXPANDER_H