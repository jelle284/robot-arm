#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "driver/i2c_master.h"

// For wifi connection (needs migration)
#include "net_connection.h"

// For micro-ROS
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <stepper_msgs/msg/stepper_state.h>
#include <stepper_msgs/msg/stepper_command.h>
#include <std_msgs/msg/int32.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>

// For stepper motor control
#include "stepper_motor.h"
#include "pid_controller.h"

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Aborting.\n",__LINE__,(int)temp_rc);vTaskDelete(NULL);}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Continuing.\n",__LINE__,(int)temp_rc);}}

#define AXIS_NUM 6 // Number of stepper motors
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

static const char *TAG = "main";

stepper_motor_handle_t motor_handle[AXIS_NUM];
const int pulse_pins[] = {26, 14, 23, 33, 18, 16};
const int dir_pins[] = {27, 13, 4, 25, 19, 17};
int target_positions[AXIS_NUM] = {0};

static TaskHandle_t mcp23017_task_handle = NULL;

rcl_publisher_t state_publisher;
rcl_subscription_t command_subscriber;
stepper_msgs__msg__StepperCommand stepper_command;
stepper_msgs__msg__StepperState stepper_state;
static const char *TAG = "main";

void control_loop(void* arg);
void micro_ros_task(void* arg);


static void IRAM_ATTR mcp23017_gpio_isr_handler(void *arg)
{
    BaseType_t higher_priority_task_woken = pdFALSE;
    vTaskNotifyGiveFromISR(mcp23017_task_handle, &higher_priority_task_woken);
    if (higher_priority_task_woken == pdTRUE) {
        portYIELD_FROM_ISR();
    }
}

static void mcp23017_task(void *arg)
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

// Main application
void app_main() 
{
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << GPIO_NUM_34),
        .mode = GPIO_MODE_INPUT,
        .intr_type = GPIO_INTR_NEGEDGE,
    };

    gpio_config(&io_conf);
    gpio_install_isr_service(0);
    gpio_isr_handler_add(GPIO_NUM_34, mcp23017_gpio_isr_handler, NULL);
    xTaskCreate(mcp23017_task, "mcp23017_task", 4096, NULL, configMAX_PRIORITIES - 1, &mcp23017_task_handle);

    for (int i = 0; i < AXIS_NUM; i++) {
        motor_handle[i] = stepper_motor_init(pulse_pins[i], dir_pins[i]);
        if (motor_handle[i] == NULL) {
            ESP_LOGE(TAG, "Failed to initialize stepper motor on pulse pin %d, dir pin %d", pulse_pins[i], dir_pins[i]);
        } else {
            ESP_LOGI(TAG, "Stepper motor initialized on pulse pin %d, dir pin %d", pulse_pins[i], dir_pins[i]);
        }
    }
    network_init();
    // xTaskCreate(micro_ros_task, "micro_ros_task", 4096, NULL, 5, NULL);
    // xTaskCreate(control_loop, "control_loop", 4096, NULL, 5, NULL);

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

// Micro-ROS stuff below

void command_callback(const void * msgin)
{
	const stepper_msgs__msg__StepperCommand * msg = (const stepper_msgs__msg__StepperCommand *)msgin;
    for (int i = 0; i < AXIS_NUM; i++) {
        if (i > msg->position.size) {
            break;
        }
        target_positions[i] = msg->position.data[i];
    }
}

void timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{
	(void) last_call_time;
	if (timer != NULL) {
        stepper_state.position.size = 0;
        stepper_state.velocity.size = 0;
        for (int i = 0; i < AXIS_NUM; i++) {
            if (motor_handle[i] == NULL) {
                continue;
            }
            stepper_state.position.data[i] = stepper_motor_get_position(motor_handle[i]);
            stepper_state.velocity.data[i] = stepper_motor_get_speed(motor_handle[i]);
            stepper_state.position.size++;
            stepper_state.velocity.size++;
        }
		RCSOFTCHECK(rcl_publish(&state_publisher, &stepper_state, NULL));
	}
}

void micro_ros_task(void * arg)
{
    ESP_LOGI("micro_ros", "Starting task");
    rcl_allocator_t allocator = rcl_get_default_allocator();
    rclc_support_t support;

    rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
    RCCHECK(rcl_init_options_init(&init_options, allocator));
    RCCHECK(rcl_init_options_set_domain_id(&init_options, 0));

    // Setup rmw options and ping agent
    rmw_init_options_t* rmw_options = rcl_init_options_get_rmw_init_options(&init_options);
    RCCHECK(rmw_uros_options_set_udp_address(CONFIG_MICRO_ROS_AGENT_IP, CONFIG_MICRO_ROS_AGENT_PORT, rmw_options));
    while (RMW_RET_OK != rmw_uros_ping_agent_options(1000, 1, rmw_options)) {
        ESP_LOGW("mirco_ros", "Could not connect to agent. Retrying...");
    }

    // Create init_options.
    RCCHECK(rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator));

    // Create node.
    char node_name[64];
    snprintf(node_name, sizeof(node_name), "uros_esp32_robot_controller_%d", rand());
    rcl_node_t node = rcl_get_zero_initialized_node();
    rcl_node_options_t node_ops = rcl_node_get_default_options();
    ESP_LOGI("micro_ros", "Creating node: %s", node_name);
    RCCHECK(rclc_node_init_with_options(&node, node_name, "", &support, &node_ops));
	
    // Create publisher.
	RCCHECK(rclc_publisher_init_best_effort(
		&state_publisher,
		&node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(stepper_msgs, msg, StepperState),
		"stepper_state"));

	// Create subscriber.
	RCCHECK(rclc_subscription_init_best_effort(
		&command_subscriber,
		&node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(stepper_msgs, msg, StepperCommand),
		"stepper_command"));
    
    // Allocate memory for stepper messages
    stepper_command.position.capacity = AXIS_NUM;
    stepper_command.position.size = 0;
    stepper_command.position.data = (int32_t*)malloc(AXIS_NUM * sizeof(int32_t));
    stepper_state.position.capacity = AXIS_NUM;
    stepper_state.position.size = 0;
    stepper_state.position.data = (int32_t*)malloc(AXIS_NUM * sizeof(int32_t));
    stepper_state.velocity.capacity = AXIS_NUM;
    stepper_state.velocity.size = 0;
    stepper_state.velocity.data = (int32_t*)malloc(AXIS_NUM * sizeof(int32_t));

	// Create timer.
	rcl_timer_t timer = rcl_get_zero_initialized_timer();
	const unsigned int timer_timeout = 10;
	RCCHECK(rclc_timer_init_default2(
		&timer,
		&support,
		RCL_MS_TO_NS(timer_timeout),
		timer_callback,
		true));

	// Create executor.
	rclc_executor_t executor = rclc_executor_get_zero_initialized_executor();
	RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));
	unsigned int rcl_wait_timeout = 1000;   // in ms
	RCCHECK(rclc_executor_set_timeout(&executor, RCL_MS_TO_NS(rcl_wait_timeout)));

	// Add timer and subscriber to executor.
	RCCHECK(rclc_executor_add_timer(&executor, &timer));
	RCCHECK(rclc_executor_add_subscription(&executor, &command_subscriber, &stepper_command, &command_callback, ON_NEW_DATA));
	
	// Spin forever.
    ESP_LOGI("micro_ros", "Starting micro-ROS executor loop");
    TickType_t ticks = xTaskGetTickCount();
	while(1){
		rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
        xTaskDelayUntil(&ticks, pdMS_TO_TICKS(10));
	}

	// Free resources.
	RCCHECK(rcl_subscription_fini(&command_subscriber, &node));
	RCCHECK(rcl_publisher_fini(&state_publisher, &node));
	RCCHECK(rcl_node_fini(&node));

  	vTaskDelete(NULL);
}
