#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
static const char *TAG = "main";

// For wifi connection
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

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Aborting.\n",__LINE__,(int)temp_rc);vTaskDelete(NULL);}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Continuing.\n",__LINE__,(int)temp_rc);}}

rcl_publisher_t publisher;
rcl_subscription_t subscriber;
stepper_msgs__msg__StepperCommand stepper_command;;
stepper_msgs__msg__StepperState stepper_state;

void micro_ros_task(void* arg);

// For stepper motor control
#include "stepper_motor.h"
#define AXIS_NUM 6 // Number of stepper motors
stepper_motor_handle_t motor_handle[AXIS_NUM];
const int pulse_pins[] = {26, 14, 23, 33, 18, 21};
const int dir_pins[] = {27, 13, 4, 25, 19, 22};

// Main application
void app_main() 
{
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
}

// Micro-ROS stuff below
void subscription_callback(const void * msgin)
{
	const stepper_msgs__msg__StepperCommand * msg = (const stepper_msgs__msg__StepperCommand *)msgin;
    for (int i = 0; i < AXIS_NUM; i++) {
        if (i > msg->velocity.size) {
            break;
        }
        if (motor_handle[i] == NULL) {
            continue;
        }
        stepper_motor_set_speed(motor_handle[i], msg->velocity.data[i]);
    }
}

void timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{
	(void) last_call_time;
	if (timer != NULL) {
        for (int i = 0; i < AXIS_NUM; i++) {
            if (motor_handle[i] == NULL) {
                continue;
            }
            stepper_state.position.data[i] = stepper_motor_get_position(motor_handle[i]);
            stepper_state.velocity.data[i] = stepper_motor_get_speed(motor_handle[i]);
        }
		RCSOFTCHECK(rcl_publish(&publisher, &stepper_state, NULL));
	}
}

void micro_ros_task(void * arg)
{
    ESP_LOGI("micro_ros", "Starting task");
    rcl_allocator_t allocator = rcl_get_default_allocator();
    rclc_support_t support;

    rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
    RCCHECK(rcl_init_options_init(&init_options, allocator));
    RCCHECK(rcl_init_options_set_domain_id(&init_options, 1));

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
		&publisher,
		&node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(stepper_msgs, msg, StepperState),
		"stepper_state"));

	// Create subscriber.
	RCCHECK(rclc_subscription_init_best_effort(
		&subscriber,
		&node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(stepper_msgs, msg, StepperCommand),
		"stepper_command"));
    
    // Allocate memory for stepper messages
    stepper_command.velocity.capacity = AXIS_NUM;
    stepper_command.velocity.size = 0;
    stepper_command.velocity.data = (int32_t*)malloc(AXIS_NUM * sizeof(int32_t));
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
	RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &stepper_command, &subscription_callback, ON_NEW_DATA));

	// Spin forever.
    ESP_LOGI("micro_ros", "Starting micro-ROS executor loop");
    TickType_t ticks = xTaskGetTickCount();
	while(1){
		rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
        xTaskDelayUntil(&ticks, pdMS_TO_TICKS(10));
	}

	// Free resources.
	RCCHECK(rcl_subscription_fini(&subscriber, &node));
	RCCHECK(rcl_publisher_fini(&publisher, &node));
	RCCHECK(rcl_node_fini(&node));

  	vTaskDelete(NULL);
}