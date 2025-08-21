
#include "micro_ros_task.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <std_msgs/msg/int32.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>


#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Aborting.\n",__LINE__,(int)temp_rc);vTaskDelete(NULL);}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Continuing.\n",__LINE__,(int)temp_rc);}}

rcl_subscription_t subscriber;
std_msgs__msg__Int32 recv_msg;

extern int32_t micro_ros_data;

void subscription_callback(const void * msgin)
{
	const std_msgs__msg__Int32 * msg = (const std_msgs__msg__Int32 *)msgin;
    micro_ros_data = msg->data; // Store the received data in the global variable
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

	// Create subscriber.
    ESP_LOGI("micro_ros", "Creating subscriber");
	RCCHECK(rclc_subscription_init_best_effort(
		&subscriber,
		&node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
		"potentiometer"));

	// Create executor.
	rclc_executor_t executor = rclc_executor_get_zero_initialized_executor();
	RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));
	unsigned int rcl_wait_timeout = 1000;   // in ms
	RCCHECK(rclc_executor_set_timeout(&executor, RCL_MS_TO_NS(rcl_wait_timeout)));

	// Add timer and subscriber to executor.
	RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &recv_msg, &subscription_callback, ON_NEW_DATA));

	// Spin forever.
    ESP_LOGI("micro_ros", "Starting micro-ROS executor loop");
	while(1){
		rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
		vTaskDelay(pdMS_TO_TICKS(100));
	}

	// Free resources.
	RCCHECK(rcl_subscription_fini(&subscriber, &node));
	RCCHECK(rcl_node_fini(&node));

  	vTaskDelete(NULL);
}