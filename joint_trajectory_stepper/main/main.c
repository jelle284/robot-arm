#include <stdio.h>
#include <string.h>
#include <math.h>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include "esp_log.h"
#include "esp_timer.h"

#include "net_connection.h"
#include "motion.h"

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <std_msgs/msg/int32.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>
#include "stepper_msgs/msg/stepper_trajectory.h"

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Aborting.\n",__LINE__,(int)temp_rc);vTaskDelete(NULL);}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Continuing.\n",__LINE__,(int)temp_rc);}}


#define NAXIS 6

static const char* TAG = "main";

QueueHandle_t motion_queue;
motion_axis_t* motion_axis[NAXIS];

typedef struct {
    motion_instruction_t axis_cmd[NAXIS];
} move_group_instruction_t;

void motion_task(void* arg) 
{
    ESP_LOGI(TAG, "Configuring motion axis");
    const int pulse_pins[] = {26, 14, 23, 33, 18, 21};
    const int dir_pins[] = {27, 13, 4, 25, 19, 22};
    for (int i = 0; i < NAXIS; ++i) {
        motion_axis[i] = motion_axis_create(pulse_pins[i], dir_pins[i]);
        motion_axis_reset(motion_axis[i]);
    }

    motion_queue = xQueueCreate(32, sizeof(move_group_instruction_t));
    if (!motion_queue) {
        ESP_LOGE(TAG, "Failed to create motion queue");
        return;
    }

    for (;;) {
        move_group_instruction_t move;
        if (xQueueReceive(motion_queue, &move, portMAX_DELAY)) {
            for (int i = 0; i < NAXIS; ++i) {
                ESP_LOGI(TAG, "Executing move on axis %d: angle=%ld, duration=%hu",
                     i, move.axis_cmd[i].steps, move.axis_cmd[i].pulse_us);
                motion_axis_move(motion_axis[i], move.axis_cmd[i]);
            }
        }
        
    }
}

void subscription_callback(const void* msg_in) 
{
    const stepper_msgs__msg__StepperTrajectoryPoint* msg = (const stepper_msgs__msg__StepperTrajectoryPoint*)msg_in;

    move_group_instruction_t move = {0};

    for (int i = 0; i < NAXIS; ++i) {
        move.axis_cmd[i].steps = msg->steps.data[i];
        if (abs(move.axis_cmd[i].steps) > 0) {
            move.axis_cmd[i].pulse_us = msg->duration_us / move.axis_cmd[i].steps;
        } else {
            move.axis_cmd[i].pulse_us = 0;
        }
    }
    if (xQueueSend(motion_queue, &move, 0) != pdTRUE) {
        ESP_LOGE(TAG, "Failed to enqueue motion command");
    }
}

void feedback_task(void *arg) 
{
    ESP_LOGI(TAG, "Feedback task entry");
    for (;;) {
        motion_event_await(motion_axis, NAXIS);
    }
}

void micro_ros_task(void * arg) 
{
	rcl_allocator_t allocator = rcl_get_default_allocator();
	rclc_support_t support;

	// Create init_options.
	rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
	RCCHECK(rcl_init_options_init(&init_options, allocator));
    RCCHECK(rcl_init_options_set_domain_id(&init_options, 1));

	// Static Agent IP and port can be used instead of autodiscovery. 
    /**/
    rmw_init_options_t* rmw_options = rcl_init_options_get_rmw_init_options(&init_options);
    const char ip[] = "192.168.0.10";
    const char port[] = "8888";
    RCCHECK(rmw_uros_options_set_udp_address(ip, port, rmw_options));
    

	// Setup support structure.
	RCCHECK(rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator));

	// Create node.
    char node_name[64];
    snprintf(node_name, sizeof(node_name), "uros_esp32_robot_controller_%d", rand());
    rcl_node_t node = rcl_get_zero_initialized_node();
    RCCHECK(rclc_node_init_default(&node, node_name, "", &support));
    
	// Create subscriber.
    rcl_subscription_t subscriber;
	RCCHECK(rclc_subscription_init_default(
		&subscriber,
		&node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(stepper_msgs, msg, StepperTrajectoryPoint),
		"/cmd_point"));

	// Create executor.
	rclc_executor_t executor = rclc_executor_get_zero_initialized_executor();
	RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));
	unsigned int rcl_wait_timeout = 1000;   // in ms
	RCCHECK(rclc_executor_set_timeout(&executor, RCL_MS_TO_NS(rcl_wait_timeout)));

    // Allocate message memory
    stepper_msgs__msg__StepperTrajectoryPoint recv_msg;
    int32_t msg_steps[NAXIS];
    recv_msg.steps.capacity = NAXIS;
    recv_msg.steps.data = msg_steps;
    recv_msg.steps.size = 0;
    recv_msg.duration_us = 0; 

	// Add subscriber to executor.
    RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &recv_msg, &subscription_callback, ON_NEW_DATA));

	// Spin forever.
	while(1){
		rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
        //rclc_sleep_ms(100);
	}

	// Free resources.
	RCCHECK(rcl_subscription_fini(&subscriber, &node));
	RCCHECK(rcl_node_fini(&node));

  	vTaskDelete(NULL);
}

void app_main() 
{   
    esp_log_level_set("*", ESP_LOG_WARN);
    network_init();
    motion_system_init();
    xTaskCreate(motion_task, "motion_task", 4096, NULL, 5, NULL);
    xTaskCreate(feedback_task, "feedback_task", 4096, NULL, 5, NULL);
    xTaskCreate(micro_ros_task, "uros_task", 32000, NULL, 5, NULL);
}
