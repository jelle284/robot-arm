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
#include "trajectory_msgs/msg/joint_trajectory_point.h"

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Aborting.\n",__LINE__,(int)temp_rc);vTaskDelete(NULL);}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Continuing.\n",__LINE__,(int)temp_rc);}}

//#define LOG_LEVEL ESP_LOG_ERROR

#define NAXIS 6
#define STEPS_PR_REV 800

static const char* TAG = "main";

QueueHandle_t motion_queue;
motion_axis_t* motion_axis[NAXIS];

typedef struct {
    float positions[NAXIS];
    float duration;
} RobotMove;

void motion_task(void* arg) {

    ESP_LOGI(TAG, "Configuring motion axis");
    const int pulse_pins[] = {26, 14, 23, 33, 18, 21};
    const int dir_pins[] = {27, 13, 4, 25, 19, 22};
    for (int i = 0; i < NAXIS; ++i) {
        motion_axis[i] = motion_axis_create(pulse_pins[i], dir_pins[i], STEPS_PR_REV);
    }
    motion_axis[0]->gear_ratio = 56.0f * 50.0f/20.0f;
    motion_axis[1]->gear_ratio = 56.0f;
    motion_axis[2]->gear_ratio = 56.0f;
    motion_axis[3]->gear_ratio = 40.25f * 50.0f/16.0f;
    motion_axis[4]->gear_ratio = 80.0f/28.0f * 80.0f/12.0f;
    motion_axis[5]->gear_ratio = 80.0f/28.0f * 80.0f/12.0f;

    motion_queue = xQueueCreate(32, sizeof(RobotMove));
    if (!motion_queue) {
        ESP_LOGE(TAG, "Failed to create motion queue");
        return;
    }

    for (;;) {
        RobotMove move;
        if (xQueueReceive(motion_queue, &move, portMAX_DELAY)) {
            for (int i = 0; i < NAXIS; ++i) {
                if (fabsf(move.positions[i]) < 0.001f) continue;
                motion_instruction_t instr = {
                    .angle = move.positions[i]*180.0f/3.141592f, // rad to deg
                    .duration = move.duration,
                    .profile = MOTION_PROFILE_SCURVE
                };
                ESP_LOGI(TAG, "Executing move on axis %d: angle=%.2f, duration=%.2f", i, instr.angle, instr.duration);
                motion_axis_move(motion_axis[i], instr);
            }
        }
    }
}

void subscription_callback(const void* msg_in) {
    const trajectory_msgs__msg__JointTrajectoryPoint* msg = (const trajectory_msgs__msg__JointTrajectoryPoint*)msg_in;

    ESP_LOGI(TAG, "Positions size: %d", msg->positions.size);
    ESP_LOGI(TAG, "Velocities size: %d", msg->velocities.size);
    ESP_LOGI(TAG, "Accelerations size: %d", msg->accelerations.size);
    ESP_LOGI(TAG, "Effort size: %d", msg->effort.size);
    
    
    ESP_LOGI(TAG, "  Positions: ");
    for (int i = 0; i < msg->positions.size; ++i) {
        ESP_LOGI(TAG, "    [%d]: %f", i, msg->positions.data[i]);
    }
    ESP_LOGI(TAG, "  Velocities: ");
    for (int i = 0; i < msg->velocities.size; ++i) {
        ESP_LOGI(TAG, "    [%d]: %f", i, msg->velocities.data[i]);
    }
    ESP_LOGI(TAG, "  Accelerations: ");
    for (int i = 0; i < msg->accelerations.size; ++i) {
        ESP_LOGI(TAG, "    [%d]: %f", i, msg->accelerations.data[i]);
    }
    ESP_LOGI(TAG, "  Effort: ");
    for (int i = 0; i < msg->effort.size; ++i) {
        ESP_LOGI(TAG, "    [%d]: %f", i, msg->effort.data[i]);
    }
    ESP_LOGI(TAG, "  Time from Start: %ld sec, %ld nsec", msg->time_from_start.sec, msg->time_from_start.nanosec);

    RobotMove move = {0};

    if (NAXIS == msg->positions.size) {
        move.positions[0] = -1.0f * msg->positions.data[0];
        move.positions[1] = 1.0f * msg->positions.data[1];
        move.positions[2] = 1.0f * msg->positions.data[2];
        move.positions[3] = -1.0f * msg->positions.data[3];
        // differential joint
        move.positions[4] = -1.0f*msg->positions.data[4] + -1.0f*msg->positions.data[5];
        move.positions[5] = -1.0f*msg->positions.data[4] + 1.0f*msg->positions.data[5];
    }

    move.duration = msg->time_from_start.sec + msg->time_from_start.nanosec / 1e9;
    if (xQueueSend(motion_queue, &move, portMAX_DELAY) != pdTRUE) {
        ESP_LOGE(TAG, "Failed to enqueue motion command");
    }
}

void micro_ros_task(void * arg) {
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
		ROSIDL_GET_MSG_TYPE_SUPPORT(trajectory_msgs, msg, JointTrajectoryPoint),
		"/cmd_point"));

	// Create executor.
	rclc_executor_t executor = rclc_executor_get_zero_initialized_executor();
	RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));
	unsigned int rcl_wait_timeout = 1000;   // in ms
	RCCHECK(rclc_executor_set_timeout(&executor, RCL_MS_TO_NS(rcl_wait_timeout)));

	// Add subscriber to executor.
    trajectory_msgs__msg__JointTrajectoryPoint recv_msg;
	RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &recv_msg, &subscription_callback, ON_NEW_DATA));

    // Allocate message memory
    double msg_positions[NAXIS];
    recv_msg.positions.capacity = NAXIS;
    recv_msg.positions.data = msg_positions;
    recv_msg.positions.size = 0;
    recv_msg.time_from_start.sec = 0; 
    recv_msg.time_from_start.nanosec = 0;

	// Spin forever.
	while(1){
		rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
        rclc_sleep_ms(100);
	}

	// Free resources.
	RCCHECK(rcl_subscription_fini(&subscriber, &node));
	RCCHECK(rcl_node_fini(&node));

  	vTaskDelete(NULL);
}

void app_main() 
{   
    network_init();
    xTaskCreate(motion_task, "motion_task", 4096, NULL, 5, NULL);
    xTaskCreate(micro_ros_task, "uros_task", 32000, NULL, 5, NULL);
}
