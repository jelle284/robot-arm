#include <stdio.h>
#include <string.h>
#include <unistd.h>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "esp_log.h"
#include "esp_timer.h"

#include "net_connection.h"
#include "motion.h"

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>

#include "stepper_msgs/msg/stepper_point.h"
#include "stepper_msgs/msg/stepper_trajectory.h"
#include "stepper_msgs/msg/stepper_feedback.h"
#include "std_msgs/msg/string.h"
#include "std_msgs/msg/int32.h"

// ROS2 error checking
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Aborting.\n",__LINE__,(int)temp_rc);vTaskDelete(NULL);}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Continuing.\n",__LINE__,(int)temp_rc);}}

// ESP Log tag
static const char* TAG = "main";

// Topics
#define TOPIC_STEPPER_CMD "/stepper_cmd"
#define TOPIC_STEPPER_FB "/stepper_fb"
#define TOPIC_STEPPER_TEST "/stepper_test"

// Publishers and subscribers
rcl_publisher_t feedback_publisher;
rcl_subscription_t stepper_cmd_subscriber;

// Messages
#define MAX_STEPPER_POINTS 32
stepper_msgs__msg__StepperFeedback msg_feedback;
stepper_msgs__msg__StepperTrajectory msg_trajectory;

// Event group
EventGroupHandle_t main_event_group;

/* ============================ SUBSCRIBER PROCESSING ============================*/

void stepper_trajectory_callback(const void* msg_in)
{
    const stepper_msgs__msg__StepperTrajectory* msg = (const stepper_msgs__msg__StepperTrajectory*)msg_in;
    bool err = false;
    for (size_t i = 0; i < msg->points.size; ++i) {
        stepper_msgs__msg__StepperPoint point = msg->points.data[i];
        if (point.steps.size != MOTION_AXIS_NUM) {
            err = true;
            break;
        }
        motion_load_trajectory(point.steps.data, point.duration_ms);
    }
    if(err) {
        ESP_LOGE(TAG, "Point size not matching MOTION_AXIS_NUM");
        return;
    }
    // Notify execution start
    if (msg->partial_count == msg->partial_total) {
        msg_feedback.transmission_id = msg->transmission_id;
        xEventGroupSetBits(main_event_group, BIT1);
    }
}

void stepper_trajectory_debug_callback(const void* msg_in) {
    const stepper_msgs__msg__StepperTrajectory* msg = (const stepper_msgs__msg__StepperTrajectory*)msg_in;
    ESP_LOGI(TAG,
        "Trajectory recieved: transmission id: %lu, \
        number of points: %u, \
        partial %hu of %hu.",
        msg->transmission_id, 
        msg->points.size,
        msg->partial_count, msg->partial_total
    );
}

/* ============================ TASKS ============================*/
void motion_task(void* arg)
{
    ESP_LOGI(TAG, "Configuring motion axis");

    const int pulse_pins[] = {26, 14, 23, 33, 18, 21};
    const int dir_pins[] = {27, 13, 4, 25, 19, 22};
    motion_system_init(pulse_pins, dir_pins);

    ESP_LOGI(TAG, "Motion setup succesfully. Starting transmission loop.");
    for (;;) {
        // Wait for the event to start execution
        xEventGroupWaitBits(main_event_group, BIT1, pdTRUE, pdFALSE, portMAX_DELAY);
        ESP_LOGI(TAG, "Executing motion.");
        motion_execute();
        ESP_LOGI(TAG, "Execution done.");
    }
}

void publish_callback(rcl_timer_t * timer, int64_t last_call_time)
{
	(void) last_call_time;
	if (timer != NULL) {
        motion_feedback_t fb;
        if(motion_get_feedback(&fb) > 0) {
            msg_feedback.steps_executed.data = fb.steps_executed;
            msg_feedback.steps_executed.capacity = MOTION_AXIS_NUM;
            msg_feedback.steps_executed.size = MOTION_AXIS_NUM;
            msg_feedback.current_point = fb.current_point;
            msg_feedback.total_points = fb.total_points;
            RCCHECK(rcl_publish(&feedback_publisher, &msg_feedback, NULL));
        }
	}
}

void micro_ros_task(void * arg)
{
    ESP_LOGI(TAG,"Micro-ros task entry.");
    rcl_allocator_t allocator = rcl_get_default_allocator();

    // Create init_options.
    rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
    RCCHECK(rcl_init_options_init(&init_options, allocator));
    RCCHECK(rcl_init_options_set_domain_id(&init_options, 1));

    // Setup rmw options and ping agent
    rmw_init_options_t* rmw_options = rcl_init_options_get_rmw_init_options(&init_options);
    RCCHECK(rmw_uros_options_set_udp_address(CONFIG_MICRO_ROS_AGENT_IP, CONFIG_MICRO_ROS_AGENT_PORT, rmw_options));
    while (RMW_RET_OK != rmw_uros_ping_agent_options(1000, 1, rmw_options)) {
        ESP_LOGW(TAG, "Could not connect to agent. Retrying...");
    }

    // Setup support structure.
    rclc_support_t support;
    RCCHECK(rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator));
    
    // Create node.
    char node_name[64];
    snprintf(node_name, sizeof(node_name), "uros_esp32_robot_controller_%d", rand());
    rcl_node_t node = rcl_get_zero_initialized_node();
    rcl_node_options_t node_ops = rcl_node_get_default_options();
    RCCHECK(rclc_node_init_with_options(&node, node_name, "", &support, &node_ops));

    // Create executor.
    rclc_executor_t executor = rclc_executor_get_zero_initialized_executor();
    RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));
    unsigned int rcl_wait_timeout = 1000;   // in ms
    RCCHECK(rclc_executor_set_timeout(&executor, RCL_MS_TO_NS(rcl_wait_timeout)));

    // Set Quality of service
    rmw_qos_profile_t qos;
    qos = rmw_qos_profile_default;
    qos.durability = RMW_QOS_POLICY_DURABILITY_VOLATILE;
    qos.history = RMW_QOS_POLICY_HISTORY_SYSTEM_DEFAULT;
    qos.reliability = RMW_QOS_POLICY_RELIABILITY_RELIABLE;
    qos.depth=1;

    // Create subscriber.
    RCCHECK(rclc_subscription_init(
        &stepper_cmd_subscriber,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(stepper_msgs, msg, StepperTrajectory),
        TOPIC_STEPPER_CMD,
        &qos
    ));

    // Allocate memory for StepperTrajectory
    msg_trajectory.partial_count = 0;
    msg_trajectory.partial_total = 0;
    msg_trajectory.transmission_id = 0;

    stepper_msgs__msg__StepperPoint msg_points[MAX_STEPPER_POINTS];
    msg_trajectory.points.capacity = MAX_STEPPER_POINTS;
    msg_trajectory.points.size = 0;
    msg_trajectory.points.data = msg_points;

    int16_t points_data[MAX_STEPPER_POINTS][MOTION_AXIS_NUM];
    for (size_t i = 0; i < MAX_STEPPER_POINTS; ++i) {
        msg_points[i].duration_ms = 0;
        msg_points[i].steps.capacity = MOTION_AXIS_NUM;
        msg_points[i].steps.size = 0;
        msg_points[i].steps.data = points_data[i];
    }

    // Add subscribers to executor.
    RCCHECK(rclc_executor_add_subscription(
        &executor, &stepper_cmd_subscriber, &msg_trajectory,
        &stepper_trajectory_callback,
        //&stepper_trajectory_debug_callback,
        ON_NEW_DATA));

    // Create publishers.
    #if 1
    RCCHECK(rclc_publisher_init(
        &feedback_publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(stepper_msgs, msg, StepperFeedback),
        TOPIC_STEPPER_FB,
        &qos
    ));

	// create timer
	rcl_timer_t timer;
	RCCHECK(rclc_timer_init_default2(
		&timer,
		&support,
		RCL_MS_TO_NS(200),
		publish_callback,
        true));

	RCCHECK(rclc_executor_add_timer(&executor, &timer));
    #endif

    // Spin forever.
    ESP_LOGI(TAG,"Spinning micro-ros executor...");
	while(1){
		rclc_executor_spin_some(&executor, RCL_MS_TO_NS(20));
		usleep(80000);
	}

    // Free resources.
    RCCHECK(rcl_subscription_fini(&stepper_cmd_subscriber, &node));
    RCCHECK(rcl_publisher_fini(&feedback_publisher, &node));
    RCCHECK(rcl_node_fini(&node));

    vTaskDelete(NULL);
}

/* ============================ APP MAIN ============================*/
void app_main() 
{   
    esp_log_level_set("*", ESP_LOG_NONE);
    network_init();
    main_event_group = xEventGroupCreate();
    xTaskCreate(micro_ros_task, "uros_task", 8192, NULL, 5, NULL);
    xTaskCreate(motion_task, "motion_task", 4096, NULL, 4, NULL);
}
