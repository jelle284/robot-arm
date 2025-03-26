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

// ROS2 error checking
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Aborting.\n",__LINE__,(int)temp_rc);vTaskDelete(NULL);}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Continuing.\n",__LINE__,(int)temp_rc);}}

// ESP Log tag
static const char* TAG = "main";

// Motion axis setup
motion_axis_handle_t motion_axis[MOTION_AXIS_NUM];

// Topics
#define TOPIC_STEPPER_POINT "/stepper_point"
#define TOPIC_STEPPER_FB "/stepper_fb"

// Publishers and subscribers
rcl_publisher_t feedback_publisher;
rcl_subscription_t stepper_point_subscriber;

// Event group
EventGroupHandle_t main_event_group;

/* ============================ SUBSCRIBER PROCESSING ============================*/
void process_axis(const stepper_msgs__msg__StepperPoint* point, size_t axis_num) {
    if (axis_num > point->steps.size) {
        ESP_LOGE(TAG,"Invalid axis_num in process_axis!");
        return;
    }
    motion_instruction_t cmd;
    cmd.steps = point->steps.data[axis_num];
    // If we have zero steps we just make 1 ms zero level pulses
    if (cmd.steps == 0) {
        cmd.steps = point->duration_ms;
        cmd.pulse_us = 1000;
        cmd.level = 0;
    } else {
        // first we check for potential overflow in pulse micros
        // if we have overflow we shorten the duration and add an extra delay instruction
        // to make up the time
        uint32_t temp_us = (1000*point->duration_ms) / abs(cmd.steps);
        if (temp_us > UINT16_MAX) { 
            uint16_t max_duration_ms = abs(cmd.steps)*(UINT16_MAX/1000);
            uint16_t delay_ms = point->duration_ms - max_duration_ms;
            motion_instruction_t delay_cmd = {
                .level = 0,
                .steps = delay_ms,
                .pulse_us = 1000
            };
            motion_axis_load_trajectory(motion_axis[axis_num], delay_cmd);
            temp_us = UINT16_MAX;
            ESP_LOGW(TAG, "Overflow in pulse width. Inserting delay of %hu", delay_ms);
        } 
        cmd.pulse_us = temp_us;
        cmd.level = 1;
    }
    motion_axis_load_trajectory(motion_axis[axis_num], cmd);
}
void stepper_trajectory_callback(const void* msg_in)
{
    const stepper_msgs__msg__StepperTrajectory* msg = (const stepper_msgs__msg__StepperTrajectory*)msg_in;

    for (size_t i = 0; i < msg->points.size; ++i) {
        stepper_msgs__msg__StepperPoint point = msg->points.data[i];
        for (size_t n = 0; n < MOTION_AXIS_NUM; ++n) {
            process_axis(&point, n);
        }
    }
    // Notify execution start
    xEventGroupSetBits(main_event_group, BIT1);
}

/* ============================ TASKS ============================*/
void motion_task(void* arg)
{
    ESP_LOGI(TAG, "Configuring motion axis");

    motion_system_init();
    
    const int pulse_pins[] = {26, 14, 23, 33, 18, 21};
    const int dir_pins[] = {27, 13, 4, 25, 19, 22};
    for (int i = 0; i < MOTION_AXIS_NUM; ++i) {
        motion_axis[i] = motion_axis_create(pulse_pins[i], dir_pins[i]);
        motion_axis_reset(motion_axis[i]);
    }
    
    ESP_LOGI(TAG, "Motion setup succesfully. Starting transmission loop.");
    for (;;) {
        // Wait for the event to start execution
        xEventGroupWaitBits(main_event_group, BIT1, pdTRUE, pdFALSE, portMAX_DELAY);
        ESP_LOGI(TAG, "Transmission event occured.");

        for (int i = 0; i < MOTION_AXIS_NUM; ++i) {
            motion_axis_execute(motion_axis[i]);
        }
        ESP_LOGI(TAG, "Execution transmitted.");
        motion_await_done();
        for (int i = 0; i < MOTION_AXIS_NUM; ++i) {
            motion_axis_reset(motion_axis[i]);
        }
    }
}

void publisher_task(void *arg)
{
    int32_t feedback[MOTION_AXIS_NUM];
    for (;;) {
        motion_get_feedback(feedback);

        /* Log feedback */
        if (esp_log_level_get(TAG) == ESP_LOG_INFO) {
            char log_message[256];
            int offset = sprintf(log_message, "Feedback ready: ");
            for (int i = 0; i < MOTION_AXIS_NUM; ++i) {
                offset += sprintf(log_message + offset, "%ld ", feedback[i]);
            }
            ESP_LOGI(TAG, "%s", log_message);
        }
        
        /* Publish feedback */
        stepper_msgs__msg__StepperFeedback feedback_msg;
        feedback_msg.steps_executed.data = feedback;
        feedback_msg.steps_executed.capacity = MOTION_AXIS_NUM;
        feedback_msg.steps_executed.size = MOTION_AXIS_NUM;
        // TODO: real values for points
        feedback_msg.current_point = 0;
        feedback_msg.total_points = 0;
        RCCHECK(rcl_publish(&feedback_publisher, &feedback_msg, NULL));
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

    // Create subscribers.
    RCCHECK(rclc_subscription_init_best_effort(
        &stepper_point_subscriber,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(stepper_msgs, msg, StepperTrajectory),
        TOPIC_STEPPER_POINT));

    // Create publishers.
    RCCHECK(rclc_publisher_init_best_effort(
        &feedback_publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(stepper_msgs, msg, StepperFeedback),
        TOPIC_STEPPER_FB));

    // Create executor.
    rclc_executor_t executor = rclc_executor_get_zero_initialized_executor();
    RCCHECK(rclc_executor_init(&executor, &support.context, 4, &allocator));
    unsigned int rcl_wait_timeout = 1000;   // in ms
    RCCHECK(rclc_executor_set_timeout(&executor, RCL_MS_TO_NS(rcl_wait_timeout)));

    // Allocate memory for StepperTrajectory
    stepper_msgs__msg__StepperTrajectory msg_trajectory;

    stepper_msgs__msg__StepperPoint msg_points[MOTION_BUFFER_SIZE];
    int16_t points_data[MOTION_BUFFER_SIZE][MOTION_AXIS_NUM];
    msg_trajectory.points.capacity = MOTION_BUFFER_SIZE;
    msg_trajectory.points.size = 0;
    msg_trajectory.points.data = msg_points;
    for (size_t i = 0; i < MOTION_BUFFER_SIZE; ++i) {
        msg_points[i].duration_ms = 0;
        msg_points[i].steps.capacity = MOTION_AXIS_NUM;
        msg_points[i].steps.size = 0;
        msg_points[i].steps.data = points_data[i];
    }

    std_msgs__msg__String msg_strings[MOTION_AXIS_NUM];
    char motor_names_data [MOTION_AXIS_NUM][16];
    msg_trajectory.motor_names.capacity = MOTION_AXIS_NUM;
    msg_trajectory.motor_names.size = 0;
    msg_trajectory.motor_names.data = msg_strings;
    for (size_t i = 0; i < MOTION_AXIS_NUM; ++i) {
        msg_trajectory.motor_names.data[i].capacity = 16;
        msg_trajectory.motor_names.data[i].size = 0;
        msg_trajectory.motor_names.data[i].data = motor_names_data[i];
    }

    // Add subscribers to executor.
    RCCHECK(rclc_executor_add_subscription(&executor, &stepper_point_subscriber, &msg_trajectory, &stepper_trajectory_callback, ON_NEW_DATA));
    
    // Spin forever.
	while(1){
		rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
		usleep(100000);
	}

    // Free resources.
    RCCHECK(rcl_subscription_fini(&stepper_point_subscriber, &node));
    RCCHECK(rcl_publisher_fini(&feedback_publisher, &node));
    RCCHECK(rcl_node_fini(&node));

    vTaskDelete(NULL);
}

/* ============================ APP MAIN ============================*/
void app_main() 
{   
    esp_log_level_set("*", ESP_LOG_WARN);
    network_init();
    main_event_group = xEventGroupCreate();
    xTaskCreate(motion_task, "motion_task", 4096, NULL, 5, NULL);
    xTaskCreate(micro_ros_task, "uros_task", 16384, NULL, 5, NULL);
    xTaskCreate(publisher_task, "publisher_task", 4096, NULL, 5, NULL);
}
