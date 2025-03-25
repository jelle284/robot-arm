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
#include "stepper_msgs/msg/stepper_point.h"
#include "stepper_msgs/msg/stepper_feedback.h"

// ROS2 error checking
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Aborting.\n",__LINE__,(int)temp_rc);vTaskDelete(NULL);}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Continuing.\n",__LINE__,(int)temp_rc);}}

// ESP Log tag
static const char* TAG = "main";

// Motion axis setup
#define NAXIS 6
motion_axis_handle_t motion_axis[NAXIS];

// Topics
#define TOPIC_STEPPER_POINT "/stepper_point"
#define TOPIC_STEPPER_FB "/stepper_fb"

// State management
unsigned int execution_state = stepper_msgs__msg__StepperFeedback__STATUS_IDLE;

// Publishers and subscribers
rcl_publisher_t feedback_publisher;
rcl_subscription_t stepper_point_subscriber;

// Event group
EventGroupHandle_t main_event_group;

/* ============================ CALLBACKS ============================*/
void stepper_point_callback(const void* msg_in)
{
    if (execution_state != stepper_msgs__msg__StepperFeedback__STATUS_IDLE) {
        // Discard points if not in IDLE or LOADING state
        ESP_LOGW(TAG, "Discarding point. Execution state: %u", execution_state);
        return;
    }

    const stepper_msgs__msg__StepperPoint* msg = (const stepper_msgs__msg__StepperPoint*)msg_in;
    if (msg->command == stepper_msgs__msg__StepperPoint__COMMAND_LOAD) {
        for (int i = 0; i < NAXIS; ++i) {
            motion_instruction_t cmd;
            cmd.steps = msg->steps.data[i];
            // If we have zero steps we just make 1 ms zero level pulses
            if (cmd.steps == 0) {
                cmd.steps = msg->duration_ms;
                cmd.pulse_us = 1000;
                cmd.level = 0;
            } else {
                // first we check for potential overflow in pulse micros
                // if we have overflow we shorten the duration and add an extra delay instruction
                // to make up the time
                uint32_t temp_us = (1000*msg->duration_ms) / abs(cmd.steps);
                if (temp_us > UINT16_MAX) { 
                    ESP_LOGW(TAG, "overflow in pulse width!");
                    uint16_t max_duration_ms = abs(cmd.steps)*(UINT16_MAX/1000);
                    uint16_t delay_ms = msg->duration_ms - max_duration_ms;
                    motion_instruction_t delay_cmd = {
                        .level = 0,
                        .steps = delay_ms,
                        .pulse_us = 1000
                    };
                    motion_axis_load_trajectory(motion_axis[i], &delay_cmd);
                    temp_us = UINT16_MAX;
                    ESP_LOGW(TAG, "Overflow in pulse width. Inserting delay of %hu", delay_ms);
                } 
                cmd.pulse_us = temp_us;
                cmd.level = 1;
            }
            motion_axis_load_trajectory(motion_axis[i], &cmd);
            ESP_LOGI(TAG, "Loaded point on axis %d. steps=%hd, duration=%hu", i, cmd.steps, cmd.pulse_us);
        }
    }
    if (msg->command == stepper_msgs__msg__StepperPoint__COMMAND_EXECUTE) {
        xEventGroupSetBits(main_event_group, BIT0);
    }
}

/* ============================ TASKS ============================*/
void motion_task(void* arg)
{
    ESP_LOGI(TAG, "Configuring motion axis");

    motion_system_init();
    
    const int pulse_pins[] = {26, 14, 23, 33, 18, 21};
    const int dir_pins[] = {27, 13, 4, 25, 19, 22};
    for (int i = 0; i < NAXIS; ++i) {
        motion_axis[i] = motion_axis_create(pulse_pins[i], dir_pins[i]);
        motion_axis_reset(motion_axis[i]);
    }
    
    ESP_LOGI(TAG, "Motion setup succesfully. Starting transmission loop.");
    for (;;) {
        // Wait for the event to start execution
        xEventGroupWaitBits(main_event_group, BIT0, pdTRUE, pdFALSE, portMAX_DELAY);
        ESP_LOGI(TAG, "Transmission event occured.");

        if (execution_state == stepper_msgs__msg__StepperFeedback__STATUS_IDLE) {
            for (int i = 0; i < NAXIS; ++i) {
                motion_axis_execute(motion_axis[i]);
            }
            execution_state = stepper_msgs__msg__StepperFeedback__STATUS_EXECUTE;
            ESP_LOGI(TAG, "Execution transmitted. Execution state: %d", execution_state);
            while(motion_event_await() == 0) { }
            ESP_LOGI(TAG, "Execution finished. Execution state: %d", execution_state);
            for (int i = 0; i < NAXIS; ++i) {
                motion_axis_reset(motion_axis[i]);
            }
            execution_state = stepper_msgs__msg__StepperFeedback__STATUS_IDLE;
            ESP_LOGI(TAG, "Axis has been reset. Execution state: %d", execution_state);
        }
    }
}

void publisher_task(void *arg)
{
    /*
    ESP_LOGI(TAG, "publisher_task entry");

    stepper_msgs__msg__StepperFeedback feedback_msg;
    int16_t memory[NAXIS];
    feedback_msg.steps_executed.data = memory;
    feedback_msg.steps_executed.capacity = NAXIS;
    feedback_msg.steps_executed.size = 0;
    feedback_msg.status = 0;

    vTaskDelay(pdMS_TO_TICKS(1000));
    ESP_LOGI(TAG, "start feedback publishing");
    for (;;) {
        feedback_msg.steps_executed.size = 0;
        for (int i = 0; i < NAXIS; ++i) {
            feedback_msg.steps_executed.data[i] = i; //TODO
            feedback_msg.steps_executed.size++;
        }
        feedback_msg.status = (uint8_t)execution_state;
        RCCHECK(rcl_publish(&feedback_publisher, &feedback_msg, NULL));
        vTaskDelay(pdMS_TO_TICKS(200));
    }
    // Clean up
    free(feedback_msg.steps_executed.data);
    */
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
    RCCHECK(rclc_subscription_init_default(
        &stepper_point_subscriber,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(stepper_msgs, msg, StepperPoint),
        TOPIC_STEPPER_POINT));

    // Create publishers.
    RCCHECK(rclc_publisher_init_default(
        &feedback_publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(stepper_msgs, msg, StepperFeedback),
        TOPIC_STEPPER_FB));

    // Create executor.
    rclc_executor_t executor = rclc_executor_get_zero_initialized_executor();
    RCCHECK(rclc_executor_init(&executor, &support.context, 4, &allocator));
    unsigned int rcl_wait_timeout = 1000;   // in ms
    RCCHECK(rclc_executor_set_timeout(&executor, RCL_MS_TO_NS(rcl_wait_timeout)));

    // Allocate message memory on the stack.
    stepper_msgs__msg__StepperPoint recv_point_msg;
    int16_t point_steps[NAXIS];
    recv_point_msg.steps.capacity = NAXIS;
    recv_point_msg.steps.data = point_steps;
    recv_point_msg.steps.size = 0;
    recv_point_msg.command = 0;
    recv_point_msg.duration_ms = 0;

    // Add subscribers to executor.
    RCCHECK(rclc_executor_add_subscription(&executor, &stepper_point_subscriber, &recv_point_msg, &stepper_point_callback, ON_NEW_DATA));
    
    // Spin forever.
    while(1){
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
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
    esp_log_level_set("*", ESP_LOG_INFO);
    network_init();
    main_event_group = xEventGroupCreate();
    xTaskCreate(motion_task, "motion_task", 4096, NULL, 5, NULL);
    xTaskCreate(micro_ros_task, "uros_task", 4096, NULL, 5, NULL);
    //xTaskCreate(publisher_task, "publisher_task", 2048, NULL, 5, NULL);
}
