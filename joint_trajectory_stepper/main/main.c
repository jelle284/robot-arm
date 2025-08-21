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

#include <stepper_msgs/msg/stepper_trajectory_point.h>
#include <stepper_msgs/msg/stepper_trajectory.h>
#include <stepper_msgs/action/follow_stepper_trajectory.h>
typedef stepper_msgs__action__FollowStepperTrajectory_SendGoal_Request goal_req_t;
typedef stepper_msgs__msg__StepperTrajectoryPoint point_t;
typedef stepper_msgs__action__FollowStepperTrajectory_FeedbackMessage feedback_t;
typedef stepper_msgs__action__FollowStepperTrajectory_GetResult_Response response_t;
// ROS2 error checking
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Aborting.\n",__LINE__,(int)temp_rc);vTaskDelete(NULL);}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Continuing.\n",__LINE__,(int)temp_rc);}}

// ESP Log tag
static const char* TAG = "main";

// Topics
#define ACTION_NAME "/follow_stepper_trajectory"

// Event group
EventGroupHandle_t main_event_group;

/* ============================ GOAL HANDLING ============================*/
#define VMIN 1000*1000

void process_goal(void* arg) {
    ESP_LOGI(TAG, "Processing goal...");
    // recieve goal
    rclc_action_goal_handle_t * goal_handle = (rclc_action_goal_handle_t *) arg;
    goal_req_t* req = (goal_req_t*)goal_handle->ros_goal_request;
    for (size_t i = 0; i < req->goal.trajectory.points.size; ++i) {
        point_t p = req->goal.trajectory.points.data[i];
        if (p.accelerations.size == MOTION_AXIS_NUM
            && p.velocities.size == MOTION_AXIS_NUM
            && p.positions.size == MOTION_AXIS_NUM) {
                motion_load_trajectory(
                    p.accelerations.data,
                    p.velocities.data,
                    p.positions.data,
                    p.time_from_start);
            }
    }
    xEventGroupSetBits(main_event_group, BIT1);
    // init feedback
    motion_feedback_t motion_fb;
    feedback_t feedback_msg;
    feedback_msg.feedback.current_positions.capacity = MOTION_AXIS_NUM;
    feedback_msg.feedback.current_positions.size = MOTION_AXIS_NUM;
    feedback_msg.feedback.current_positions.data = motion_fb.steps_executed;
    while ((xEventGroupGetBits(main_event_group) & BIT2) == 0) {
        if (motion_get_feedback(&motion_fb) > 0) {
            feedback_msg.feedback.time_elapsed = motion_fb.time_elapsed;
            rclc_action_publish_feedback(goal_handle, &feedback_msg);
            vTaskDelay(pdMS_TO_TICKS(200));
        }
    }
    xEventGroupClearBits(main_event_group, BIT2);
    
    rcl_action_goal_state_t goal_state = GOAL_STATE_SUCCEEDED;
    response_t response = {0};
    response.result.error_code = 0;
    rcl_ret_t rc = RCLC_RET_ACTION_WAIT_RESULT_REQUEST;
    while (rc != RCL_RET_OK) {
      rc = rclc_action_send_result(goal_handle, goal_state, &response);
    };
    vTaskDelete(NULL);
}

rcl_ret_t handle_goal(rclc_action_goal_handle_t * goal_handle, void * context)
{
    goal_req_t* req = (goal_req_t*)goal_handle->ros_goal_request;
    (void) req;
    ESP_LOGI(TAG, "Goal accepted");
    xTaskCreate(process_goal, "process_goal", 4096, goal_handle, 5, NULL);
    
    return RCL_RET_ACTION_GOAL_ACCEPTED;
}

bool handle_cancel(rclc_action_goal_handle_t * goal_handle, void * context)
{
  (void) context;
  (void) goal_handle;

  return true;
}

/* ============================ TASKS ============================*/

void motion_task(void* arg)
{
    ESP_LOGI(TAG, "Configuring motion axis");

    const int pulse_pins[] = {26, 14, 23, 33, 18, 21};
    const int dir_pins[] = {27, 13, 4, 25, 19, 22};
    motion_system_init(pulse_pins, dir_pins);

    ESP_LOGI(TAG, "Motion setup successfully. Starting transmission loop.");
    for (;;) {
        // Wait for the event to start execution
        xEventGroupWaitBits(main_event_group, BIT1, pdFALSE, pdFALSE, portMAX_DELAY);

        ESP_LOGI(TAG, "Executing motion.");
        motion_execute();
        ESP_LOGI(TAG, "Execution done.");
        xEventGroupClearBits(main_event_group, BIT1);
        xEventGroupSetBits(main_event_group, BIT2);
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

    // Create action server 
    rclc_action_server_t action_server;
    RCCHECK(rclc_action_server_init_default(
        &action_server, 
        &node, 
        &support, 
        ROSIDL_GET_ACTION_TYPE_SUPPORT(stepper_msgs, FollowStepperTrajectory), 
        ACTION_NAME
    ));

    // Create executor
    rclc_executor_t executor = rclc_executor_get_zero_initialized_executor();
    RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));

    // Add action server to executor
    goal_req_t goal_req;
    goal_req.goal.trajectory.points.capacity = MOTION_BUFFER_SIZE;
    goal_req.goal.trajectory.points.size = 0;
    goal_req.goal.trajectory.points.data = malloc(MOTION_BUFFER_SIZE*sizeof(stepper_msgs__msg__StepperTrajectoryPoint));
    for (size_t i = 0; i < MOTION_BUFFER_SIZE; ++i) {
        goal_req.goal.trajectory.points.data[i].accelerations.capacity = MOTION_BUFFER_SIZE;
        goal_req.goal.trajectory.points.data[i].accelerations.size = 0;
        goal_req.goal.trajectory.points.data[i].accelerations.data = malloc(MOTION_AXIS_NUM*sizeof(int64_t));
        goal_req.goal.trajectory.points.data[i].velocities.capacity = MOTION_BUFFER_SIZE;
        goal_req.goal.trajectory.points.data[i].velocities.size = 0;
        goal_req.goal.trajectory.points.data[i].velocities.data = malloc(MOTION_AXIS_NUM*sizeof(int64_t));
        goal_req.goal.trajectory.points.data[i].positions.capacity = MOTION_BUFFER_SIZE;
        goal_req.goal.trajectory.points.data[i].positions.size = 0;
        goal_req.goal.trajectory.points.data[i].positions.data = malloc(MOTION_AXIS_NUM*sizeof(int64_t));
    }
    
    RCCHECK(rclc_executor_add_action_server(
        &executor, 
        &action_server, 
        1,
        &goal_req,
        sizeof(goal_req),
        handle_goal,
        handle_cancel,
        NULL
    ));

    unsigned int rcl_wait_timeout = 1000;
    RCCHECK(rclc_executor_set_timeout(&executor, RCL_MS_TO_NS(rcl_wait_timeout)));

    // Spin forever.
    ESP_LOGI(TAG,"Spinning micro-ros executor...");
	while(1){
		rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
		usleep(100*1000);
	}
    // Free resources.
    RCCHECK(rcl_node_fini(&node));
    vTaskDelete(NULL);
}

/* ============================ APP MAIN ============================*/
void app_main()
{
    esp_log_level_set("*", ESP_LOG_ERROR);
    network_init();
    main_event_group = xEventGroupCreate();
    xTaskCreate(micro_ros_task, "uros_task", 4096, NULL, 5, NULL);
    xTaskCreate(motion_task, "motion_task", 4096, NULL, 5, NULL);
}
