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

    // init feedback
    feedback_t feedback;
    int64_t feedback_data[1];
    feedback.feedback.current_positions.capacity = MOTION_AXIS_NUM;
    feedback.feedback.current_positions.size = 1;
    feedback.feedback.current_positions.data = feedback_data;

    // loop over points
    for (size_t i = 0; i < req->goal.trajectory.points.size - 1; ++i) {
        point_t p_current = req->goal.trajectory.points.data[i];
        point_t p_next = req->goal.trajectory.points.data[i + 1];
        if (p_current.accelerations.size > 0
            && p_current.velocities.size > 0
            && p_current.positions.size  > 0) {
            const int64_t ts = p_next.time_from_start - p_current.time_from_start;
            const int64_t a0 = p_current.accelerations.data[0];
            const int64_t v0 = p_current.velocities.data[0];
            const int64_t x0 = p_current.positions.data[0];
            int64_t t = 0;
            int64_t x = x0;
            int64_t v = v0;
            while (t < ts) {
                int64_t vdiv;
                if (v > VMIN || v < -VMIN) {
                    vdiv = llabs(v);
                }
                else {
                    vdiv = VMIN;
                }
                int64_t dt = (1000*1000*1000)/vdiv;
                t += dt;
                v = (a0*t)/1000 + v0;
                x = (a0*t*t)/(2*1000*1000) + (v0*t)/1000 + x0;
            }
            feedback.feedback.time_elapsed = t+p_current.time_from_start;
            feedback_data[0] = x;
            rclc_action_publish_feedback(goal_handle, &feedback);
            vTaskDelay(pdMS_TO_TICKS(ts/1000));
        }
    }
    rcl_action_goal_state_t goal_state = GOAL_STATE_SUCCEEDED;
    response_t response = {0};
    response.result.error_code = 0;
    rcl_ret_t rc = RCLC_RET_ACTION_WAIT_RESULT_REQUEST;
    while (rc != RCL_RET_OK) {
      rc = rclc_action_send_result(goal_handle, goal_state, &response);
      vTaskDelay(pdMS_TO_TICKS(1000));
    };
    vTaskDelete(NULL);
}

rcl_ret_t handle_goal(rclc_action_goal_handle_t * goal_handle, void * context)
{
    goal_req_t* req = (goal_req_t*)goal_handle->ros_goal_request;
    ESP_LOGI(TAG, "Received goal");
    for (size_t i = 0; i < req->goal.trajectory.points.size; ++i) {
        point_t p = req->goal.trajectory.points.data[i];
        for (size_t j = 0; j < p.accelerations.size; ++j) {
            ESP_LOGI(TAG, "accel: %lld", p.accelerations.data[0]);
            ESP_LOGI(TAG, "vel: %lld", p.velocities.data[0]);
            ESP_LOGI(TAG, "pos: %lld", p.positions.data[0]);
        }
    }
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
        xEventGroupWaitBits(main_event_group, BIT1, pdTRUE, pdFALSE, portMAX_DELAY);

        ESP_LOGI(TAG, "Executing motion.");
        motion_execute();
        ESP_LOGI(TAG, "Execution done.");

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
    point_t points[10];
    int64_t accelerations[MOTION_AXIS_NUM][10];
    int64_t velocities[MOTION_AXIS_NUM][10];
    int64_t positions[MOTION_AXIS_NUM][10];
    for (size_t i = 0; i < 10; ++i) {
        points[i].accelerations.capacity = MOTION_AXIS_NUM;
        points[i].accelerations.data = accelerations[i];
        points[i].accelerations.size = 0;
        points[i].velocities.capacity = MOTION_AXIS_NUM;
        points[i].velocities.data = velocities[i];
        points[i].velocities.size = 0;
        points[i].positions.capacity = MOTION_AXIS_NUM;
        points[i].positions.data = positions[i];
        points[i].positions.size = 0;
        points[i].time_from_start = 0;
    }
    goal_req.goal.trajectory.points.capacity = 10;
    goal_req.goal.trajectory.points.size = 0;
    goal_req.goal.trajectory.points.data = points;
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
    esp_log_level_set("*", ESP_LOG_INFO);
    network_init();
    main_event_group = xEventGroupCreate();
    xTaskCreate(micro_ros_task, "uros_task", 8192, NULL, 5, NULL);
    xTaskCreate(motion_task, "motion_task", 4096, NULL, 5, NULL);
}
