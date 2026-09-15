#include "uros_routine.h"

#include <rosidl_runtime_c/primitives_sequence_functions.h>
// Timeout for each ping attempt
const int timeout_ms = 100;

// Number of ping attempts
const uint8_t attempts = 3;

// Spin period
const unsigned int spin_timeout = RCL_MS_TO_NS(100);

// Enum with connection status
enum states
{
    WAITING_AGENT,
    AGENT_AVAILABLE,
    AGENT_CONNECTED,
    AGENT_DISCONNECTED
} state;

rcl_allocator_t allocator;
rclc_support_t support;
rcl_node_t node;
rclc_executor_t executor;
rcl_publisher_t state_publisher;
rcl_subscription_t command_subscriber;
rcl_timer_t timer;
rcl_init_options_t init_options;

extern stepper_motor_handle_t motor_handle[AXIS_NUM];
extern int target_positions[AXIS_NUM];

void command_callback(const void *msgin)
{
    const stepper_msgs__msg__StepperCommand *msg = (const stepper_msgs__msg__StepperCommand *)msgin;
    for (int i = 0; i < AXIS_NUM; ++i)
    {
        target_positions[i] = msg->position.data[i];
    }
}

void timer_callback(rcl_timer_t *timer, int64_t last_call_time)
{
    (void)last_call_time;
    if (timer != NULL)
    {
        for (int i = 0; i < AXIS_NUM; ++i)
        {
            stepper_state.position.data[i] = stepper_motor_get_position(motor_handle[i]);
            stepper_state.velocity.data[i] = stepper_motor_get_speed(motor_handle[i]);
        }
        RCSOFTCHECK(rcl_publish(&state_publisher, &stepper_state, NULL));
    }
}

bool init_rmw()
{
    allocator = rcl_get_default_allocator();

    init_options = rcl_get_zero_initialized_init_options();
    RCCHECK(rcl_init_options_init(&init_options, allocator));
    RCCHECK(rcl_init_options_set_domain_id(&init_options, 0));

    // Setup rmw options
    rmw_init_options_t *rmw_options = rcl_init_options_get_rmw_init_options(&init_options);
    RCCHECK(rmw_uros_options_set_udp_address(CONFIG_MICRO_ROS_AGENT_IP, CONFIG_MICRO_ROS_AGENT_PORT, rmw_options));

    // Initialize support
    RCCHECK(rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator));
    return true;
}

bool create_entities()
{

    // Create node.
    node = rcl_get_zero_initialized_node();
    rcl_node_options_t node_ops = rcl_node_get_default_options();
    ESP_LOGI("micro_ros", "Creating node: %s", node_name);
    RCCHECK(rclc_node_init_with_options(&node, node_name, "", &support, &node_ops));

    // Allocate memory for stepper messages
    rosidl_runtime_c__int32__Sequence__init(&stepper_command.position, AXIS_NUM);
    rosidl_runtime_c__int32__Sequence__init(&stepper_state.position, AXIS_NUM);
    rosidl_runtime_c__int32__Sequence__init(&stepper_state.velocity, AXIS_NUM);
    stepper_command.position.size = AXIS_NUM;
    stepper_state.position.size = AXIS_NUM;
    stepper_state.velocity.size = AXIS_NUM;

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

    // Create timer.
    timer = rcl_get_zero_initialized_timer();
    RCCHECK(rclc_timer_init_default2(
        &timer,
        &support,
        spin_timeout,
        timer_callback,
        true));

    // Create executor.
    executor = rclc_executor_get_zero_initialized_executor();
    RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));

    // Add timer and subscriber to executor.
    RCCHECK(rclc_executor_add_timer(&executor, &timer));
    RCCHECK(rclc_executor_add_subscription(&executor, &command_subscriber, &stepper_command, &command_callback, ON_NEW_DATA));

    return true;
}

bool destroy_entities()
{
    // Remove entities from executor first
    RCSOFTCHECK(rclc_executor_remove_subscription(&executor, &command_subscriber));
    RCSOFTCHECK(rclc_executor_remove_timer(&executor, &timer));

    // Finalize executor
    RCSOFTCHECK(rclc_executor_fini(&executor));

    // Now safe to finalize individual entities
    RCSOFTCHECK(rcl_subscription_fini(&command_subscriber, &node));
    RCSOFTCHECK(rcl_publisher_fini(&state_publisher, &node));
    RCSOFTCHECK(rcl_timer_fini(&timer));
    RCSOFTCHECK(rcl_node_fini(&node));

    // Clean up sequences
    rosidl_runtime_c__int32__Sequence__fini(&stepper_command.position);
    rosidl_runtime_c__int32__Sequence__fini(&stepper_state.position);
    rosidl_runtime_c__int32__Sequence__fini(&stepper_state.velocity);

    // Clean up support context - THIS IS CRITICAL
    RCSOFTCHECK(rclc_support_fini(&support));

    return true;
}

void micro_ros_task(void *arg)
{
    ESP_LOGI("micro_ros", "Starting task");
    state = WAITING_AGENT;
    init_rmw();
    TickType_t ticks;
    while (true)
    {
        switch (state)
        {
        case WAITING_AGENT:
            // Check for agent connection
            state = (RMW_RET_OK == rmw_uros_ping_agent(timeout_ms, attempts)) ? AGENT_AVAILABLE : WAITING_AGENT;
            if (state == WAITING_AGENT)
            {
                break;
            }
            ESP_LOGI("micro-ros", "Agent Found!");
            break;

        case AGENT_AVAILABLE:
            // Create micro-ROS entities
            state = (true == create_entities()) ? AGENT_CONNECTED : WAITING_AGENT;

            if (state == WAITING_AGENT)
            {
                ESP_LOGW("micro-ros", "Failed to create entities.");
                destroy_entities();
                // Re-initialize rmw for next attempt
                init_rmw();
                break;
            };
            ESP_LOGI("micro-ros", "Connected to agent.");
            ticks = xTaskGetTickCount();
            break;

        case AGENT_CONNECTED:
            // Check connection and spin on success
            state = (RMW_RET_OK == rmw_uros_ping_agent(timeout_ms, attempts)) ? AGENT_CONNECTED : AGENT_DISCONNECTED;
            if (state == AGENT_CONNECTED)
            {
                rclc_executor_spin_some(&executor, spin_timeout);
                xTaskDelayUntil(&ticks, spin_timeout);
                break;
            }
            break;

        case AGENT_DISCONNECTED:
            // Connection is lost, destroy entities and go back to first step
            ESP_LOGW("micro-ros", "Disconnected from agent.");
            destroy_entities();
            // Re-initialize rmw for reconnection
            init_rmw();
            state = WAITING_AGENT;
            break;

        default:
            break;
        }
    }
}