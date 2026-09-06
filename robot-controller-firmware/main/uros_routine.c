#include "uros_routine.h"

// Timeout for each ping attempt
const int timeout_ms = 100;

// Number of ping attempts
const uint8_t attempts = 1;

// Spin period
const unsigned int spin_timeout = RCL_MS_TO_NS(10);

// Enum with connection status
enum states {
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

void command_callback(const void * msgin)
{
	const stepper_msgs__msg__StepperCommand * msg = (const stepper_msgs__msg__StepperCommand *)msgin;
}

void timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{
	(void) last_call_time;
	if (timer != NULL) {
		RCSOFTCHECK(rcl_publish(&state_publisher, &stepper_state, NULL));
	}
}

bool create_entities() {
    allocator = rcl_get_default_allocator();

    rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
    RCCHECK(rcl_init_options_init(&init_options, allocator));
    RCCHECK(rcl_init_options_set_domain_id(&init_options, 0));

    // Setup rmw options and ping agent
    rmw_init_options_t* rmw_options = rcl_init_options_get_rmw_init_options(&init_options);
    RCCHECK(rmw_uros_options_set_udp_address(CONFIG_MICRO_ROS_AGENT_IP, CONFIG_MICRO_ROS_AGENT_PORT, rmw_options));

    // Create init_options.
    RCCHECK(rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator));

    // Create node.
    node = rcl_get_zero_initialized_node();
    rcl_node_options_t node_ops = rcl_node_get_default_options();
    ESP_LOGI("micro_ros", "Creating node: %s", node_name);
    RCCHECK(rclc_node_init_with_options(&node, node_name, "", &support, &node_ops));
	
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
    
    // Allocate memory for stepper messages
    stepper_command.position.capacity = AXIS_NUM;
    stepper_command.position.size = 0;
    stepper_command.position.data = (int32_t*)malloc(AXIS_NUM * sizeof(int32_t));
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
	executor = rclc_executor_get_zero_initialized_executor();
	RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));

	// Add timer and subscriber to executor.
	RCCHECK(rclc_executor_add_timer(&executor, &timer));
	RCCHECK(rclc_executor_add_subscription(&executor, &command_subscriber, &stepper_command, &command_callback, ON_NEW_DATA));

    return true;
}

bool destroy_entities() {
    RCCHECK(rcl_subscription_fini(&command_subscriber, &node));
	RCCHECK(rcl_publisher_fini(&state_publisher, &node));
	RCCHECK(rcl_node_fini(&node));
    return true;
}

void micro_ros_task(void * arg)
{
    ESP_LOGI("micro_ros", "Starting task");
    state = WAITING_AGENT;

    while (true)
    {
        switch (state)
        {
            case WAITING_AGENT:
                // Check for agent connection
                state = (RMW_RET_OK == rmw_uros_ping_agent(timeout_ms, attempts)) ? AGENT_AVAILABLE : WAITING_AGENT;
                break;

            case AGENT_AVAILABLE:
                // Create micro-ROS entities
                state = (true == create_entities()) ? AGENT_CONNECTED : WAITING_AGENT;

                if (state == WAITING_AGENT)
                {
                    // Creation failed, release allocated resources
                    destroy_entities();
                };
                break;

            case AGENT_CONNECTED:
                // Check connection and spin on success
                state = (RMW_RET_OK == rmw_uros_ping_agent(timeout_ms, attempts)) ? AGENT_CONNECTED : AGENT_DISCONNECTED;
                if (state == AGENT_CONNECTED)
                {
                    rclc_executor_spin_some(&executor, spin_timeout);
                }
                break;

            case AGENT_DISCONNECTED:
                // Connection is lost, destroy entities and go back to first step
                destroy_entities();
                state = WAITING_AGENT;
                break;

            default:
                break;
        }
    }
}