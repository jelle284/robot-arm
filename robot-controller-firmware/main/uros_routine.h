
#ifndef UROS_ROUTINE_H
#define UROS_ROUTINE_H
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <stepper_msgs/msg/stepper_state.h>
#include <stepper_msgs/msg/stepper_command.h>
#include <std_msgs/msg/int32.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Aborting.\n",__LINE__,(int)temp_rc);vTaskDelete(NULL);}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Continuing.\n",__LINE__,(int)temp_rc);}}

#define AXIS_NUM 6 // Number of stepper motors

static stepper_msgs__msg__StepperCommand stepper_command;
static stepper_msgs__msg__StepperState stepper_state;

static const char* node_name = "uros_esp32_robot_controller";

void micro_ros_task(void* arg);
#endif // UROS_ROUTINE_H