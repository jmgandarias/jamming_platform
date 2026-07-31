/*
Author: Juan M. Gandarias
Date: 30/07/2026
Description: Deterministic pressure control using micro-ROS and hardware timer trigger.

Main behavior:
1. Subscribe to goal pressure on topic goal_pressure (sensor_msgs/msg/FluidPressure).
2. Publish measured pressure on topic current_pressure (sensor_msgs/msg/FluidPressure).
3. Publish valve states on topics:
   - on_rigid_valve_state (std_msgs/msg/Bool) for pressurization valve (pin 12)
   - on_soft_valve_state (std_msgs/msg/Bool) for depressurization valve (pin 14)
4. Run hysteresis ON/OFF control every 10 ms (100 Hz).
5. Execute control from a periodic FreeRTOS task using vTaskDelayUntil.
*/

#include <micro_ros_arduino.h>

#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/adc.h"
#include "driver/gpio.h"

#include <std_msgs/msg/bool.h>
#include <sensor_msgs/msg/fluid_pressure.h>

// -------------------------
// Topic and node names
// -------------------------
constexpr const char *NODE_NAME = "micro_ros_arduino_node";
constexpr const char *NODE_NAMESPACE = "";
constexpr const char *TOPIC_GOAL_PRESSURE = "goal_pressure";
constexpr const char *TOPIC_CURRENT_PRESSURE = "current_pressure";
constexpr const char *TOPIC_ON_RIGID_VALVE_STATE = "on_rigid_valve_state";
constexpr const char *TOPIC_ON_SOFT_VALVE_STATE = "on_soft_valve_state";

// -------------------------
// Hardware configuration
// -------------------------
constexpr adc1_channel_t PRESSURE_ADC_CHANNEL = ADC1_CHANNEL_7; // GPIO35
constexpr adc_bits_width_t PRESSURE_ADC_WIDTH = ADC_WIDTH_BIT_12;
constexpr adc_atten_t PRESSURE_ADC_ATTEN = ADC_ATTEN_DB_11;

constexpr gpio_num_t SOFT_VALVE_PIN = GPIO_NUM_14;  // Depressurization valve
constexpr gpio_num_t RIGID_VALVE_PIN = GPIO_NUM_12; // Pressurization valve

constexpr uint8_t VALVE_ON = HIGH;
constexpr uint8_t VALVE_OFF = LOW;

// -------------------------
// Control and timing
// -------------------------
constexpr uint32_t CONTROL_TIMESTEP_MS = 10U; // 100 Hz

constexpr uint32_t EXECUTOR_SPIN_TIMEOUT_NS = 0U;
constexpr int64_t NS_PER_SEC = 1000000000LL;

constexpr uint32_t STARTUP_DELAY_MS = 2000U;
constexpr uint32_t SESSION_SYNC_TIMEOUT_MS = 1000U;

constexpr uint32_t CONTROL_TASK_STACK_WORDS = 4096U;
constexpr UBaseType_t CONTROL_TASK_PRIORITY = 2U;
constexpr int ROS_INIT_ARGC = 0;
constexpr char **ROS_INIT_ARGV = nullptr;
constexpr size_t EXECUTOR_HANDLES = 1U;

// Pressure sensor linear model: pressure = slope * adc + offset
constexpr double SENSOR_SLOPE = 0.00046115046610263106;
constexpr double SENSOR_OFFSET = -0.8958145172523699;

// Hysteresis settings
double GoalPressure = 0.0;   // Initial desired pressure
double PressureThres = 0.05; // Half-band around GoalPressure

// -------------------------
// ROS objects
// -------------------------
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rclc_executor_t executor;

rcl_subscription_t goal_pressure_subscriber;
rcl_publisher_t current_pressure_publisher;
rcl_publisher_t rigid_valve_state_publisher;
rcl_publisher_t soft_valve_state_publisher;

sensor_msgs__msg__FluidPressure pressure_msg;
sensor_msgs__msg__FluidPressure goal_pressure_msg;
std_msgs__msg__Bool rigid_valve_state_msg;
std_msgs__msg__Bool soft_valve_state_msg;

// -------------------------
// RTOS objects
// -------------------------
TaskHandle_t control_task_handle = nullptr;

// -------------------------
// Error handling macros
// -------------------------
#define RCCHECK(fn)            \
  {                            \
    rcl_ret_t temp_rc = fn;    \
    if (temp_rc != RCL_RET_OK) \
    {                          \
      error_loop();            \
    }                          \
  }
#define RCSOFTCHECK(fn)        \
  {                            \
    rcl_ret_t temp_rc = fn;    \
    if (temp_rc != RCL_RET_OK) \
    {                          \
    }                          \
  }

void error_loop()
{
  while (1)
  {
    // Halt execution on unrecoverable ROS errors.
  }
}

void goal_pressure_callback(const void *msgin)
{
  const sensor_msgs__msg__FluidPressure *msg =
      (const sensor_msgs__msg__FluidPressure *)msgin;
  GoalPressure = msg->fluid_pressure;
}

double read_current_pressure()
{
  const int adc_raw = adc1_get_raw(PRESSURE_ADC_CHANNEL);
  const double adc_value = static_cast<double>(adc_raw);
  return (SENSOR_SLOPE * adc_value) + SENSOR_OFFSET;
}

void update_pressure_message(const double current_pressure)
{
  const int64_t time_ns = rmw_uros_epoch_nanos();
  pressure_msg.header.stamp.sec = time_ns / NS_PER_SEC;
  pressure_msg.header.stamp.nanosec = time_ns % NS_PER_SEC;
  pressure_msg.fluid_pressure = current_pressure;
  pressure_msg.variance = 0.0;
}

void publish_valve_states(const bool rigid_on, const bool soft_on)
{
  rigid_valve_state_msg.data = rigid_on;
  soft_valve_state_msg.data = soft_on;

  RCSOFTCHECK(rcl_publish(
      &rigid_valve_state_publisher,
      &rigid_valve_state_msg,
      NULL));
  RCSOFTCHECK(rcl_publish(
      &soft_valve_state_publisher,
      &soft_valve_state_msg,
      NULL));
}

void TaskControl(void *argument)
{
  (void)argument;
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xPeriod = pdMS_TO_TICKS(CONTROL_TIMESTEP_MS);

  while (1)
  {
    // Refresh subscriber input so GoalPressure uses the newest command.
    RCSOFTCHECK(rclc_executor_spin_some(&executor, EXECUTOR_SPIN_TIMEOUT_NS));

    // Run control loop at a fixed 100 Hz period.
    const int previous_rigid_level = gpio_get_level(RIGID_VALVE_PIN);
    const int previous_soft_level = gpio_get_level(SOFT_VALVE_PIN);

    const double current_pressure = read_current_pressure();

    if (current_pressure <= (GoalPressure - PressureThres))
    {
      // Below lower threshold: pressurize.
      gpio_set_level(RIGID_VALVE_PIN, VALVE_ON);
      gpio_set_level(SOFT_VALVE_PIN, VALVE_OFF);
    }
    else if (current_pressure >= (GoalPressure + PressureThres))
    {
      // Above upper threshold: depressurize.
      gpio_set_level(SOFT_VALVE_PIN, VALVE_ON);
      gpio_set_level(RIGID_VALVE_PIN, VALVE_OFF);
    }
    // Inside the hysteresis band, keep previous states.

    update_pressure_message(current_pressure);
    RCSOFTCHECK(rcl_publish(&current_pressure_publisher, &pressure_msg, NULL));

    const int current_rigid_level = gpio_get_level(RIGID_VALVE_PIN);
    const int current_soft_level = gpio_get_level(SOFT_VALVE_PIN);
    if ((previous_rigid_level != current_rigid_level) || (previous_soft_level != current_soft_level))
    {
      publish_valve_states(
          current_rigid_level == VALVE_ON,
          current_soft_level == VALVE_ON);
    }

    vTaskDelayUntil(&xLastWakeTime, xPeriod);
  }
}

void setup()
{
  set_microros_transports();
  delay(STARTUP_DELAY_MS);

  allocator = rcl_get_default_allocator();
  RCCHECK(rclc_support_init(&support, ROS_INIT_ARGC, ROS_INIT_ARGV, &allocator));
  RCCHECK(rclc_node_init_default(&node, NODE_NAME, NODE_NAMESPACE, &support));

  adc1_config_width(PRESSURE_ADC_WIDTH);
  adc1_config_channel_atten(PRESSURE_ADC_CHANNEL, PRESSURE_ADC_ATTEN);

  gpio_set_direction(SOFT_VALVE_PIN, GPIO_MODE_OUTPUT);
  gpio_set_direction(RIGID_VALVE_PIN, GPIO_MODE_OUTPUT);

  gpio_set_level(SOFT_VALVE_PIN, VALVE_OFF);
  gpio_set_level(RIGID_VALVE_PIN, VALVE_OFF);

  RCCHECK(rclc_publisher_init_default(
      &current_pressure_publisher,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, FluidPressure),
      TOPIC_CURRENT_PRESSURE));

  RCCHECK(rclc_publisher_init_default(
      &rigid_valve_state_publisher,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
      TOPIC_ON_RIGID_VALVE_STATE));

  RCCHECK(rclc_publisher_init_default(
      &soft_valve_state_publisher,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
      TOPIC_ON_SOFT_VALVE_STATE));

  RCCHECK(rclc_subscription_init_default(
      &goal_pressure_subscriber,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, FluidPressure),
      TOPIC_GOAL_PRESSURE));

  RCCHECK(rclc_executor_init(&executor, &support.context, EXECUTOR_HANDLES, &allocator));
  RCCHECK(rclc_executor_add_subscription(
      &executor,
      &goal_pressure_subscriber,
      &goal_pressure_msg,
      &goal_pressure_callback,
      ON_NEW_DATA));

  sensor_msgs__msg__FluidPressure__init(&pressure_msg);
  sensor_msgs__msg__FluidPressure__init(&goal_pressure_msg);
  std_msgs__msg__Bool__init(&rigid_valve_state_msg);
  std_msgs__msg__Bool__init(&soft_valve_state_msg);

  RCCHECK(rmw_uros_sync_session(SESSION_SYNC_TIMEOUT_MS));

  // Deterministic control path: periodic FreeRTOS task at 100 Hz.
  xTaskCreate(
      TaskControl,
      "control_task",
      CONTROL_TASK_STACK_WORDS,
      NULL,
      CONTROL_TASK_PRIORITY,
      &control_task_handle);
}

void loop()
{
  // All application logic runs in TaskControl; stop Arduino loop task.
  vTaskDelete(NULL);
}
