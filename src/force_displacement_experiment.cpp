/*
Author: Juan M. Gandarias
Date: 30/07/2026
Description: Automatic experiment to measure the force-displacement relationship of a jamming platform using a Dynamixel motor and ROS2.

Main behavior:
1. Subscribe to measured pressure on topic current_pressure (sensor_msgs/msg/FluidPressure).
2. Subscribe to measured force on topic sensor_wrench (geometry_msgs/msg/WrenchStamped).
3. Publish measured position on topic current_position (std_msgs/msg/Float64).
4. In every step of the loop:
  - Set the motor in position control mode and move to the initial position.
  - Set the motor in velocity control mode.
  - Set the goal pressure that ranges from 0 to -80 kPa every 10 kPa.
  - Set a contstant goal velocity for the Dynamixel motor.
  - while the force and position are within tolerance, keep moving the motor and reading the current pressure, force, and position. Publish in a float64 message the current time, pressure, position and force.
  - when the force and position are not within tolerance, stop the motor and wait for a few seconds before starting the next step.
  - move the motor back to the initial position and wait for a few seconds before starting the next step.
  - Repeat each pressure step 10 times.
5. The experiment ends when the pressure reaches -80 kPa.
*/

#include "jamming_platform/froce_displacement_experiment.h"

#include <vector>

int main(int argc, char *argv[])
{
  // 1) ROS2 initialization and experiment node creation.
  rclcpp::init(argc, argv);

  auto node = std::make_shared<ForceDisplacementExperimentNode>();

  // 2) Read configuration parameters (YAML + default values).
  const std::string port_name = node->get_param_string("port_name");
  const double protocol_version = node->get_param_double("protocol_version");
  const int baudrate = node->get_param_int("baudrate");
  const int motor_id = node->get_param_int("motor_id");

  const double initial_position_deg = node->get_param_double("initial_position_deg");
  const double position_tolerance_deg = node->get_param_double("position_tolerance_deg");
  const double max_displacement_deg = node->get_param_double("max_displacement_deg");
  const double goal_velocity_rpm = node->get_param_double("goal_velocity_rpm");
  const double max_force_n = node->get_param_double("max_force_n");

  const double pressure_start_kpa = node->get_param_double("pressure_start_kpa");
  const double pressure_end_kpa = node->get_param_double("pressure_end_kpa");
  const double pressure_step_kpa = node->get_param_double("pressure_step_kpa");
  const double pressure_tolerance_kpa = node->get_param_double("pressure_tolerance_kpa");
  const int repetitions_per_pressure = node->get_param_int("repetitions_per_pressure");

  const double settle_time_s = node->get_param_double("settle_time_s");
  const double pressure_settle_timeout_s = node->get_param_double("pressure_settle_timeout_s");
  const double move_timeout_s = node->get_param_double("move_timeout_s");

  // 3) Basic pressure sweep validations.
  if (pressure_step_kpa >= 0.0)
  {
    RCLCPP_ERROR(node->get_logger(), "Parameter pressure_step_kpa must be negative for descending pressure sweep.");
    rclcpp::shutdown();
    return 1;
  }

  if (pressure_end_kpa > pressure_start_kpa)
  {
    RCLCPP_ERROR(node->get_logger(), "pressure_end_kpa must be <= pressure_start_kpa.");
    rclcpp::shutdown();
    return 1;
  }

  dynamixelMotor motor("experiment_motor", motor_id);

  // iniComm requires char*; build a mutable buffer from std::string.
  std::vector<char> port_name_buffer(port_name.begin(), port_name.end());
  port_name_buffer.push_back('\0');

  // 4) Initialize communication and motor control table.
  if (!dynamixelMotor::iniComm(port_name_buffer.data(), static_cast<float>(protocol_version), baudrate))
  {
    RCLCPP_ERROR(node->get_logger(), "Failed to initialize Dynamixel communication.");
    rclcpp::shutdown();
    return 1;
  }

  motor.setControlTable();

  motor.setTorqueState(false);
  motor.setOperatingMode(dynamixelMotor::POSITION_CONTROL_MODE);
  motor.setTorqueState(true);

  // 5) Wait for valid sensor data before starting.
  RCLCPP_INFO(node->get_logger(), "Waiting for first pressure and wrench samples...");
  rclcpp::Rate pre_rate(node->get_loop_rate());
  while (rclcpp::ok() && (!node->has_pressure() || !node->has_wrench()))
  {
    rclcpp::spin_some(node);
    pre_rate.sleep();
  }

  if (!rclcpp::ok())
  {
    motor.setTorqueState(false);
    rclcpp::shutdown();
    return 0;
  }

  const auto experiment_t0 = node->now();

  // 6) Main loop: pressure sweep and repetitions at each level.
    int n_exp = 0;
  for (double goal_pressure_kpa = pressure_start_kpa;
       rclcpp::ok() && goal_pressure_kpa >= pressure_end_kpa;
       goal_pressure_kpa += pressure_step_kpa)
  {
    for (int rep = 0; rclcpp::ok() && rep < repetitions_per_pressure; ++rep)
    {
      RCLCPP_INFO(
          node->get_logger(),
          "Pressure %.1f kPa | repetition %d/%d",
          goal_pressure_kpa,
          rep + 1,
          repetitions_per_pressure);

      // 6.1) Move to initial position in position control mode.
      motor.setTorqueState(false);
      motor.setOperatingMode(dynamixelMotor::POSITION_CONTROL_MODE);
      motor.setTorqueState(true);
      motor.setGoalPosition(initial_position_deg);

      const bool reached_initial = wait_until_position(
          node,
          motor,
          initial_position_deg,
          position_tolerance_deg,
          move_timeout_s);

      if (!reached_initial)
      {
        RCLCPP_WARN(node->get_logger(), "Timeout while moving to initial position.");
      }

      std::this_thread::sleep_for(std::chrono::duration<double>(settle_time_s));

      // 6.2) Publish pressure setpoint and wait for settling.
      node->publish_goal_pressure_kpa(goal_pressure_kpa);
      (void)wait_until_pressure(
          node,
          goal_pressure_kpa,
          pressure_tolerance_kpa,
          pressure_settle_timeout_s);

      // 6.3) Switch to velocity control and execute movement.
      motor.setTorqueState(false);
      motor.setOperatingMode(dynamixelMotor::VELOCITY_CONTROL_MODE);
      motor.setTorqueState(true);
      motor.setGoalVelocity(goal_velocity_rpm);

      const auto move_t0 = node->now();
      rclcpp::Rate loop_rate(node->get_loop_rate());
      while (rclcpp::ok())
      {
        // 6.3.1) While within tolerances, publish experiment telemetry.
        rclcpp::spin_some(node);

        const double position_deg = motor.getPresentPosition();
        const double displacement_deg = std::fabs(position_deg - initial_position_deg);
        const double pressure_kpa = node->get_last_pressure().fluid_pressure / 1000.0;
        const double force_n = node->get_last_wrench().wrench.force.z;
        const double elapsed_s = (node->now() - experiment_t0).seconds();

        node->publish_current_position(position_deg);
        node->publish_experiment_sample(n_exp, rep + 1, goal_pressure_kpa, elapsed_s, pressure_kpa, position_deg, force_n);
        node->publish_goal_pressure_kpa(goal_pressure_kpa);

        const bool force_within_tolerance = std::fabs(force_n) <= max_force_n;
        const bool position_within_tolerance = displacement_deg <= max_displacement_deg;

        if (!force_within_tolerance || !position_within_tolerance)
        {
          break;
        }

        if ((node->now() - move_t0).seconds() > move_timeout_s)
        {
          RCLCPP_WARN(node->get_logger(), "Move timeout reached at pressure %.1f kPa.", goal_pressure_kpa);
          break;
        }

        loop_rate.sleep();
      }

      // 6.4) Stop and settle before returning to origin.
      motor.setGoalVelocity(0.0);
      std::this_thread::sleep_for(std::chrono::duration<double>(settle_time_s));

      // 6.5) Return to initial position for the next repetition.
      motor.setTorqueState(false);
      motor.setOperatingMode(dynamixelMotor::POSITION_CONTROL_MODE);
      motor.setTorqueState(true);
      motor.setGoalPosition(initial_position_deg);

      (void)wait_until_position(
          node,
          motor,
          initial_position_deg,
          position_tolerance_deg,
          move_timeout_s);

      std::this_thread::sleep_for(std::chrono::duration<double>(settle_time_s));
    }

    ++n_exp;
  }

  // 7) Safe shutdown.
  motor.setGoalVelocity(0.0);
  motor.setTorqueState(false);
  RCLCPP_INFO(node->get_logger(), "Force-displacement experiment finished.");
  rclcpp::shutdown();
  return 0;
}