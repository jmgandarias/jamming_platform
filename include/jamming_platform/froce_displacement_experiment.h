#ifndef JAMMING_PLATFORM_FROCE_DISPLACEMENT_EXPERIMENT_H_
#define JAMMING_PLATFORM_FROCE_DISPLACEMENT_EXPERIMENT_H_

#include <dynamixel_ros2.h>

#include "geometry_msgs/msg/wrench_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/fluid_pressure.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

#include <chrono>
#include <cmath>
#include <functional>
#include <string>
#include <thread>

class ForceDisplacementExperimentNode : public rclcpp::Node
{
public:
    // ROS2 node that centralizes experiment I/O:
    // - Reads measured pressure and force.
    // - Publishes current position and experiment samples.
    ForceDisplacementExperimentNode()
        : Node("force_displacement_experiment")
    {
        // General parameters and Dynamixel communication settings.
        this->declare_parameter("loop_rate", 100.0);
        this->declare_parameter("port_name", std::string("/dev/ttyUSB0"));
        this->declare_parameter("protocol_version", 2.0);
        this->declare_parameter("baudrate", 57600);
        this->declare_parameter("motor_id", 1);

        // Motion/force limits for safety during the experiment.
        this->declare_parameter("initial_position_deg", 180.0);
        this->declare_parameter("position_tolerance_deg", 2.0);
        this->declare_parameter("max_displacement_deg", 90.0);
        this->declare_parameter("goal_velocity_rpm", -10.0);
        this->declare_parameter("max_force_n", 40.0);

        // Target pressure sweep (kPa) and repetitions per pressure level.
        this->declare_parameter("pressure_start_kpa", 0.0);
        this->declare_parameter("pressure_end_kpa", -80.0);
        this->declare_parameter("pressure_step_kpa", -10.0);
        this->declare_parameter("pressure_tolerance_kpa", 2.0);
        this->declare_parameter("repetitions_per_pressure", 10);

        // Settling times and timeouts to avoid blocking conditions.
        this->declare_parameter("settle_time_s", 2.0);
        this->declare_parameter("pressure_settle_timeout_s", 5.0);
        this->declare_parameter("move_timeout_s", 20.0);

        // Input sensors: force and pressure.
        wrench_subscription_ = this->create_subscription<geometry_msgs::msg::WrenchStamped>(
            "sensor_wrench", 10,
            std::bind(&ForceDisplacementExperimentNode::wrench_callback, this, std::placeholders::_1));

        pressure_subscription_ = this->create_subscription<sensor_msgs::msg::FluidPressure>(
            "current_pressure", 10,
            std::bind(&ForceDisplacementExperimentNode::pressure_callback, this, std::placeholders::_1));

        // Outputs: measured position, pressure setpoint, and compact experiment sample.
        current_position_publisher_ = this->create_publisher<std_msgs::msg::Float64>("current_position", 10);
        goal_pressure_publisher_ = this->create_publisher<sensor_msgs::msg::FluidPressure>("goal_pressure", 10);
        experiment_data_publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("experiment_data", 10);
    }

    double get_loop_rate()
    {
        return this->get_parameter("loop_rate").as_double();
    }

    geometry_msgs::msg::WrenchStamped get_last_wrench()
    {
        return last_wrench_;
    }

    sensor_msgs::msg::FluidPressure get_last_pressure()
    {
        return last_pressure_;
    }

    bool has_wrench() const
    {
        return has_wrench_;
    }

    bool has_pressure() const
    {
        return has_pressure_;
    }

    double get_param_double(const std::string &name)
    {
        return this->get_parameter(name).as_double();
    }

    int get_param_int(const std::string &name)
    {
        return this->get_parameter(name).as_int();
    }

    std::string get_param_string(const std::string &name)
    {
        return this->get_parameter(name).as_string();
    }

    void publish_current_position(double position_deg)
    {
        std_msgs::msg::Float64 msg;
        msg.data = position_deg;
        current_position_publisher_->publish(msg);
    }

    void publish_goal_pressure_kpa(double goal_pressure_kpa)
    {
        sensor_msgs::msg::FluidPressure msg;
        msg.fluid_pressure = goal_pressure_kpa * 1000.0;
        msg.variance = 0.0;
        goal_pressure_publisher_->publish(msg);
    }

    void publish_experiment_sample(
        double elapsed_s,
        double pressure_kpa,
        double position_deg,
        double force_n)
    {
        std_msgs::msg::Float64MultiArray msg;
        msg.data = {elapsed_s, pressure_kpa, position_deg, force_n};
        experiment_data_publisher_->publish(msg);
    }

private:
    // Stores the latest available force measurement.
    void wrench_callback(const geometry_msgs::msg::WrenchStamped &msg)
    {
        last_wrench_ = msg;
        has_wrench_ = true;
    }

    // Stores the latest available pressure measurement.
    void pressure_callback(const sensor_msgs::msg::FluidPressure &msg)
    {
        last_pressure_ = msg;
        has_pressure_ = true;
    }

    sensor_msgs::msg::FluidPressure last_pressure_;
    geometry_msgs::msg::WrenchStamped last_wrench_;
    bool has_pressure_{false};
    bool has_wrench_{false};

    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr current_position_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::FluidPressure>::SharedPtr goal_pressure_publisher_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr experiment_data_publisher_;

    rclcpp::Subscription<sensor_msgs::msg::FluidPressure>::SharedPtr pressure_subscription_;
    rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr wrench_subscription_;
};

// Active wait until a target position is reached, publishing position meanwhile.
inline bool wait_until_position(
    const std::shared_ptr<ForceDisplacementExperimentNode> &node,
    dynamixelMotor &motor,
    double target_deg,
    double tolerance_deg,
    double timeout_s)
{
    const auto t0 = node->now();
    rclcpp::Rate rate(node->get_loop_rate());
    while (rclcpp::ok())
    {
        rclcpp::spin_some(node);
        const double current_pos = motor.getPresentPosition();
        node->publish_current_position(current_pos);

        if (std::fabs(current_pos - target_deg) <= tolerance_deg)
        {
            return true;
        }

        if ((node->now() - t0).seconds() > timeout_s)
        {
            return false;
        }
        rate.sleep();
    }
    return false;
}

// Active wait until measured pressure approaches the target setpoint.
inline bool wait_until_pressure(
    const std::shared_ptr<ForceDisplacementExperimentNode> &node,
    double goal_pressure_kpa,
    double tolerance_kpa,
    double timeout_s)
{
    const auto t0 = node->now();
    rclcpp::Rate rate(node->get_loop_rate());
    while (rclcpp::ok())
    {
        rclcpp::spin_some(node);
        node->publish_goal_pressure_kpa(goal_pressure_kpa);

        if (node->has_pressure())
        {
            const double current_kpa = node->get_last_pressure().fluid_pressure / 1000.0;
            if (std::fabs(current_kpa - goal_pressure_kpa) <= tolerance_kpa)
            {
                return true;
            }
        }

        if ((node->now() - t0).seconds() > timeout_s)
        {
            return false;
        }

        rate.sleep();
    }
    return false;
}

#endif // JAMMING_PLATFORM_FROCE_DISPLACEMENT_EXPERIMENT_H_
