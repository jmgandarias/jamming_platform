#ifndef JAMMING_PLATFORM_FROCE_DISPLACEMENT_EXPERIMENT_H_
#define JAMMING_PLATFORM_FROCE_DISPLACEMENT_EXPERIMENT_H_

#include <dynamixel_ros2.h>

#include "geometry_msgs/msg/wrench_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/fluid_pressure.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

#include <chrono>
#include <cerrno>
#include <cmath>
#include <ctime>
#include <fstream>
#include <functional>
#include <iomanip>
#include <sstream>
#include <string>
#include <sys/stat.h>
#include <sys/types.h>
#include <thread>

constexpr double kScrewMmPerRevolution = 1.25;
constexpr double kDegreesPerRevolution = 360.0;

inline double deg_to_mm(double position_deg)
{
    return position_deg * (kScrewMmPerRevolution / kDegreesPerRevolution);
}

inline double mm_to_deg(double position_mm)
{
    return position_mm * (kDegreesPerRevolution / kScrewMmPerRevolution);
}

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

        // Motion/force limits for safety during the experiment (linear axis in mm).
        this->declare_parameter("initial_position_mm", deg_to_mm(180.0));
        this->declare_parameter("position_tolerance_mm", deg_to_mm(2.0));
        this->declare_parameter("max_displacement_mm", deg_to_mm(90.0));
        this->declare_parameter("goal_velocity_rpm", -10.0);
        this->declare_parameter("max_force_n", 40.0);

        // Target pressure sweep (kPa) and repetitions per pressure level.
        this->declare_parameter("pressure_start_kpa", 0.0);
        this->declare_parameter("pressure_end_kpa", -80.0);
        this->declare_parameter("pressure_step_kpa", -10.0);
        this->declare_parameter("pressure_tolerance_kpa", 2.0);
        this->declare_parameter("repetitions_per_pressure", 10);

        // Output folder for CSV experiment logs.
        this->declare_parameter("results_directory", std::string("experiemnt_results"));

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

        initialize_csv_output();
    }

    ~ForceDisplacementExperimentNode()
    {
        if (csv_file_.is_open())
        {
            csv_file_.flush();
            csv_file_.close();
        }
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

    void publish_current_position(double position_mm)
    {
        std_msgs::msg::Float64 msg;
        msg.data = position_mm;
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
        int n_exp,
        int rep,
        double goal_pressure_kpa,
        double elapsed_s,
        double pressure_kpa,
        double position_mm,
        double force_n)
    {
        std_msgs::msg::Float64MultiArray msg;
        msg.data = {elapsed_s, pressure_kpa, position_mm, force_n};
        experiment_data_publisher_->publish(msg);

        log_experiment_sample_csv(
            n_exp,
            rep,
            goal_pressure_kpa,
            pressure_kpa,
            position_mm,
            force_n,
            elapsed_s);
    }

private:
    static bool ensure_directory_exists(const std::string &dir_path)
    {
        if (dir_path.empty())
        {
            return false;
        }

        std::string current_path;
        if (!dir_path.empty() && dir_path.front() == '/')
        {
            current_path = "/";
        }

        std::stringstream ss(dir_path);
        std::string segment;
        while (std::getline(ss, segment, '/'))
        {
            if (segment.empty())
            {
                continue;
            }

            if (!current_path.empty() && current_path.back() != '/')
            {
                current_path += "/";
            }
            current_path += segment;

            struct stat info
            {
            };
            if (stat(current_path.c_str(), &info) == 0)
            {
                if (!S_ISDIR(info.st_mode))
                {
                    return false;
                }
                continue;
            }

            if (mkdir(current_path.c_str(), 0755) != 0 && errno != EEXIST)
            {
                return false;
            }
        }

        return true;
    }

    void initialize_csv_output()
    {
        const std::string results_directory = get_param_string("results_directory");
        if (!ensure_directory_exists(results_directory))
        {
            RCLCPP_WARN(this->get_logger(), "Could not create results directory: %s", results_directory.c_str());
            return;
        }

        const auto now = std::chrono::system_clock::now();
        const std::time_t now_time = std::chrono::system_clock::to_time_t(now);
        std::tm local_tm;
        localtime_r(&now_time, &local_tm);

        std::ostringstream output_path;
        output_path << results_directory << "/force_displacement_" << std::put_time(&local_tm, "%Y%m%d_%H%M%S") << ".csv";
        csv_output_path_ = output_path.str();

        csv_file_.open(csv_output_path_, std::ios::out);
        if (!csv_file_.is_open())
        {
            RCLCPP_WARN(this->get_logger(), "Could not open CSV file for writing: %s", csv_output_path_.c_str());
            return;
        }

        csv_file_ << "n_exp;rep;goal_pressure;current_pressure;position;force;time\n";
        csv_enabled_ = true;
        RCLCPP_INFO(this->get_logger(), "Experiment CSV output enabled: %s", csv_output_path_.c_str());
    }

    void log_experiment_sample_csv(
        int n_exp,
        int rep,
        double goal_pressure_kpa,
        double current_pressure_kpa,
        double position_deg,
        double force_n,
        double elapsed_s)
    {
        if (!csv_enabled_)
        {
            return;
        }

        csv_file_ << n_exp << ';'
                  << rep << ';'
                  << goal_pressure_kpa << ';'
                  << current_pressure_kpa << ';'
                  << position_deg << ';'
                  << force_n << ';'
                  << elapsed_s << '\n';

        if (!csv_file_ && !csv_write_error_logged_)
        {
            RCLCPP_WARN(this->get_logger(), "Failed writing experiment sample to CSV: %s", csv_output_path_.c_str());
            csv_write_error_logged_ = true;
        }
    }

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

    std::ofstream csv_file_;
    std::string csv_output_path_;
    bool csv_enabled_{false};
    bool csv_write_error_logged_{false};

    rclcpp::Subscription<sensor_msgs::msg::FluidPressure>::SharedPtr pressure_subscription_;
    rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr wrench_subscription_;
};

// Active wait until a target position is reached, publishing position meanwhile.
inline bool wait_until_position(
    const std::shared_ptr<ForceDisplacementExperimentNode> &node,
    dynamixelMotor &motor,
    double target_mm,
    double tolerance_mm,
    double timeout_s)
{
    const auto t0 = node->now();
    rclcpp::Rate rate(node->get_loop_rate());
    while (rclcpp::ok())
    {
        rclcpp::spin_some(node);
        const double current_pos_deg = motor.getPresentPosition();
        const double current_pos_mm = deg_to_mm(current_pos_deg);
        node->publish_current_position(current_pos_mm);

        if (std::fabs(current_pos_mm - target_mm) <= tolerance_mm)
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
