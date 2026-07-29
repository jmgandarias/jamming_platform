#include <dynamixel_ros2.h>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/wrench_stamped.hpp"
#include "std_msgs/msg/float64.hpp"



class MotorController : public rclcpp::Node
{
public:
  MotorController() : Node("motor_controller")
  {
    this->declare_parameter("loop_rate", 100.0);

    wrench_subscription_ = this->create_subscription<geometry_msgs::msg::WrenchStamped>(
      "sensor_wrench", 10,
      std::bind(&MotorController::wrench_callback, this, std::placeholders::_1));
    
  }

  double get_loop_rate()
  {
    return this->get_parameter("loop_rate").as_double();
  }

  geometry_msgs::msg::WrenchStamped get_last_wrench()
  {
    return last_wrench_;
  }

  std_msgs::msg::Float64 get_last_reference()
  {
    return last_reference_;
  }

private:
  void wrench_callback(const geometry_msgs::msg::WrenchStamped &msg)
  {
    last_wrench_ = msg;
    
    RCLCPP_INFO(this->get_logger(),
    "Force: [%.3f, %.3f, %.3f]  Torque: [%.3f, %.3f, %.3f]",
    msg.wrench.force.x,
    msg.wrench.force.y,
    msg.wrench.force.z,
    msg.wrench.torque.x,
    msg.wrench.torque.y,
    msg.wrench.torque.z);
  }

  geometry_msgs::msg::WrenchStamped last_wrench_;
  rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr wrench_subscription_;
};


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<MotorController>();

  dynamixelMotor motor("/dev/ttyUSB0", 1);

  // initialize communication (port, protocol version, baudrate)
  motor.iniComm("/dev/ttyUSB0", 2, 57600);

  motor.setControlTable();

  motor.setOperatingMode(dynamixelMotor::VELOCITY_CONTROL_MODE); 


  geometry_msgs::msg::WrenchStamped wrench;
  std_msgs::msg::Float64 force_reference;

  motor.setTorqueState(true);

  rclcpp::Rate rate(node->get_loop_rate());

  while (rclcpp::ok())
  {
    rclcpp::spin_some(node);


    // Set Goal Pressure


    // Set Goal Velocity
    motor.setGoalVelocity(action);

    // If pos and force aren't within tolerance, stop the motor

    // else, continue moving the motor
    // Get Present position, force, and actual pressure
      pos = motor.getPresentPosition();
      wrench = node->get_last_wrench();

      // Publish Present Position and Force




    rate.sleep();
  }
  
  motor.setTorqueState(false);
  rclcpp::shutdown();
  return 0;
}