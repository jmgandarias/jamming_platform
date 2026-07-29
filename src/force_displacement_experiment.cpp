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
    this->declare_parameter("K", 10.0);

    wrench_subscription_ = this->create_subscription<geometry_msgs::msg::WrenchStamped>(
      "sensor_wrench", 10,
      std::bind(&MotorController::wrench_callback, this, std::placeholders::_1));
    
    reference_subscription_ = this->create_subscription<std_msgs::msg::Float64>(
      "force_reference", 10,
      std::bind(&MotorController::reference_callback, this, std::placeholders::_1)
    );
  }

  double get_loop_rate()
  {
    return this->get_parameter("loop_rate").as_double();
  }

  double get_K()
  {
    return this->get_parameter("K").as_double();
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

  void reference_callback(const std_msgs::msg::Float64 &msg)
  {
    last_reference_ = msg;
  }

  geometry_msgs::msg::WrenchStamped last_wrench_;
  rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr wrench_subscription_;
  std_msgs::msg::Float64 last_reference_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr reference_subscription_;
};


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<MotorController>();

  dynamixelMotor motor("/dev/ttyUSB0", 1);

  // initialize communication (port, protocol version, baudrate)
  motor.iniComm("/dev/ttyUSB0", 2, 57600);

  motor.setControlTable();

  //motor.setOperatingMode(dynamixelMotor::POSITION_CONTROL_MODE); 
  //motor.setOperatingMode(dynamixelMotor::VELOCITY_CONTROL_MODE); 
  //motor.setOperatingMode(dynamixelMotor::CURRENT_CONTROL_MODE); 
  motor.setOperatingMode(dynamixelMotor::PWM_CONTROL_MODE);

  //motor.showDriveModeConfig();

  // read status
  //double pos = motor.getPresentPosition();
  //double vel = motor.getPresentVelocity();
  //double cur = motor.getPresentCurrent();

  double K = node->get_K();

  geometry_msgs::msg::WrenchStamped wrench;
  std_msgs::msg::Float64 force_reference;
  double force_error = 0;
  double action = 0;

  //motor.setGoalPWM(100);
  motor.setTorqueState(true);

  rclcpp::Rate rate(node->get_loop_rate());

  while (rclcpp::ok())
  {
    rclcpp::spin_some(node);

    //pos = motor.getPresentPosition();

    wrench = node->get_last_wrench();


    //motor.setGoalPosition(-wrench.wrench.force.z);
    //motor.setGoalVelocity(-wrench.wrench.force.z);
    //motor.setGoalCurrent((int)-wrench.wrench.force.z*4);
    //motor.setGoalPWM(-wrench.wrench.force.z*10);

    ///////////////////
    // FORCE CONTROL //
    ///////////////////

    // Get Reference
    force_reference = node->get_last_reference();

    // Calculate Error
    force_error = force_reference.data - wrench.wrench.force.z;

    // Controller (P/PI/PID)
    action = force_error*K;

    // Action
    motor.setGoalPWM((int)action);

    rate.sleep();
  }
  
  motor.setTorqueState(false);
  rclcpp::shutdown();
  return 0;
}