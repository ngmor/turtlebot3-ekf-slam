#include <stdexcept>
#include <string>
#include "turtlelib/diff_drive.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

/// \brief Enables control of the turtlebot
class Odometry : public rclcpp::Node
{
public:
  /// \brief initialize the node
  Odometry()
  : Node ("odometry")
  {
    //TODO remove
    //temporary run command
    //ros2 run nuturtle_control odometry --ros-args -p body_id:=base_footprint -p wheel_left:=wheel_left_joint -p wheel_right:=wheel_right_joint

    //Parameters
    auto param = rcl_interfaces::msg::ParameterDescriptor{};
    
    //Check if required parameters were provided
    bool required_parameters_received = true;

    param.description = "The name of the robot's body frame (REQUIRED)";
    declare_parameter("body_id", "", param);
    body_id_ = get_parameter("body_id").get_parameter_value().get<std::string>();

    if (body_id_ == "") {
      RCLCPP_ERROR_STREAM(get_logger(), "No body frame provided.");
      required_parameters_received = false;
    }

    param.description = "The name of the robot's odometry frame";
    declare_parameter("odom_id", "odom", param);
    odom_id_ = get_parameter("odom_id").get_parameter_value().get<std::string>();

    param.description = "The name of the robot's left wheel joint (REQUIRED)";
    declare_parameter("wheel_left", "", param);
    wheel_left_ = get_parameter("wheel_left").get_parameter_value().get<std::string>();

    if (wheel_left_ == "") {
      RCLCPP_ERROR_STREAM(get_logger(), "No left wheel joint provided.");
      required_parameters_received = false;
    }

    param.description = "The name of the robot's right wheel joint (REQUIRED)";
    declare_parameter("wheel_right", "", param);
    wheel_right_ = get_parameter("wheel_right").get_parameter_value().get<std::string>();

    if (wheel_right_ == "") {
      RCLCPP_ERROR_STREAM(get_logger(), "No right wheel joint provided.");
      required_parameters_received = false;
    }

    //Abort if any required parameters were not provided
    if (!required_parameters_received) {
      throw std::logic_error(
        "Required parameters were not received or were invalid. Please provide valid parameters."
      );
    }


    //Publishers
    
    //Subscribers
    




    RCLCPP_INFO_STREAM(get_logger(), "odometry node started");
  }
private:
  
  std::string body_id_, odom_id_, wheel_left_, wheel_right_;
};

/// \brief Run the node 
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Odometry>());
  rclcpp::shutdown();
  return 0;
}