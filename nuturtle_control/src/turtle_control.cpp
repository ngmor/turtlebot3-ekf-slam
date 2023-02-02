#include <stdexcept>
#include "turtlelib/diff_drive.hpp"
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

using turtlelib::DiffDrive;

/// \brief Enables control of the turtlebot
class TurtleControl : public rclcpp::Node
{
public:
  /// \brief initialize the node
  TurtleControl()
  : Node ("turtle_control")
  {
    //Parameters
    auto param = rcl_interfaces::msg::ParameterDescriptor{};
    
    param.description = "The wheel track in meters (REQUIRED)";
    declare_parameter("wheel_track", 0.0, param);
    double wheel_track = get_parameter("wheel_track").get_parameter_value().get<double>();

    param.description = "The wheel radius in meters (REQUIRED)";
    declare_parameter("wheel_radius", 0.0, param);
    double wheel_radius = get_parameter("wheel_radius").get_parameter_value().get<double>();

    bool required_parameters_received = true;

    if (wheel_track <= 0) {
      RCLCPP_ERROR_STREAM(get_logger(), "Invalid wheel track provided: " << wheel_track);
      required_parameters_received = false;
    }

    if (wheel_radius <= 0) {
      RCLCPP_ERROR_STREAM(get_logger(), "Invalid wheel radius provided: " << wheel_radius);
      required_parameters_received = false;
    }

    if (!required_parameters_received) {
      throw std::logic_error(
        "Required parameters were not received or were invalid. Please provide valid parameters."
      );
    }

    //Initialize turtlebot with input parameters
    turtlebot_ = DiffDrive {wheel_track, wheel_radius};

    RCLCPP_INFO_STREAM(get_logger(), "turtle_control node started");
  }
private:
  DiffDrive turtlebot_ {0.16, 0.033}; //Default values, to be overwritten in constructor
};

/// \brief Run the node 
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TurtleControl>());
  rclcpp::shutdown();
  return 0;
}