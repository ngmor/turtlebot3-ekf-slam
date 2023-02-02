#include <stdexcept>
#include "turtlelib/diff_drive.hpp"
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nuturtlebot_msgs/msg/wheel_commands.hpp"

using turtlelib::DiffDrive;
using turtlelib::Wheel;
using turtlelib::Twist2D;
using turtlelib::PI;

constexpr double TICKS_PER_RAD = 4096.0 / (2.0*PI);

/// \brief Enables control of the turtlebot
class TurtleControl : public rclcpp::Node
{
public:
  /// \brief initialize the node
  TurtleControl()
  : Node ("turtle_control")
  {
    //TODO remove
    //temporary run command
    //ros2 run nuturtle_control turtle_control --ros-args --params-file install/nuturtle_description/share/nuturtle_description/config/diff_params.yaml

    //Parameters
    auto param = rcl_interfaces::msg::ParameterDescriptor{};
    
    param.description = "The wheel track width in meters (REQUIRED)";
    declare_parameter("track_width", 0.0, param);
    double wheel_track = get_parameter("track_width").get_parameter_value().get<double>();

    param.description = "The wheel radius in meters (REQUIRED)";
    declare_parameter("wheel_radius", 0.0, param);
    double wheel_radius = get_parameter("wheel_radius").get_parameter_value().get<double>();

    //Check if required parameters were provided
    bool required_parameters_received = true;

    if (wheel_track <= 0) {
      RCLCPP_ERROR_STREAM(get_logger(), "Invalid wheel track provided: " << wheel_track);
      required_parameters_received = false;
    }

    if (wheel_radius <= 0) {
      RCLCPP_ERROR_STREAM(get_logger(), "Invalid wheel radius provided: " << wheel_radius);
      required_parameters_received = false;
    }

    //Abort if any required parameters were not provided
    if (!required_parameters_received) {
      throw std::logic_error(
        "Required parameters were not received or were invalid. Please provide valid parameters."
      );
    }


    //Publishers
    pub_wheel_cmd_ = create_publisher<nuturtlebot_msgs::msg::WheelCommands>("wheel_cmd", 10);

    //Subscribers
    sub_cmd_vel_ = create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel",
      10,
      std::bind(&TurtleControl::cmd_vel_callback, this, std::placeholders::_1)
    );

    //Initialize turtlebot with input parameters
    turtlebot_ = DiffDrive {wheel_track, wheel_radius};

    RCLCPP_INFO_STREAM(get_logger(), "turtle_control node started");
  }
private:
  rclcpp::Publisher<nuturtlebot_msgs::msg::WheelCommands>::SharedPtr pub_wheel_cmd_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_cmd_vel_;

  DiffDrive turtlebot_ {0.16, 0.033}; //Default values, to be overwritten in constructor

  /// \brief take received cmd_vel twist, convert to wheel_cmd, and publish
  /// \param msg - received cmd_vel message
  void cmd_vel_callback(const geometry_msgs::msg::Twist & msg) {
    
    //Use inverse kinematics to calculate the required wheel velocities for the
    //input twist
    Wheel wheel_vel = turtlebot_.get_required_wheel_vel(Twist2D{
        msg.angular.z,
        msg.linear.x,
        msg.linear.y
    });

    //Convert wheel velocities in rad/s to dynamixel ticks per second
    wheel_vel.left *= TICKS_PER_RAD;
    wheel_vel.right *= TICKS_PER_RAD;

    //generate wheel command message
    nuturtlebot_msgs::msg::WheelCommands wheel_cmd;
    wheel_cmd.left_velocity = static_cast<int32_t>(wheel_vel.left);
    wheel_cmd.right_velocity = static_cast<int32_t>(wheel_vel.right);

    //publish wheel command message
    pub_wheel_cmd_->publish(wheel_cmd);
  }
};

/// \brief Run the node 
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TurtleControl>());
  rclcpp::shutdown();
  return 0;
}