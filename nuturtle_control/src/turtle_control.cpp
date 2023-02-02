#include <stdexcept>
#include "turtlelib/diff_drive.hpp"
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "nuturtlebot_msgs/msg/wheel_commands.hpp"
#include "nuturtlebot_msgs/msg/sensor_data.hpp"

using turtlelib::DiffDrive;
using turtlelib::Wheel;
using turtlelib::Twist2D;
using turtlelib::PI;
using turtlelib::almost_equal;

constexpr int TICKS_PER_REV = 4096;
constexpr double TICKS_PER_RAD = TICKS_PER_REV / (2.0*PI);
constexpr double RAD_PER_TICKS = (2.0*PI) / TICKS_PER_REV;

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
    pub_joint_states_ = create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);

    //Subscribers
    sub_cmd_vel_ = create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel",
      10,
      std::bind(&TurtleControl::cmd_vel_callback, this, std::placeholders::_1)
    );
    sub_sensor_data_ = create_subscription<nuturtlebot_msgs::msg::SensorData>(
      "sensor_data",
      10,
      std::bind(&TurtleControl::sensor_data_callback, this, std::placeholders::_1)
    );


    //Initialize turtlebot with input parameters
    turtlebot_ = DiffDrive {wheel_track, wheel_radius};
    wheel_pos_last_ = turtlebot_.config().wheel_pos;

    //Init joint states
    joint_states_.name = {
      "wheel_left_joint",
      "wheel_right_joint"
    };
    joint_states_.position = {
      wheel_pos_last_.left,
      wheel_pos_last_.right
    };
    joint_states_.velocity = {
      0.0,
      0.0
    };

    sensor_stamp_last_ = get_clock()->now();

    RCLCPP_INFO_STREAM(get_logger(), "turtle_control node started");
  }
private:
  rclcpp::Publisher<nuturtlebot_msgs::msg::WheelCommands>::SharedPtr pub_wheel_cmd_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_joint_states_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_cmd_vel_;
  rclcpp::Subscription<nuturtlebot_msgs::msg::SensorData>::SharedPtr sub_sensor_data_;

  DiffDrive turtlebot_ {0.16, 0.033}; //Default values, to be overwritten in constructor
  Wheel wheel_pos_last_ {0,0};
  rclcpp::Time sensor_stamp_last_;
  sensor_msgs::msg::JointState joint_states_;

  /// \brief take received cmd_vel twist, convert to wheel_cmd, and publish
  /// \param msg - received cmd_vel message
  void cmd_vel_callback(const geometry_msgs::msg::Twist & msg)
  {
    
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

  /// \brief interpret wheel position and velocity from sensor data, publish to joint states
  /// \param msg - received sensor_data message
  void sensor_data_callback(const nuturtlebot_msgs::msg::SensorData & msg)
  {
    //Convert received wheel position to radians
    Wheel wheel_pos {
      static_cast<double>(msg.left_encoder)*RAD_PER_TICKS,
      static_cast<double>(msg.right_encoder)*RAD_PER_TICKS
    };

    double elapsed_time = 
      static_cast<double>(msg.stamp.nanosec - sensor_stamp_last_.nanoseconds()) * 1.0e-9;

    //Calculate wheel velocity
    Wheel wheel_vel {0,0};

    if (!almost_equal(elapsed_time, 0.0)) {
      wheel_vel.left = (wheel_pos.left - wheel_pos_last_.left) / elapsed_time;
      wheel_vel.right = (wheel_pos.right - wheel_pos_last_.right) / elapsed_time;
    }

    //Update joint state message
    joint_states_.header.stamp = msg.stamp;
    joint_states_.position[0] = wheel_pos.left;
    joint_states_.position[1] = wheel_pos.right;
    joint_states_.velocity[0] = wheel_vel.left;
    joint_states_.velocity[1] = wheel_vel.right;

    //publish joint state message
    pub_joint_states_->publish(joint_states_);

    //Update stored last values
    wheel_pos_last_ = wheel_pos;
    sensor_stamp_last_ = msg.stamp;
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