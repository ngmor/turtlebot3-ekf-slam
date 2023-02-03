#include <stdexcept>
#include <string>
#include <array>
#include "turtlelib/diff_drive.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/transform_broadcaster.h"

using turtlelib::DiffDrive;
using turtlelib::Wheel;

/// \brief Calculates odometry for the turtlebot
class Odometry : public rclcpp::Node
{
public:
  /// \brief initialize the node
  Odometry()
  : Node ("odometry")
  {
    //TODO remove
    //temporary run command
    //ros2 run nuturtle_control odometry --ros-args -p body_id:=base_footprint -p wheel_left:=wheel_left_joint -p wheel_right:=wheel_right_joint --params-file install/nuturtle_description/share/nuturtle_description/config/diff_params.yaml

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
    wheel_left_joint_ = get_parameter("wheel_left").get_parameter_value().get<std::string>();

    if (wheel_left_joint_ == "") {
      RCLCPP_ERROR_STREAM(get_logger(), "No left wheel joint provided.");
      required_parameters_received = false;
    }

    param.description = "The name of the robot's right wheel joint (REQUIRED)";
    declare_parameter("wheel_right", "", param);
    wheel_right_joint_ = get_parameter("wheel_right").get_parameter_value().get<std::string>();

    if (wheel_right_joint_ == "") {
      RCLCPP_ERROR_STREAM(get_logger(), "No right wheel joint provided.");
      required_parameters_received = false;
    }

    param.description = "The wheel track width in meters (REQUIRED)";
    declare_parameter("track_width", 0.0, param);
    double wheel_track = get_parameter("track_width").get_parameter_value().get<double>();

    if (wheel_track <= 0) {
      RCLCPP_ERROR_STREAM(get_logger(), "Invalid wheel track provided: " << wheel_track);
      required_parameters_received = false;
    }

    param.description = "The wheel radius in meters (REQUIRED)";
    declare_parameter("wheel_radius", 0.0, param);
    double wheel_radius = get_parameter("wheel_radius").get_parameter_value().get<double>();

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
    pub_odom_ = create_publisher<nav_msgs::msg::Odometry>("odom", 10);
    
    //Subscribers
    sub_joint_states_ = create_subscription<sensor_msgs::msg::JointState>(
      "joint_states",
      10,
      std::bind(&Odometry::joint_states_callback, this, std::placeholders::_1)
    );
    
    //Initialize turtlebot with input parameters and at q(0,0,0)
    turtlebot_ = DiffDrive {wheel_track, wheel_radius};

    //Init odom message
    odom_msg_.header.frame_id = odom_id_;
    odom_msg_.child_frame_id = body_id_;
    odom_msg_.pose.pose.position.z = 0;
    odom_msg_.pose.covariance = std::array<double,36> {36, 0};
    odom_msg_.twist.twist.linear.z = 0;
    odom_msg_.twist.twist.angular.x = 0;
    odom_msg_.twist.twist.angular.y = 0;
    odom_msg_.twist.covariance = std::array<double,36> {36, 0};


    RCLCPP_INFO_STREAM(get_logger(), "odometry node started");
  }
private:
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odom_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_joint_states_;
  
  std::string body_id_, odom_id_, wheel_left_joint_, wheel_right_joint_;
  DiffDrive turtlebot_ {0.16, 0.033}; //Default values, to be overwritten in constructor
  nav_msgs::msg::Odometry odom_msg_;

  /// \brief update internal odometry from received joint states
  /// \param msg - joint states
  void joint_states_callback(const sensor_msgs::msg::JointState & msg)
  {
    Wheel wheel_pos;
    Wheel wheel_vel;
    int wheel_count = 0;

    //Search for joints and wheel values
    for (unsigned int i = 0; i < msg.name.size(); i++) {
      if (msg.name.at(i) == wheel_left_joint_) {
        wheel_pos.left = msg.position.at(i);
        wheel_vel.left = msg.velocity.at(i);
        wheel_count++;
      } else if (msg.name.at(i) == wheel_right_joint_) {
        wheel_pos.right = msg.position.at(i);
        wheel_vel.right = msg.velocity.at(i);
        wheel_count++;
      }
    }

    //No point in doing odometry if both wheels have not been detected
    if (wheel_count != 2) {return;}

    //Calculate new configuration
    turtlebot_.update_config(wheel_pos);

    // build odometry message
    odom_msg_.pose.pose.position.x = turtlebot_.config().location.translation().x;
    odom_msg_.pose.pose.position.y = turtlebot_.config().location.translation().y;
    
    tf2::Quaternion q;
    q.setRPY(0,0, turtlebot_.config().location.rotation());
    odom_msg_.pose.pose.orientation = tf2::toMsg(q);

    odom_msg_.header.stamp = msg.header.stamp;

    //TODO - velocity

    //Publish odometry message
    pub_odom_->publish(odom_msg_);

    //TODO - broadcaster
  }
};

/// \brief Run the node 
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Odometry>());
  rclcpp::shutdown();
  return 0;
}