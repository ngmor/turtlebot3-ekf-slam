/// \file
/// \brief Runs the simulation for the NUTurtle.
///
/// PARAMETERS:
///     track_width (double): The wheel track width in meters (REQUIRED).
///     wheel_radius (double): The wheel radius in meters (REQUIRED).
///     motor_cmd_per_rad_sec (double): Motor command value per rad/sec conversion factor (REQUIRED).
///     motor_cmd_max (int32_t): Maximum possible motor command value (REQUIRED).
///     encoder_ticks_per_rad (double): Motor encoder ticks per radian conversion factor (REQUIRED).
///     rate (double): The rate the simulation runs at (Hz).
///     x0 (double): Initial x position of the robot (m).
///     y0 (double): Initial y position of the robot (m).
///     theta0 (double): Initial rotation of the robot (rad).
///     obstacles.x (std::vector<double>): List of x starting positions of obstacles (m). Arbitrary length, but must match length of `y`.
///     obstacles.y (std::vector<double>): List of y starting positions of obstacles (m). Arbitray length, but must match length of `x`.
///     obstacles.r (double): Radius of all cylinder obstacles (m). Single value applies to all obstacles.
///     x_length (double): Length of the arena in the x direction (m).
///     y_length (double): Length of the arena in the y direction (m).
/// PUBLISHERS:
///     ~/timestep (std_msgs/msg/UInt64): current timestep of the simulation
///     ~/obstacles (visualization_msgs/msg/MarkerArray): marker array containing cylindrical obstacles in the world.
///     sensor_data (nuturtlebot_msgs/msg/SensorData): simulated sensor data
/// SUBSCRIBERS:
///     wheel_cmd (nuturtlebot_msgs/msg/WheelCommands): wheel commands from control nodes
/// SERVERS:
///     ~/reset (std_srvs/srv/Empty): resets the simulation to its starting state
///     ~/teleport (nusim/srv/Teleport): teleports the actual turtlebot to a provided location
/// CLIENTS:
///     none

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <string_view>
#include <vector>
#include <exception>
#include <algorithm>
#include <random>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/u_int64.hpp"
#include "std_srvs/srv/empty.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"
#include "nusim/srv/teleport.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "nuturtlebot_msgs/msg/wheel_commands.hpp"
#include "nuturtlebot_msgs/msg/sensor_data.hpp"
#include "turtlelib/diff_drive.hpp"

using namespace std::chrono_literals;
using turtlelib::almost_equal;
using turtlelib::Vector2D;
using turtlelib::Transform2D;
using turtlelib::DiffDrive;
using turtlelib::DiffDriveConfig;
using turtlelib::Wheel;

// Constants
constexpr std::string_view WORLD_FRAME = "nusim/world";
constexpr std::string_view ROBOT_GROUND_TRUTH_FRAME = "red/base_footprint";
constexpr double OBSTACLE_HEIGHT = 0.25;
constexpr double WALL_HEIGHT = 0.25;
constexpr double WALL_WIDTH = 0.1;
constexpr int32_t MARKER_ID_WALL = 0;
constexpr int32_t MARKER_ID_OFFSET_OBSTACLES = 100;


//Function prototypes
geometry_msgs::msg::TransformStamped pose_to_transform(Transform2D pose);
std::mt19937 & get_random();


/// \brief Runs the simulation for the NUTurtle.
class NuSim : public rclcpp::Node
{
public:
  /// \brief initialize the node
  NuSim()
  : Node("nusim")
  {

    //Parameters
    auto param = rcl_interfaces::msg::ParameterDescriptor{};

    //Check if required parameters were provided
    bool required_parameters_received = true;

    param.description = "The wheel track width in meters (REQUIRED).";
    declare_parameter("track_width", 0.0, param);
    double wheel_track = get_parameter("track_width").get_parameter_value().get<double>();

    if (wheel_track <= 0) {
      RCLCPP_ERROR_STREAM(get_logger(), "Invalid wheel track provided: " << wheel_track);
      required_parameters_received = false;
    }

    param.description = "The wheel radius in meters (REQUIRED).";
    declare_parameter("wheel_radius", 0.0, param);
    double wheel_radius = get_parameter("wheel_radius").get_parameter_value().get<double>();

    if (wheel_radius <= 0.0) {
      RCLCPP_ERROR_STREAM(get_logger(), "Invalid wheel radius provided: " << wheel_radius);
      required_parameters_received = false;
    }

    param.description = "Motor command value per rad/sec conversion factor (REQUIRED).";
    declare_parameter("motor_cmd_per_rad_sec", 0.0, param);
    motor_cmd_per_rad_sec_ = get_parameter(
      "motor_cmd_per_rad_sec").get_parameter_value().get<double>();

    if (motor_cmd_per_rad_sec_ <= 0.0) {
      RCLCPP_ERROR_STREAM(
        get_logger(),
        "Invalid motor command to rad/sec conversion provided: " << motor_cmd_per_rad_sec_);
      required_parameters_received = false;
    }

    param.description = "Maximum possible motor command value (REQUIRED).";
    declare_parameter("motor_cmd_max", 0, param);
    motor_cmd_max_ = get_parameter(
      "motor_cmd_max").get_parameter_value().get<int32_t>();

    if (motor_cmd_max_ <= 0) {
      RCLCPP_ERROR_STREAM(
        get_logger(),
        "Invalid maximum motor command provided: " << motor_cmd_max_);
      required_parameters_received = false;
    }

    param.description = "Motor encoder ticks per radian conversion factor (REQUIRED).";
    declare_parameter("encoder_ticks_per_rad", 0.0, param);
    encoder_ticks_per_rad_ = get_parameter(
      "encoder_ticks_per_rad").get_parameter_value().get<double>();

    if (encoder_ticks_per_rad_ <= 0.0) {
      RCLCPP_ERROR_STREAM(
        get_logger(),
        "Invalid encoder ticks to radian conversion provided: " << encoder_ticks_per_rad_);
      required_parameters_received = false;
    }

    param.description = "The rate the simulation runs at (Hz).";
    declare_parameter("rate", 200.0, param);
    sim_rate_ = get_parameter("rate").get_parameter_value().get<double>();
    sim_interval_ = 1.0 / sim_rate_;

    Vector2D translation_initial;

    param.description = "Initial x position of the robot (m).";
    declare_parameter("x0", 0.0, param);
    translation_initial.x = get_parameter("x0").get_parameter_value().get<double>();

    param.description = "Initial y position of the robot (m).";
    declare_parameter("y0", 0.0, param);
    translation_initial.y = get_parameter("y0").get_parameter_value().get<double>();

    param.description = "Initial rotation of the robot (rad).";
    declare_parameter("theta0", 0.0, param);
    double rotation_initial = get_parameter("theta0").get_parameter_value().get<double>();

    param.description =
      "List of x starting positions of obstacles (m). Arbitrary length, but must match length of y.";
    declare_parameter("obstacles.x", std::vector<double> {}, param);
    obstacles_x_ = get_parameter("obstacles.x").as_double_array();

    param.description =
      "List of y starting positions of obstacles (m). Arbitray length, but must match length of x.";
    declare_parameter("obstacles.y", std::vector<double> {}, param);
    obstacles_y_ = get_parameter("obstacles.y").as_double_array();

    //If vectors differ in size, exit node
    if (obstacles_x_.size() != obstacles_y_.size()) {
      RCLCPP_ERROR_STREAM(
        get_logger(),
        "Size mismatch between input obstacle coordinate lists (x: " << obstacles_x_.size() <<
          " elements, y: " << obstacles_y_.size() << " elements)"
      );
      required_parameters_received = false;
    }

    param.description =
      "Radius of all cylinder obstacles (m). Single value applies to all obstacles.";
    declare_parameter("obstacles.r", 0.015, param);
    obstacles_r_ = get_parameter("obstacles.r").get_parameter_value().get<double>();

    param.description =
      "Length of the arena in the x direction (m).";
    declare_parameter("x_length", 5.0, param);
    x_length_ = get_parameter("x_length").get_parameter_value().get<double>();

    param.description =
      "Length of the arena in the y direction (m).";
    declare_parameter("y_length", 5.0, param);
    y_length_ = get_parameter("y_length").get_parameter_value().get<double>();

    param.description = 
      "Standard deviation for noise on input wheel commands (rad/s).";
    declare_parameter("input_noise", 0.1, param);
    wheel_vel_dist_ = std::normal_distribution<> {
      0.0, //mean
      get_parameter("input_noise").get_parameter_value().get<double>()
    };


    //Abort if any required parameters were not provided
    if (!required_parameters_received) {
      throw std::logic_error(
              "Required parameters were not received or were invalid. Please provide valid parameters."
      );
    }

    //Timers
    timer_ = create_wall_timer(
      static_cast<std::chrono::microseconds>(static_cast<int>(sim_interval_ * 1000000.0)),
      std::bind(&NuSim::timer_callback, this)
    );

    //Publishers
    pub_timestep_ = create_publisher<std_msgs::msg::UInt64>("~/timestep", 10);
    pub_obstacles_ = create_publisher<visualization_msgs::msg::MarkerArray>("~/obstacles", 10);
    //TODO - I don't think this should have a prefix, have to check
    pub_sensor_data_ = create_publisher<nuturtlebot_msgs::msg::SensorData>("sensor_data", 10);

    //Subscribers
    sub_wheel_cmd_ = create_subscription<nuturtlebot_msgs::msg::WheelCommands>(
      "wheel_cmd", //TODO - I don't think this should have a prefix, have to check
      10,
      std::bind(&NuSim::wheel_cmd_callback, this, std::placeholders::_1)
    );

    // Services
    srv_reset_ = create_service<std_srvs::srv::Empty>(
      "~/reset",
      std::bind(&NuSim::reset_callback, this, std::placeholders::_1, std::placeholders::_2)
    );
    srv_teleport_ = create_service<nusim::srv::Teleport>(
      "~/teleport",
      std::bind(&NuSim::teleport_callback, this, std::placeholders::_1, std::placeholders::_2)
    );

    //Broadcasters
    broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    //Other variables
    turtlebot_ = DiffDrive {
      wheel_track,
      wheel_radius,
      {
        translation_initial,
        rotation_initial
      }
    };

    init_obstacles();

    RCLCPP_INFO_STREAM(get_logger(), "nusim node started");
  }

private:
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::UInt64>::SharedPtr pub_timestep_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_obstacles_;
  rclcpp::Publisher<nuturtlebot_msgs::msg::SensorData>::SharedPtr pub_sensor_data_;
  rclcpp::Subscription<nuturtlebot_msgs::msg::WheelCommands>::SharedPtr sub_wheel_cmd_;

  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr srv_reset_;
  rclcpp::Service<nusim::srv::Teleport>::SharedPtr srv_teleport_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> broadcaster_;

  double sim_rate_, sim_interval_;
  uint64_t timestep_ = 0;
  DiffDrive turtlebot_ {0.16, 0.033}; //Default values, to be overwritten in constructor
  Wheel wheel_vel_ {0.0, 0.0};
  std::vector<double> obstacles_x_, obstacles_y_;
  double obstacles_r_, x_length_, y_length_;
  visualization_msgs::msg::MarkerArray obstacle_markers_;
  double motor_cmd_per_rad_sec_, encoder_ticks_per_rad_;
  int32_t motor_cmd_max_;
  rclcpp::Time current_time_;
  std::normal_distribution<> wheel_vel_dist_;

  /// \brief main simulation timer loop
  void timer_callback()
  {
    //Use a single time value for all publishing
    current_time_ = get_clock()->now();

    //Publish timestep and increment
    auto timestep_msg = std_msgs::msg::UInt64();
    timestep_msg.data = timestep_++;
    pub_timestep_->publish(timestep_msg);

    //Update wheel positions and publish
    update_wheel_pos_and_config();

    //Publish markers
    publish_obstacles();
  }

  /// \brief update the robot's wheel positions based on the current
  /// commanded velocity and update the robot's configuration based
  /// on the new wheel position
  void update_wheel_pos_and_config()
  {
    //Create new wheel position based on wheel velocity
    Wheel new_wheel_pos {
      turtlebot_.config().wheel_pos.left + wheel_vel_.left * sim_interval_,
      turtlebot_.config().wheel_pos.right + wheel_vel_.right * sim_interval_,
    };

    //Publish new wheel position
    nuturtlebot_msgs::msg::SensorData sensor_data;
    sensor_data.stamp = current_time_;
    sensor_data.left_encoder = static_cast<int32_t>(new_wheel_pos.left * encoder_ticks_per_rad_);
    sensor_data.right_encoder = static_cast<int32_t>(new_wheel_pos.right * encoder_ticks_per_rad_);

    pub_sensor_data_->publish(sensor_data);

    //Update configuration based on new wheel position
    turtlebot_.update_config(new_wheel_pos);

    //Broadcast updated transform of robot
    auto tf = pose_to_transform(turtlebot_.config().location);
    tf.header.stamp = current_time_;
    broadcaster_->sendTransform(tf);
  }

  /// \brief initialize the obstacle marker array.
  void init_obstacles()
  {

    visualization_msgs::msg::Marker marker;

    //Add walls
    marker.header.frame_id = WORLD_FRAME;
    marker.id = MARKER_ID_WALL;
    marker.type = visualization_msgs::msg::Marker::CUBE_LIST;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.scale.x = WALL_WIDTH;
    marker.scale.y = WALL_WIDTH;
    marker.scale.z = WALL_HEIGHT;
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    marker.color.a = 1.0;
    marker.points = {};

    double x_bound = (x_length_ + WALL_WIDTH) / 2.0;
    double y_bound = (y_length_ + WALL_WIDTH) / 2.0;

    geometry_msgs::msg::Point point;
    point.z = WALL_HEIGHT / 2.0;

    //Add top and bottom wall points
    for (double x = -x_bound; x <= x_bound; x += WALL_WIDTH) {
      //Bottom wall
      point.x = x;
      point.y = -y_bound;
      marker.points.push_back(point);

      //Top wall
      point.y = y_bound;
      marker.points.push_back(point);
    }

    //Add left and right wall points
    for (double y = -y_bound; y <= y_bound; y += WALL_WIDTH) {
      //Left wall
      point.x = -x_bound;
      point.y = y;
      marker.points.push_back(point);

      //Right wall
      point.x = x_bound;
      marker.points.push_back(point);
    }

    obstacle_markers_.markers.push_back(marker);

    //Create markers from input lists
    for (size_t i = 0; i < obstacles_x_.size(); i++) {
      //Reset marker
      marker = visualization_msgs::msg::Marker {};

      marker.header.frame_id = WORLD_FRAME;
      marker.id = i + MARKER_ID_OFFSET_OBSTACLES;
      marker.type = visualization_msgs::msg::Marker::CYLINDER;
      marker.action = visualization_msgs::msg::Marker::ADD;
      marker.pose.position.x = obstacles_x_.at(i);
      marker.pose.position.y = obstacles_y_.at(i);
      marker.pose.position.z = OBSTACLE_HEIGHT / 2.0;
      marker.scale.x = obstacles_r_ * 2.0;
      marker.scale.y = obstacles_r_ * 2.0;
      marker.scale.z = OBSTACLE_HEIGHT;
      marker.color.r = 1.0;
      marker.color.g = 0.0;
      marker.color.b = 0.0;
      marker.color.a = 1.0;

      //Append to marker array
      obstacle_markers_.markers.push_back(marker);
    }


  }

  /// \brief publish the obstacle marker array
  void publish_obstacles()
  {

    //Update timestamps of all markers
    for (auto & marker : obstacle_markers_.markers) {
      marker.header.stamp = current_time_;
    }

    //Publish marker array
    pub_obstacles_->publish(obstacle_markers_);
  }

  /// \brief convert and store received wheel commands
  /// \param msg - received wheel command message
  void wheel_cmd_callback(const nuturtlebot_msgs::msg::WheelCommands & msg)
  {
    //Clamp velocities to max allowed and
    //store wheel velocities in rad/s
    wheel_vel_.left = static_cast<double>(
      std::clamp(msg.left_velocity, -motor_cmd_max_, motor_cmd_max_)
      ) / motor_cmd_per_rad_sec_;
    wheel_vel_.right = static_cast<double>(
      std::clamp(msg.right_velocity, -motor_cmd_max_, motor_cmd_max_)
      ) / motor_cmd_per_rad_sec_;

    //If wheel command is not 0 and we have nonzero standard deviation, inject input noise
    if (wheel_vel_dist_.stddev() != 0.0){
      if (!almost_equal(wheel_vel_.left, 0.0)) {
        wheel_vel_.left += wheel_vel_dist_(get_random());
      }

      if (!almost_equal(wheel_vel_.right, 0.0)) {
        wheel_vel_.right += wheel_vel_dist_(get_random());
      }
    }
  }

  /// \brief reset simulation back to initial parameters. callback for ~/reset service.
  void reset_callback(
    const std::shared_ptr<std_srvs::srv::Empty::Request>,
    std::shared_ptr<std_srvs::srv::Empty::Response>
  )
  {
    //Reset simulation timestep
    timestep_ = 0;

    //Reset current pose
    turtlebot_.reset();
  }

  /// \brief teleport the actual robot to a specified pose. callback for ~/teleport service.
  /// \param request - pose data to which to teleport the robot.
  void teleport_callback(
    const std::shared_ptr<nusim::srv::Teleport::Request> request,
    std::shared_ptr<nusim::srv::Teleport::Response>
  )
  {
    //Teleport current pose
    turtlebot_.set_config(
      DiffDriveConfig{
      {
        {
          request->config.x,
          request->config.y
        },
        request->config.theta
      },
      turtlebot_.config().wheel_pos
    });
  }
};

/// \brief format a pose as a TransformStamped message
/// \param pose - pose to turn into a transform
/// \return - TransformStamped message for the pose, with no timestamp
geometry_msgs::msg::TransformStamped pose_to_transform(Transform2D pose)
{
  geometry_msgs::msg::TransformStamped tf;

  tf.header.frame_id = WORLD_FRAME;
  tf.child_frame_id = ROBOT_GROUND_TRUTH_FRAME;

  tf.transform.translation.x = pose.translation().x;
  tf.transform.translation.y = pose.translation().y;

  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, pose.rotation());
  tf.transform.rotation.x = q.x();
  tf.transform.rotation.y = q.y();
  tf.transform.rotation.z = q.z();
  tf.transform.rotation.w = q.w();

  return tf;
}

std::mt19937 & get_random()
{
    // static variables inside a function are created once and persist for the remainder of the program
    static std::random_device rd{}; 
    static std::mt19937 mt{rd()};
    // we return a reference to the pseudo-random number genrator object. This is always the
    // same object every time get_random is called
    return mt;
}

/// \brief Run the node
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<NuSim>());
  rclcpp::shutdown();
  return 0;
}
