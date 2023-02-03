#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nuturtle_control/srv/circle.hpp"

/// \brief Moves the turtlebot in a circle with a configurable velocity/radius
class Circle : public rclcpp::Node
{
public:
  /// \brief initialize the node
  Circle()
  : Node ("circle")
  {
    //TODO remove
    //temporary run command
    //ros2 run nuturtle_control turtle_control --ros-args --params-file install/nuturtle_description/share/nuturtle_description/config/diff_params.yaml

    //Parameters
    auto param = rcl_interfaces::msg::ParameterDescriptor{};

    param.description = "The frequency of publishing cmd_vel messages (Hz).";
    declare_parameter("frequency", 100.0, param);
    sim_freq_ = get_parameter("frequency").get_parameter_value().get<double>();
    sim_interval_ = 1.0 / sim_freq_;
    
    //Timers
    timer_ = create_wall_timer(
      static_cast<std::chrono::microseconds>(static_cast<int>(sim_interval_ * 1000000.0)),
      std::bind(&Circle::timer_callback, this)
    );

    //Publishers
    pub_cmd_vel_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel",10);

    //Services
    srv_control_ = create_service<nuturtle_control::srv::Circle>(
      "control",
      std::bind(&Circle::control_callback, this, std::placeholders::_1, std::placeholders::_2)
    );

    //Init cmd_vel twist
    cmd_vel_.linear.y = 0;
    cmd_vel_.linear.z = 0;
    cmd_vel_.angular.x = 0;
    cmd_vel_.angular.y = 0;

    RCLCPP_INFO_STREAM(get_logger(), "circle node started");
  }
private:
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_cmd_vel_;
  rclcpp::Service<nuturtle_control::srv::Circle>::SharedPtr srv_control_;

  double sim_freq_, sim_interval_;
  bool publish_ = false;
  geometry_msgs::msg::Twist cmd_vel_;

  /// \brief main timer loop for publishing cmd_vel messages
  void timer_callback()
  {
    if (publish_)
    {
      pub_cmd_vel_->publish(cmd_vel_);
    }
  }

  void control_callback(
    const std::shared_ptr<nuturtle_control::srv::Circle::Request> request,
    std::shared_ptr<nuturtle_control::srv::Circle::Response> response
  )
  {
    //Get rid of unused warnings
    (void)response;

    cmd_vel_.angular.z = request->velocity;
    cmd_vel_.linear.x = request->radius*request->velocity;

    //start publishing
    publish_ = true;
  }
};

/// \brief Run the node
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Circle>());
  rclcpp::shutdown();
  return 0;
}