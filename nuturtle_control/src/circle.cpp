#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

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
    

    RCLCPP_INFO_STREAM(get_logger(), "circle node started");
  }
private:
  
};

/// \brief Run the node 
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Circle>());
  rclcpp::shutdown();
  return 0;
}