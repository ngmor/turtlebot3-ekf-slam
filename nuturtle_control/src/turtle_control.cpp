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