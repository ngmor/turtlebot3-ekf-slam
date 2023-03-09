//TODO - document

#include "rclcpp/rclcpp.hpp"

/// \brief Performs landmark detections from input lidar data
class Landmarks : public rclcpp::Node
{
public:
  /// \brief initialize the node
  Landmarks()
  : Node("landmarks")
  {
    //Parameters
    auto param = rcl_interfaces::msg::ParameterDescriptor{};

    param.description = "Controls whether clustered points are published as markers.";
    declare_parameter("visualize_clusters", true, param);
    visualize_clusters_ = get_parameter("visualize_clusters").get_parameter_value().get<bool>();

    RCLCPP_INFO_STREAM(get_logger(), "landmarks node started");
  }

private:
  bool visualize_clusters_;
};

/// \brief Run the node
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Landmarks>());
  rclcpp::shutdown();
  return 0;
}
