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
    declare_parameter("clusters.visualize", false, param);
    clusters_visualize_ = get_parameter("clusters.visualize").get_parameter_value().get<bool>();

    param.description = "Euclidean distance between points to be considered part of the same cluster.";
    declare_parameter("clusters.threshold", 0.01, param);
    clusters_threshold_ = get_parameter("clusters.threshold").get_parameter_value().get<double>();

    RCLCPP_INFO_STREAM(get_logger(), "landmarks node started");
  }

private:
  bool clusters_visualize_;
  double clusters_threshold_;
};

/// \brief Run the node
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Landmarks>());
  rclcpp::shutdown();
  return 0;
}
