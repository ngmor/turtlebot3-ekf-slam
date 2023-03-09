//TODO - document
#include <cmath>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "turtlelib/rigid2d.hpp"

using turtlelib::Vector2D;

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

    sub_lidar_scan_ = create_subscription<sensor_msgs::msg::LaserScan>(
      "scan",
      10,
      std::bind(&Landmarks::lidar_scan_callback, this, std::placeholders::_1)
    );

    RCLCPP_INFO_STREAM(get_logger(), "landmarks node started");
  }

private:
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_lidar_scan_;

  bool clusters_visualize_;
  double clusters_threshold_;

  void lidar_scan_callback(const sensor_msgs::msg::LaserScan & msg)
  {

    std::vector<Vector2D> measurements {msg.ranges.size(), Vector2D{}};

    //Convert into points as Vector2Ds
    for (size_t i = 0; i < msg.ranges.size(); i++) {
      auto & measurement = measurements.at(i);

      const auto & range = msg.ranges.at(i);
      const auto bearing = msg.angle_min + i*msg.angle_increment;
      
      measurement = {
        range * std::cos(bearing),
        range * std::sin(bearing)
      };

      //Skip clustering for first point
      if (i == 0) {continue;}
    }
  }
};

/// \brief Run the node
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Landmarks>());
  rclcpp::shutdown();
  return 0;
}
