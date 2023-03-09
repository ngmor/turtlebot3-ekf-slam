//TODO - document
#include <cmath>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "turtlelib/rigid2d.hpp"

using turtlelib::Vector2D;
using turtlelib::almost_equal;

constexpr double OBSTACLE_HEIGHT = 0.25;

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

    //Publishers
    pub_clusters_ = create_publisher<visualization_msgs::msg::MarkerArray>("clusters", 10);

    //Subscribers
    sub_lidar_scan_ = create_subscription<sensor_msgs::msg::LaserScan>(
      "scan",
      10,
      std::bind(&Landmarks::lidar_scan_callback, this, std::placeholders::_1)
    );

    //Init markers
    if (clusters_visualize_) {
      default_marker_.type = visualization_msgs::msg::Marker::CYLINDER;
      default_marker_.action = visualization_msgs::msg::Marker::ADD;
      default_marker_.pose.position.z = OBSTACLE_HEIGHT / 2.0;
      default_marker_.scale.x = 0.005;
      default_marker_.scale.y = 0.005;
      default_marker_.scale.z = OBSTACLE_HEIGHT;
      default_marker_.color.r = 1.0;
      default_marker_.color.g = 1.0;
      default_marker_.color.b = 0.0;
      default_marker_.color.a = 1.0;
    }

    RCLCPP_INFO_STREAM(get_logger(), "landmarks node started");
  }

private:
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_lidar_scan_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_clusters_;

  bool clusters_visualize_;
  double clusters_threshold_;
  visualization_msgs::msg::MarkerArray cluster_markers_;
  visualization_msgs::msg::Marker default_marker_;
  size_t max_markers_ = 0;

  void lidar_scan_callback(const sensor_msgs::msg::LaserScan & msg)
  {

    std::vector<Vector2D> measurements {};
    measurements.reserve(msg.ranges.size()); //max number of measurements equals the number of ranges

    std::vector<std::vector<Vector2D>> clusters {};
    clusters.reserve(msg.ranges.size()); //max number of clusters equals the number of ranges

    std::vector<Vector2D> current_cluster;

    Vector2D last_measurement;

    //Cluster points
    for (size_t i = 0; i < msg.ranges.size(); i++) {
      
      //Get range of point from message
      const auto & range = msg.ranges.at(i);

      //Do not consider out of range measurements in clustering
      if (range < msg.range_min) {continue;}

      //Calculate bearing from message
      const auto bearing = msg.angle_min + i*msg.angle_increment;

      //Convert into point as Vector2D
      const Vector2D measurement = {
        range * std::cos(bearing),
        range * std::sin(bearing)
      };

      //Store measurement
      measurements.push_back(measurement);

      //Do not perform distance comparison for first point
      if (measurements.size() > 1) {
        //calculate distance between this and last point
        const auto distance = (measurement - last_measurement).magnitude();

        //if measurement is above current threshold, store current cluster and start a new one
        if (distance > clusters_threshold_) {
          clusters.push_back(current_cluster);
          current_cluster.clear();
        }
      }

      //Add measurement to current cluster
      current_cluster.push_back(measurement);

      //Store last measurement for next iteration
      last_measurement = measurement;
    }

    //Wrap clusters
    if (clusters.size() > 1) {
      //if first and last measurements are within the threshold of each other,
      //add the last cluster to the first cluster
      if ((measurements.back() - measurements.front()).magnitude() <= clusters_threshold_) {
        //Add last cluster to first cluster
        //https://stackoverflow.com/questions/3177241/what-is-the-best-way-to-concatenate-two-vectors
        clusters.front().reserve(clusters.front().size() + clusters.back().size());
        clusters.front().insert(clusters.front().end(), clusters.back().begin(), clusters.back().end());

        //Remove last cluster
        clusters.pop_back();
      }
    }

    //Publish markers
    if (clusters_visualize_) {
      const auto markers_to_publish = measurements.size();

      if (max_markers_ < markers_to_publish) {
        max_markers_ = markers_to_publish;
      }

      cluster_markers_.markers.clear();

      default_marker_.header = msg.header;

      for (size_t i = 0; i < max_markers_; i++) {
        auto marker = default_marker_;
        
        marker.id = i;

        if (i < (markers_to_publish - 1)) {
          marker.pose.position.x = measurements.at(i).x;
          marker.pose.position.y = measurements.at(i).y;
        } else { //delete markers beyond what we see
          marker.action = visualization_msgs::msg::Marker::DELETE;
        }

        cluster_markers_.markers.push_back(marker);
      }

      //Publish markers
      pub_clusters_->publish(cluster_markers_);
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
