#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <string_view>
#include <vector>
#include <exception>
#include <typeinfo>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/u_int64.hpp"
#include "std_srvs/srv/empty.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"
#include "nusim/srv/teleport.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

using namespace std::chrono_literals;

// Constants
//TODO - define somewhere centrally?
constexpr std::string_view WORLD_FRAME = "nusim/world";
constexpr std::string_view ROBOT_GROUND_TRUTH_FRAME = "red/base_footprint";
constexpr double OBSTACLE_HEIGHT = 0.25;

//Simple pose struct
struct Pose2D {
    // x position
    double x = 0.0;

    // y position
    double y = 0.0;

    // angle
    double theta = 0.0;
};

//Custom size mismatch exception
class ObstacleListSizeMismatchException : public std::exception {
private:
    std::string msg_;
public:
    ObstacleListSizeMismatchException(int x, int y) :
        msg_{
            "Size mismatch between input obstacle coordinate lists (x: "
            + std::to_string(x) + " elements, y: " + std::to_string(y) + " elements)"
        }
    {};
    const char * what() const throw () { return msg_.c_str(); }
};


//Function prototypes
geometry_msgs::msg::TransformStamped pose_to_transform(Pose2D pose);


//Node class
class NuSim : public rclcpp::Node {
public:
    NuSim(): Node("nusim") {

        //Parameters
        auto param = rcl_interfaces::msg::ParameterDescriptor{};
        param.description = "The rate the simulation runs at.";
        declare_parameter("rate", 200.0, param);
        sim_rate_ = get_parameter("rate").get_parameter_value().get<double>();
        sim_interval_ = 1.0 / sim_rate_;


        param.description = "Initial x position of the robot.";
        declare_parameter("x0", 0.0, param);
        pose_initial_.x = get_parameter("x0").get_parameter_value().get<double>();

        param.description = "Initial y position of the robot.";
        declare_parameter("y0", 0.0, param);
        pose_initial_.y = get_parameter("y0").get_parameter_value().get<double>();

        param.description = "Initial rotation of the robot.";
        declare_parameter("theta0", 0.0, param);
        pose_initial_.theta = get_parameter("theta0").get_parameter_value().get<double>();
        
        pose_current_ = pose_initial_;


        param.description = "List of x positions of obstacles.";
        declare_parameter("obstacles.x", std::vector<double> {}, param);
        obstacles_x_ = get_parameter("obstacles.x").as_double_array();

        param.description = "List of y positions of obstacles.";
        declare_parameter("obstacles.y", std::vector<double> {}, param);
        obstacles_y_ = get_parameter("obstacles.y").as_double_array();

        //If vectors differ in size, exit node
        if (obstacles_x_.size() != obstacles_y_.size()) {
            throw ObstacleListSizeMismatchException(obstacles_x_.size(), obstacles_y_.size());
        }

        param.description = "Obstacle radius.";
        declare_parameter("obstacles.r", 0.015, param);
        obstacles_r_ = get_parameter("obstacles.r").get_parameter_value().get<double>();



        //Timers
        timer_ = create_wall_timer(
            static_cast<std::chrono::microseconds>(static_cast<int>(sim_interval_*1000000.0)),
            std::bind(&NuSim::timer_callback, this)
        );

        //Publishers
        pub_timestep_ = create_publisher<std_msgs::msg::UInt64>("~/timestep", 10);
        pub_obstacles_ = create_publisher<visualization_msgs::msg::MarkerArray>("~/obstacles", 10);

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

        init_obstacles();
    }

private:
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::UInt64>::SharedPtr pub_timestep_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_obstacles_;
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr srv_reset_;
    rclcpp::Service<nusim::srv::Teleport>::SharedPtr srv_teleport_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> broadcaster_;

    double sim_rate_, sim_interval_;
    uint64_t timestep_ = 0;
    Pose2D pose_initial_, pose_current_;
    std::vector<double> obstacles_x_, obstacles_y_;
    double obstacles_r_;
    visualization_msgs::msg::MarkerArray obstacle_markers_;

    void timer_callback() {
        // //RCLCPP_INFO(get_logger(), "Publishing");

        //Publish timestep and increment
        auto timestep_msg = std_msgs::msg::UInt64();
        timestep_msg.data = timestep_++;
        pub_timestep_->publish(timestep_msg);

        //Broadcast current transform of robot
        auto tf = pose_to_transform(pose_current_);
        tf.header.stamp = get_clock()->now();
        broadcaster_->sendTransform(tf);

        //Publish markers
        publish_obstacles();
    }

    void reset_callback(
        const std::shared_ptr<std_srvs::srv::Empty::Request> request,
        std::shared_ptr<std_srvs::srv::Empty::Response> response
    ) {
        //Get rid of unused warnings
        (void)request;
        (void)response;

        //Reset simulation timestep
        timestep_ = 0;

        //Reset current pose
        pose_current_ = pose_initial_;
    }

    void teleport_callback(
        const std::shared_ptr<nusim::srv::Teleport::Request> request,
        std::shared_ptr<nusim::srv::Teleport::Response> response
    ) {
        //Get rid of unused warnings
        (void)response;

        //Teleport current pose
        pose_current_.x = request->x;
        pose_current_.y = request->y;
        pose_current_.theta = request->theta;
    }

    void init_obstacles() {

        visualization_msgs::msg::Marker marker;

        //Create markers from input lists
        for (size_t i = 0; i < obstacles_x_.size(); i++) {
            //Reset marker
            marker = visualization_msgs::msg::Marker {};

            marker.header.frame_id = WORLD_FRAME;
            marker.id = i;
            marker.type = visualization_msgs::msg::Marker::CYLINDER;
            marker.action = visualization_msgs::msg::Marker::ADD;
            marker.pose.position.x = obstacles_x_[i];
            marker.pose.position.y = obstacles_y_[i];
            marker.pose.position.z = OBSTACLE_HEIGHT / 2;
            marker.scale.x = obstacles_r_*2;
            marker.scale.y = obstacles_r_*2;
            marker.scale.z = OBSTACLE_HEIGHT;
            marker.color.r = 1.0;
            marker.color.g = 0.0;
            marker.color.b = 0.0;
            marker.color.a = 1.0;

            //Append to marker array
            obstacle_markers_.markers.push_back(marker);
        }
    }

    void publish_obstacles() {
        
        auto time = get_clock()->now();

        //Update timestamps of all markers
        for (auto & marker : obstacle_markers_.markers) {
            marker.header.stamp = time;
        }

        //Publish marker array
        pub_obstacles_->publish(obstacle_markers_);
    }

};

//Get transform from pose
geometry_msgs::msg::TransformStamped pose_to_transform(Pose2D pose) {
    geometry_msgs::msg::TransformStamped tf;

    tf.header.frame_id = WORLD_FRAME;
    tf.child_frame_id = ROBOT_GROUND_TRUTH_FRAME;

    tf.transform.translation.x = pose.x;
    tf.transform.translation.y = pose.y;
    
    tf2::Quaternion q;
    q.setRPY(0,0, pose.theta);
    tf.transform.rotation.x = q.x();
    tf.transform.rotation.y = q.y();
    tf.transform.rotation.z = q.z();
    tf.transform.rotation.w = q.w();

    return tf;
}


int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<NuSim>());
    rclcpp::shutdown();
    return 0;
}