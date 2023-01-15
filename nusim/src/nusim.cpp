#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/u_int64.hpp"
#include "std_srvs/srv/empty.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"

using namespace std::chrono_literals;

//Simple pose struct
//TODO - replace with Transform2D?
struct Pose2D {
    // x position
    double x = 0.0;

    // y position
    double y = 0.0;

    // angle
    double theta = 0.0;
};

//Function prototypes
geometry_msgs::msg::TransformStamped pose_to_transform(Pose2D pose);


//Node class
class NuSim : public rclcpp::Node {
public:
    NuSim(): Node("nusim") {
        //Parameters
        auto param_desc = rcl_interfaces::msg::ParameterDescriptor{};
        param_desc.description = "The rate the simulation runs at.";
        declare_parameter("rate", 200.0, param_desc);
        sim_rate_ = get_parameter("rate").get_parameter_value().get<double>();
        sim_interval_ = 1.0 / sim_rate_;

        param_desc.description = "Initial x position of the robot.";
        declare_parameter("x0", 0.0, param_desc);
        pose_initial_.x = get_parameter("x0").get_parameter_value().get<double>();

        param_desc.description = "Initial y position of the robot.";
        declare_parameter("y0", 0.0, param_desc);
        pose_initial_.y = get_parameter("y0").get_parameter_value().get<double>();

        param_desc.description = "Initial rotation of the robot.";
        declare_parameter("theta0", 0.0, param_desc);
        pose_initial_.theta = get_parameter("theta0").get_parameter_value().get<double>();
        
        pose_current_ = pose_initial_;

        //Timers
        timer_ = create_wall_timer(
            static_cast<std::chrono::microseconds>(static_cast<int>(sim_interval_*1000000.0)),
            std::bind(&NuSim::timer_callback, this)
        );

        //Publishers
        pub_timestep_ = create_publisher<std_msgs::msg::UInt64>("~/timestep", 10);

        // Services
        srv_reset_ = create_service<std_srvs::srv::Empty>(
            "~/reset",
            std::bind(&NuSim::reset_callback, this, std::placeholders::_1, std::placeholders::_2)
        );

        //Broadcasters
        broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    }
private:
    double sim_rate_, sim_interval_;
    Pose2D pose_initial_, pose_current_;
    uint64_t timestep_ = 0;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::UInt64>::SharedPtr pub_timestep_;
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr srv_reset_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> broadcaster_;

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

};

//Get transform from pose
geometry_msgs::msg::TransformStamped pose_to_transform(Pose2D pose) {
    geometry_msgs::msg::TransformStamped tf;

    tf.header.frame_id = "nusim/world";
    tf.child_frame_id = "red/base_footprint";

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