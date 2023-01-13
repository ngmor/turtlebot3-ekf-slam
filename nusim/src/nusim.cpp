#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

//Basic node setup adapted from
//https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Cpp-Publisher-And-Subscriber.html

class NuSim : public rclcpp::Node {
public:
    NuSim(): Node("nusim") {
        //Parameters
        auto param_desc = rcl_interfaces::msg::ParameterDescriptor{};
        param_desc.description = "The rate the simulation runs at.";
        declare_parameter("rate", 200.0, param_desc);
        sim_rate_ = get_parameter("rate").get_parameter_value().get<double>();
        sim_interval_ = 1.0 / sim_rate_;

        timer_ = create_wall_timer(
            static_cast<std::chrono::microseconds>(static_cast<int>(sim_interval_*1000000.0)),
            std::bind(&NuSim::timer_callback, this)
        );
        test_publisher_ = create_publisher<std_msgs::msg::String>("~/topic", 10);
    }
private:
    double sim_rate_;
    double sim_interval_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr test_publisher_;

    void timer_callback() {
        auto message = std_msgs::msg::String();
        message.data = "Hello, world!";
        //RCLCPP_INFO(get_logger(), "Publishing");
        test_publisher_->publish(message);
    }
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<NuSim>());
    rclcpp::shutdown();
    return 0;
}