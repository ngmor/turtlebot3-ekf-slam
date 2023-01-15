#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/u_int64.hpp"
#include "std_srvs/srv/empty.hpp"

using namespace std::chrono_literals;


class NuSim : public rclcpp::Node {
public:
    NuSim(): Node("nusim") {
        //Parameters
        auto param_desc = rcl_interfaces::msg::ParameterDescriptor{};
        param_desc.description = "The rate the simulation runs at.";
        declare_parameter("rate", 200.0, param_desc);
        sim_rate_ = get_parameter("rate").get_parameter_value().get<double>();
        sim_interval_ = 1.0 / sim_rate_;

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
    }
private:
    double sim_rate_;
    double sim_interval_;
    uint64_t timestep_ = 0;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::UInt64>::SharedPtr pub_timestep_;
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr srv_reset_;

    void timer_callback() {
        // //RCLCPP_INFO(get_logger(), "Publishing");

        //Publish timestep and increment
        auto timestep_msg = std_msgs::msg::UInt64();
        timestep_msg.data = timestep_++;
        pub_timestep_->publish(timestep_msg);
    }

    void reset_callback(
        const std::shared_ptr<std_srvs::srv::Empty::Request> request,
        std::shared_ptr<std_srvs::srv::Empty::Response> response
    ) {
        //Get rid of unused warnings
        (void)request;
        (void)response;

        timestep_ = 0;
    }

};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<NuSim>());
    rclcpp::shutdown();
    return 0;
}