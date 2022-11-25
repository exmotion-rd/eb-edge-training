#include <ctime>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/header.hpp"
#include "thermo_msgs/msg/temperature.hpp"

class ThermoMonitor : public rclcpp::Node
{
public:
    ThermoMonitor()
        : Node("thermo_cpp_monitor")
    {
        auto callback = [this](const thermo_msgs::msg::Temperature::SharedPtr msg) -> void
        {
            auto timestamp = this->format_timestamp(msg->header.stamp);
            RCLCPP_INFO(this->get_logger(), "[%s] temperature=%.2f", timestamp.c_str(), msg->temperature);
        };
        subscription_ = this->create_subscription<thermo_msgs::msg::Temperature>("temperature", rclcpp::QoS(10), callback);
    }

private:
    std::string format_timestamp(const builtin_interfaces::msg::Time &time) const
    {
        char date[64];
        time_t sec = time.sec;
        std::strftime(date, sizeof(date), "%Y-%m-%d %H:%M:%S", std::localtime(&sec));
        return date;
    }

private:
    rclcpp::Subscription<thermo_msgs::msg::Temperature>::SharedPtr subscription_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ThermoMonitor>());
    rclcpp::shutdown();
    return 0;
}
