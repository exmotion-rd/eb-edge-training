#include <chrono>
#include <random>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/header.hpp"
#include "thermo_msgs/msg/temperature.hpp"

using namespace std::chrono_literals;

class Sensor
{
private:
    constexpr static double MIN = -5;
    constexpr static double MAX = 50;

    std::random_device rd;
    std::mt19937 eng;
    std::uniform_real_distribution<double> dist;

public:
    Sensor()
        : dist{MIN, MAX}
    {
        eng.seed(rd());
    }

    double value()
    {
        return dist(eng);
    }
};

class ThermoMonitor : public rclcpp::Node
{
public:
    ThermoMonitor()
        : Node("thermo_cpp_meter"), sensor()
    {
        publisher_ = this->create_publisher<thermo_msgs::msg::Temperature>("temperature", 10);

        auto timer_callback = [this]() -> void
        {
            auto message = thermo_msgs::msg::Temperature();
            message.header.stamp = get_clock()->now();
            message.temperature = sensor.value();
            publisher_->publish(message);
        };
        timer_ = this->create_wall_timer(1000ms, timer_callback);
    }

private:
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<thermo_msgs::msg::Temperature>::SharedPtr publisher_;
    Sensor sensor;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ThermoMonitor>());
    rclcpp::shutdown();
    return 0;
}
