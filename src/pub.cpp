#include "vehicle_interfaces/vehicle_interfaces.h"
#include "vehicle_interfaces/msg/wheel_state.hpp"

#define NODE_NAME "timesyncsubtest_0_node"
#define TOPIC_NAME "topic"

using namespace std::chrono_literals;

class SampleTimeSyncPublisher : public vehicle_interfaces::TimeSyncNode
{
private:
    rclcpp::Publisher<vehicle_interfaces::msg::WheelState>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::string nodeName_;
    int cnt;

private:
    void timer_callback()
    {
        auto msg = vehicle_interfaces::msg::WheelState();
        msg.header.device_id = this->nodeName_;
        msg.header.stamp = this->getTimestamp();

        msg.gear = vehicle_interfaces::msg::WheelState::GEAR_NEUTRAL;
        msg.steering = cnt % 512;
        msg.pedal_throttle = cnt % 256;
        msg.pedal_brake = cnt % 128;
        msg.pedal_clutch = cnt % 64;
        msg.button = cnt % 32;
        msg.func = cnt % 16;

        RCLCPP_INFO(this->get_logger(), "Publishing: %d | %d %d %d %d | %d %d", msg.gear, 
            msg.steering, msg.pedal_throttle, msg.pedal_brake, msg.pedal_clutch, msg.button, msg.func);
        this->publisher_->publish(msg);
        this->cnt++;
    }

public:
    SampleTimeSyncPublisher(const std::shared_ptr<vehicle_interfaces::GenericParams>& gParams) : 
        vehicle_interfaces::TimeSyncNode(NODE_NAME, gParams->timesyncService, gParams->timesyncInterval_ms, gParams->timesyncAccuracy_ms), 
        rclcpp::Node(NODE_NAME)
    {
        this->nodeName_ = NODE_NAME;
        this->publisher_ = this->create_publisher<vehicle_interfaces::msg::WheelState>(TOPIC_NAME, 10);
        this->timer_ = this->create_wall_timer(500ms, std::bind(&SampleTimeSyncPublisher::timer_callback, this));
        this->cnt = 0;
    }
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto params = std::make_shared<vehicle_interfaces::GenericParams>("timesyncpubtest_params_node");
    auto timeSyncPub = std::make_shared<SampleTimeSyncPublisher>(params);
    rclcpp::spin(timeSyncPub);
    std::cerr << "Spin Exit" << std::endl;
    rclcpp::shutdown();
}
