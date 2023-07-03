#include "vehicle_interfaces/vehicle_interfaces.h"
#include "vehicle_interfaces/msg/wheel_state.hpp"

#define NODE_NAME "timesyncsubtest_0_node"
#define TOPIC_NAME "topic"

class SampleTimeSyncSubscriber : public vehicle_interfaces::TimeSyncNode
{
private:
    rclcpp::Subscription<vehicle_interfaces::msg::WheelState>::SharedPtr subscription_;

private:
    void topic_callback(const vehicle_interfaces::msg::WheelState::SharedPtr msg)
    {
        rclcpp::Time nowTime = this->getTimestamp();
        RCLCPP_INFO(this->get_logger(), "I heard: %03d | %05d %05d %05d %05d | %03d %03d", 
            msg->gear, msg->steering, msg->pedal_throttle, msg->pedal_brake, msg->pedal_clutch, 
            msg->button, msg->func);
        
        printf("Publish time:\t%f s\n", msg->header.stamp.sec + msg->header.stamp.nanosec / 1000000000.0);
        printf("Subscribe time:\t%f s\n", nowTime.seconds());
        printf("Transmit time:\t%f ms\n", (nowTime - (rclcpp::Time)(msg->header.stamp)).seconds() * 1000.0);
    }

public:
    SampleTimeSyncSubscriber(const std::shared_ptr<vehicle_interfaces::GenericParams>& gParams) : 
        vehicle_interfaces::TimeSyncNode(NODE_NAME, gParams->timesyncService, gParams->timesyncInterval_ms, gParams->timesyncAccuracy_ms), 
        rclcpp::Node(NODE_NAME)
    {
        this->subscription_ = this->create_subscription<vehicle_interfaces::msg::WheelState>(TOPIC_NAME, 
            10, std::bind(&SampleTimeSyncSubscriber::topic_callback, this, std::placeholders::_1));
    }
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto params = std::make_shared<vehicle_interfaces::GenericParams>("timesyncsubtest_params_node");
    auto timeSyncSub = std::make_shared<SampleTimeSyncSubscriber>(params);
    rclcpp::spin(timeSyncSub);
    std::cerr << "Spin Exit" << std::endl;
    rclcpp::shutdown();
}
