#include "vehicle_interfaces/params.h"
#include "vehicle_interfaces/timesync.h"
#include "vehicle_interfaces/msg/wheel_state.hpp"

#define NODE_NAME "timesyncsubtest_0_node"
#define TOPIC_NAME "topic"

class SampleTimeSyncSubscriber : public TimeSyncNode
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
    SampleTimeSyncSubscriber(std::string nodeName, std::string topicName, std::string timeServiceName) : TimeSyncNode(nodeName, timeServiceName, 10000, 2), Node(nodeName)
    {
        this->subscription_ = this->create_subscription<vehicle_interfaces::msg::WheelState>(topicName, 
            10, std::bind(&SampleTimeSyncSubscriber::topic_callback, this, std::placeholders::_1));
    }
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto params = std::make_shared<vehicle_interfaces::GenericParams>("timesyncsubtest_params_node");
    auto timeSyncSub = std::make_shared<SampleTimeSyncSubscriber>(NODE_NAME, TOPIC_NAME, params->timesyncService);
    rclcpp::spin(timeSyncSub);
    std::cerr << "Spin Exit" << std::endl;
    rclcpp::shutdown();
}
