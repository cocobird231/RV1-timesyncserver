#include "vehicle_interfaces/timesync.h"
#include "vehicle_interfaces/msg/wheel_state.hpp"

#define TOPIC_NAME "topic"
#define TIME_SERVICE_NAME "timesync_0"

using namespace std::chrono_literals;

class SampleTimeSyncPublisher : public TimeSyncNode
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
    SampleTimeSyncPublisher(std::string nodeName, 
                            std::string topicName, 
                            std::string timeServiceName, 
                            double syncInterval_ms, 
                            double syncAccuracy) : TimeSyncNode(nodeName, 
                                                                timeServiceName, 
                                                                syncInterval_ms, 
                                                                syncAccuracy), Node(nodeName)
    {
        this->nodeName_ = nodeName;
        this->publisher_ = this->create_publisher<vehicle_interfaces::msg::WheelState>(topicName, 10);
        this->timer_ = this->create_wall_timer(500ms, std::bind(&SampleTimeSyncPublisher::timer_callback, this));
        this->cnt = 0;
    }
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto timeSyncPub = std::make_shared<SampleTimeSyncPublisher>("timesync_publisher", TOPIC_NAME, TIME_SERVICE_NAME, 10000.0, 0.3);
    rclcpp::spin(timeSyncPub);
    std::cerr << "Spin Exit" << std::endl;
    rclcpp::shutdown();
}