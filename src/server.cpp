#include "vehicle_interfaces/timesync.h"
#define TIME_SERVICE_NAME "timeService"

class Params : public rclcpp::Node
{
public:
    std::string nodeName = "timesyncserver_0_node";
    std::string qosService = "qos_0";
    std::string safetyService = "safety_0";
    std::string timesyncService = "timesync_0";

private:
    void _getParams()
    {
        this->get_parameter("nodeName", this->nodeName);
        this->get_parameter("qosService", this->qosService);
        this->get_parameter("safetyService", this->safetyService);
        this->get_parameter("timesyncService", this->timesyncService);
    }

public:
    Params(std::string nodeName) : Node(nodeName)
    {
        this->declare_parameter<std::string>("nodeName", this->nodeName);
        this->declare_parameter<std::string>("qosService", this->qosService);
        this->declare_parameter<std::string>("safetyService", this->safetyService);
        this->declare_parameter<std::string>("timesyncService", this->timesyncService);
        this->_getParams();
    }
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto params = std::make_shared<Params>("timesyncserver_params_node");
    auto server = std::make_shared<TimeSyncServer>(params->nodeName, params->timesyncService);
    rclcpp::spin(server);
    std::cerr << "Spin Exit" << std::endl;
    rclcpp::shutdown();
}