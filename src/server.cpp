#include "vehicle_interfaces/params.h"
#include "vehicle_interfaces/timesync.h"

class Params : public vehicle_interfaces::GenericParams
{
public:
    bool UTCStatus = false;

private:
    void _getParams()
    {
        this->get_parameter("UTCStatus", this->UTCStatus);
    }

public:
    Params(std::string nodeName) : vehicle_interfaces::GenericParams(nodeName)
    {
        this->declare_parameter<bool>("UTCStatus", this->UTCStatus);
        this->_getParams();
    }
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto params = std::make_shared<Params>("timesyncserver_params_node");
    auto server = std::make_shared<vehicle_interfaces::TimeSyncServer>(params->nodeName, params->timesyncService);
    server->setUTCSyncStatus(params->UTCStatus);
    rclcpp::spin(server);
    std::cerr << "Spin Exit" << std::endl;
    rclcpp::shutdown();
}
