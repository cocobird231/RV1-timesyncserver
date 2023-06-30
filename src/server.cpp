#include "vehicle_interfaces/params.h"
#include "vehicle_interfaces/timesync.h"


int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto params = std::make_shared<vehicle_interfaces::GenericParams>("timesyncserver_params_node");
    auto server = std::make_shared<vehicle_interfaces::TimeSyncServer>(params->nodeName, params->timesyncService);
    rclcpp::spin(server);
    std::cerr << "Spin Exit" << std::endl;
    rclcpp::shutdown();
}
