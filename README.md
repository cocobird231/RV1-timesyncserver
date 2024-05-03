*`Established: 2024/05/03`* *`Updated: 2024/05/03`*

## About The Project
The time synchronization service for the robot vehicle ver. 1 project.

The service is used to send the reference time point to the client device. The client device can request the reference time point from the server and calculate the time offset between the client device and the server. The refernce time point is defined as the server time point when the server receives the request from the client device.

**NOTE:** The implementation of communication between the time synchronization server and the client was implemented under `TimeSyncNode` at `vehicle_interfaces/timesync.h`. The client device can easily communicate with the server by inheriting the `TimeSyncNode` class or the derived `VehicleServiceNode` class.

**NOTE:** To enable the funcitons of `TimeSyncNode`, make sure to pass the correct service name to the `TimeSyncNode` constructor. If user want to disable the functions, set the service name to empty string.

For the client device, there are several functions can be called:
- `addTimeSyncCallbackFunc()`: Add the callback function for the time synchronized event.
- `syncTime()`: Request the time synchronization to the server and calculate the time offset between the client device and the server.
- `getTimestamp()`: Get the timestamp of the client device whether the time is synchronized or not.
- `getCorrectDuration()`: Get the time offset between the client device and the server. If time is not synchronized, the function will return the zero duration.
- `getTimestampType()`: Get the timestamp type of the client device whether the time is synchronized or not.

If the `TimeSyncNode` was enabled, the node will create a wait service to wait for the time synchronization service to be ready. If the argument `timeSyncWaitService` set to `true`, the node will wait for the time synchronization service to be ready before the node is ready.

If the wait service is done, the node will create a timer to periodically request the time synchronization to the server. The request period is defined by the `timesyncPeriod_ms` argument under `service.json` file for each node.

**NOTE:** The argument `timesyncAccuracy_ms` defined in the `service.json` file is used to determine the time synchronization accuracy. If the spent time of the time synchronization is greater than the `timesyncAccuracy_ms`, the time synchronization will be failed.

## Getting Started

### Prerequisites
- ROS2 `Foxy` or later (`Humble` recommended)
    Install ROS2 from official website: [ROS2 official website](https://docs.ros.org/en/humble/Installation.html) or simply run the following command to automatically install ROS2:
    ```bash
    curl -fsSL ftp://61.220.23.239/scripts/install-ros2.sh | bash
    ```
    **NOTE:** The script only supports `Foxy` and `Humble` versions depending on the Ubuntu version.
    **NOTE:** The script will create a new workspace at `~/ros2_ws`.
    **NOTE:** The script will create an alias `humble` or `foxy` for global ROS2 environment setup (e.g. `source /opt/ros/<$ROS_DISTRO>/setup.bash`) depending on the ROS2 version.
- [vehicle_interfaces](https://github.com/cocobird231/RV1-vehicle_interfaces.git)

The required packages are listed in the `requirements_apt.txt` file. Install the required packages by running the following command:
```bash
xargs sudo apt install -y < requirements_apt.txt
```
**NOTE:** The required packages will be installed automatically while installing the package using the (`vcu-installer`)[https://github.com/cocobird231/RV1-vcu-install.git].


### Installation
There are two ways to install the package: manually or using the `vcu-installer`. 

#### Install Manually
1. Check if `vehicle_interfaces` package is installed. If not, install the package by following the instructions in the [vehicle_interfaces](https://github.com/cocobird231/RV1-vehicle_interfaces.git).
2. Clone the repository under `~/ros2_ws/src` and rename it to `cpp_timesyncserver`:
    ```bash
    git clone https://github.com/cocobird231/RV1-timesyncserver.git cpp_timesyncserver
    ```
3. Change the directory to the `~/ros2_ws` workspace and build the package:
    ```bash
    # Change directory to workspace.
    cd ~/ros2_ws

    # Source the local environment.
    . install/setup.bash

    # Build the package.
    colcon build --symlink-install --packages-select cpp_timesyncserver
    ```
    **NOTE:** The package is installed in the local workspace.


#### Install Using `vcu-installer`
1. Run the installer and press `Scan` button under Package Management window. If the installer not installed, install the installer by following the instructions in the [`vcu-installer`](https://github.com/cocobird231/RV1-vcu-install.git).

2. Checked the `Timesync Server` checkbox under package list, right-click to modify the internet setting, then press the `Install` button to install the package.

3. The installer will create the start-up script for the package under `/etc/xdg/autostart` directory. The package will be started automatically after the system boot-up.


## Usage
The package contains only one executable `server` for the main time synchronization service.

### Run the Main Service
1. Source the local workspace:
    ```bash
    source ~/ros2_ws/install/setup.bash
    ```

2. Run the main service:
    - Using the `launch`:
        ```bash
        ros2 launch cpp_timesyncserver launch.py
        ```
        **NOTE:** The launch file parsed the `common.yaml` file to set the parameters. The `common.yaml` file is located in the `cpp_timesyncserver/launch` directory.
        **NOTE:** The `common.yaml` file default the namespace to `V0`.

    - Using the `run`:
        ```bash
        ros2 run cpp_timesyncserver server
        ```


## Description

### Communicate with the Service
For the derived class of `TimeSyncNode`, the client device is used to communicate with the server using `TimeSync.srv` which are defined in the `vehicle_interfaces/srv`. 

#### `TimeSync.srv`
The service is used to send client device timestamp to server and receive the server timestamp from the server. The service contains the following fields:
```.srv
# Request field
uint8 request_code # Request status of the client device.
builtin_interfaces/Time request_time # Client device timestamp.

# Response field
uint8 response_code # Response status of the server.
builtin_interfaces/Time request_time # Client device timestamp.
builtin_interfaces/Time response_time # Server timestamp.
```

The request is usually sent from the `syncTime()` function under `TimeSyncNode`, and the time offset is calculated when receiving the response from the server.

**NOTE:** The callback function will only be called by the internal timer of `TimeSyncNode` if the time synchronization is successful. If manually call the `syncTime()` function, the callback function will not be called.


### `common.yaml` File
The `common.yaml` file is currently not used. The service name will be determined by the `timesyncService` under `service.json` file.
