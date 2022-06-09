# micro-ROS Agent

Guide on how to set up and use the micro-ROS Agent to connect micro-ROS nodes with the ROS2 network.

## How it works

## Setup

The Agent can be set up in either ROS2 or Docker. With ROS2 it's compatible with launch files and doesn't struggle with permissions, while Docker doesn't take as much time to set up and is easier to start and test with.

### ROS2
Info based on [micro_ros_setup](https://github.com/micro-ROS/micro_ros_setup/blob/galactic/README.md).

1. Clone the `micro_ros_setup` repository inside `src` of your ROS2 workspace:
    ```bash
    git clone -b $ROS_DISTRO https://github.com/micro-ROS/micro_ros_setup.git
    ```
2. Build the workspace with `colcon`.
3. Run the following commands to download and build the agent:
    ```bash
    ros2 run micro_ros_setup create_agent_ws.sh
    ros2 run micro_ros_setup build_agent.sh
    ```
4. You can now run the agent with, for example, either of the following commands:
    ```bash
    ros2 run micro_ros_agent micro_ros_agent serial --dev [device]
    ros2 run micro_ros_agent micro_ros_agent udp4 --port [port]
    ```
    See [example](ROS2/agent_launch.py) of how to start the Agent with a launch file.

    4.1. Add a `-v6` flag to see the output from the Agent.

### Docker

1. Install [Docker](https://docs.docker.com/engine/install/).
    
    1.1 Linux users need to also configure for [non-root users](https://docs.docker.com/engine/install/linux-postinstall/#manage-docker-as-a-non-root-user).
2. You can now run the Agent with, for example, either of the following commands:
    ```
    docker run -it --rm --net=host microros/micro-ros-agent:galactic serial --dev /dev/ttyUSB0 -v6
    docker run -it --rm --net=host microros/micro-ros-agent:galactic udp4 --port 8888 -v6
    ```

### Troubleshoot

TODO