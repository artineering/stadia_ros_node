
# Stadia ROS2 Package

A ROS2 node that captures input from the Stadia bluetooth controller and publishes to the various ROS2 topics.


## License

[APL](https://choosealicense.com/licenses/apache-2.0/)


## Prerequisites
* ROS2 build environment
    - ROS2 Iron has been installed
    - ROS2 workspace is installed at ~/ros2_ws
    - Source folder is configured at ~/ros2_ws/src
* Python3
* Stadia controller converted to bluetooth
## Roadmap

- Capture input from Stadia Controller : DONE
- Add ROS parameter to specify Stadia bluetooth device name : WIP
- Add ROS parameter to specify type of message to be published: WIP
- Publish sensor_msgs/Joy message : WIP
- Publish geometry_msgs/Twist message : TO DO


## Features

- Detects connection/disconnection of Stadia bluetooth device
- Captures all axes and button inputs



## Build & Deployment

To build this project run

```bash
>> cd ~/ros2_ws/src
>> colcon build --packages-select stadia
```

To deploy this project run

```bash
>> source ~/ros2_ws/install.setup.bash
>> ros2 run stadia stadia_node
```

