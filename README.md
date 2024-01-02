
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
>> source ~/ros2_ws/install/setup.bash
>> ros2 run stadia stadia_node
```

## Screenshots
### Startup Screen
![image](https://github.com/artineering/stadia_ros_node/assets/5471319/de06fd0e-8032-42f6-8b33-47623512db45)

### Stadia controller is connected
![image](https://github.com/artineering/stadia_ros_node/assets/5471319/d3a50d8c-7864-439b-b6d8-59eedc6786f0)

### Interacting with the controller
#### Buttons
![image](https://github.com/artineering/stadia_ros_node/assets/5471319/5dc4f10f-e04a-4b5e-ae3d-4ba4fae086fc)

#### Axes data
![image](https://github.com/artineering/stadia_ros_node/assets/5471319/2167cd74-7aa5-4ccb-a1cc-4424ce0251a0)

### Device disconnection
![image](https://github.com/artineering/stadia_ros_node/assets/5471319/9708d5f1-4895-4c3c-a26b-51c39b5dcf9a)

