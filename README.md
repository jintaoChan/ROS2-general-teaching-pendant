# ROS 2 General Teaching Pendant

This repository provides a general teaching pendant for robotic arms with different kinematic configurations.  
Just plug it in and play.

---

## Dependencies

- Development platform: **ROS 2**  
- Communication with motor drivers: **EtherCAT**  
- EtherCAT master: **IgH**  
- ROS 2 Control hardware interface wrapper: [ethercat_driver_ros2](https://github.com/ICube-Robotics/ethercat_driver_ros2)  
- UI framework: **Qt**  

---

## Usage

### 1. Install Dependencies
Install all the dependencies listed above.  

---

### 2. Generate a MoveIt 2 Configuration
You need a MoveIt 2 configuration package for your robot.  
Here is a [demo configuration](https://github.com/jintaoChan/ROS2-general-teaching-pendant/tree/dev/src/auto_store_robot_moveit_config).  

#### Notes
a. **Set acceleration and deceleration limits** in `joint_limits.yaml`, for example:
```yaml
joint0:
  has_velocity_limits: true
  max_velocity: 0.2
  has_acceleration_limits: true
  max_acceleration: 0.05
  has_deceleration_limits: true
  max_deceleration: 0.5
```


b. Add a custom launch file (e.g., launch.py) in the launch/ directory.
This file is used to start your custom controller.

c. Configure motor driver parameters in your YAML files.
For details, refer to ethercat_driver_ros2.

### 3. Run

```cmd
ros2 launch auto_store_robot_moveit_config launch.py   # replace with your own MoveIt config package
ros2 run auto_store_ui auto_store_ui
```