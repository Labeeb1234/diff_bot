# An Open Source Documentation of NAV2 Stack Integrated on our very friendly neighbourhood DiffBot with a modified ros2_controller for diff-drive 😊
---
## Robot Model (A generic rough differential drive bot)
<div align="center">
  <img src="https://github.com/user-attachments/assets/8b03e003-577b-4788-ba45-bcdd59a77109" alt="BOT Under Question">
</div>
- 4WD-Bot(Differential Drive); pic above
- Kinematics of a 2-wheeled diff drive
- sensors:
  ---> wheel encoders
  ---> IMU
  ---> LIDAR

<div align="center">
  <img src="https://github.com/user-attachments/assets/6d543a61-8a11-4238-b966-e311904907d4" alt="images" width="640" height="480">
</div>

> **Note**: This repo is for those starting out on nav2 stack of ROS2 (just use any bot you have but the rest of the process just works)

---
## Simulation Environment
### Gazebo-Classic-Ver
<div align="center">
  <img src="https://github.com/user-attachments/assets/7b658de1-af24-4b73-9f14-9ce09ed224ce" alt="env-gazebo-classic">
</div>


### Gazebo-Ignition-Ver
<div align="center">
  <img src="https://github.com/user-attachments/assets/4730a15e-e183-4d31-8bc8-f434fcda8710" alt="env-gazebo">
</div>


---

---
## Drive Controller and Sensor Setup (Simulation)
- A custom ros2_control based diff-drive controller was developed from scratch which is responsible for the drive commands (based on wheel kinematics ofc)
- As of now the diff_bot_controller is just for exposing the cmd_vel topic to control the motion of the bot; haven't integrated any innate odometery publisher or tf-broadcasters yet (which can be done easily with kinematics/wheel_encoder data)
- Given below is the wheel kinematics equation used for building the controller for the Differential Drive
<div align="center">
  <img src="https://github.com/user-attachments/assets/c5f844a7-2789-4eb5-a41c-c62dba525b27" alt="4wd-kinematics">
</div>

> **Note**: The ros2_control controller can even be used for hardware interface (after you develop the hardware_interface code for it that is  :-) [here is a tutorial explaining this](https://youtu.be/J02jEKawE5U?si=voKkIPAWuf_jeQ7E) ---> this guy explains everything for the hardware part and even has the code for the hardware interface using Arduino Boards....

- The imu sensor for simulation was taken from the default [imu_ros_sensor plugin](https://github.com/ros-simulation/gazebo_ros_pkgs/blob/noetic-devel/gazebo_plugins/src/gazebo_ros_imu.cpp) for ROS2 version (Gazebo-Classic [EOL soon]). For the hardware we used an MPU9250 fusing the Mag and Gyro data to get the yaw and yaw_rates
- The lidar sensor for simulation was taken from the default [ray_ros_sensor plugin](https://github.com/ros-simulation/gazebo_ros_pkgs/blob/ros2/gazebo_plugins/src/gazebo_ros_ray_sensor.cpp). For the hardware we used the rplidar A2M8 lidar for the laser scan data; SDK used: [ros2 rplidar wrapper](https://github.com/CreedyNZ/rplidar_ros2)
- Optionally I also used the depth cam to simulate depth data, using a custom [realsense_cam_plugin](https://github.com/Labeeb1234/e-YRC-Hackathon/blob/main/realsense_gazebo_plugin/README.md). For the hardware we used an Intel Realsense D435i device to get the pointcloud data to generate the "stvl_layer" for the nav2 stack

> **Note**: The hardware documentation is not completely included in this one, I was just mentioning them alongside the simulated sensors used for this project

---
## NAV2-Stack Integration
### SLAM operation
- **Teleop SLAM**
- **Auto SLAM**

---
## Ignition Gazebo/ GZ_SIM
### Handling The Bare Basics to start with ROS2-GZ
---
- For the ignition gazebo/gz_sim setup of the same setup used for gazebo-classic few key changes needs to be made in the launch file as well as the bot model URDF file
- Before diving in first install the compatible GZ simulator in your system, preferrably a stable version corresponding to the ROS2 distro installed. A guide for the same is [given here](https://gazebosim.org/docs/all/getstarted/). Read through the installation guide and install the simulator
- Now install the ROS2 dependencies and plugins and bridge nodes corresponding to your configuration
- [Ignition Basic Tutorial/GZ Tutorial](https://gazebosim.org/docs/fortress/tutorials/)
- [Latest Tutorials](https://gazebosim.org/docs/jetty/tutorials/)
- Go through the ROS2 integration tutorials mainly to understand about the bridge and ROS2 Gazebo interoperability
- Trying to launch GZ worlds using ROS2 launch files: [my launch file here](https://github.com/Labeeb1234/diff_bot/blob/main/diff_bot_description/launch/world.launch.py). Make sure the world file formats are in ".sdf" mainly

> **Note**: For detailed and similar launch as that of Gazebo-Classic add the server and client nodes given below:
``` python
gzserver_cmd = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
        os.path.join(ros_gz_sim, 'launch', 'gz_sim.launch.py')
    ),
    launch_arguments={'gz_args': ['-r -s -v4 ', f'{world_file_path}.sdf], 'on_exit_shutdown': 'true'}.items()
)

gzclient_cmd = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
        os.path.join(ros_gz_sim, 'launch', 'gz_sim.launch.py')
    ),
    launch_arguments={'gz_args': '-g -v4 '}.items()
)
```
>
> **Note**: This tutorial has used the Fortress Version of GZ simulator which may or may not have the composition features of using GZ, ROS2 and the ros_gz_bridge (I haven't exactly confirmed this but just learn the new tutorials as it has everything including the older features hence making the usage of GZ sim easier)  

### URDF Spawning via ROS2
---
- "create" node of the ros_gz_sim package is used for the urdf spawning in GZ simulator using launch files, given below are the lines required to launch the said node
``` python
    urdf_spawn_node = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=['-string', robot_doc.toxml(),
                   '-name', 'diff_bot',
                   '-allow_renaming', 'true'
        ]
    )
```
- The entire launch file to launch a model from URDF using launch files is given [here](https://github.com/Labeeb1234/diff_bot/blob/main/diff_bot_description/launch/gz.launch.py)

---

### Bridging GZ/Ign Data to ROS2
---


## ==================== WORK---IN---PROGRESS ====================

