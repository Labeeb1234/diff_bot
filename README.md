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
  <img src="https://github.com/user-attachments/assets/0b945339-4390-4472-9d93-8cf83ba6a45e" alt="images">
</div>

> **Note**: This repo is for those starting out on nav2 stack of ROS2 (just use any bot you have but the rest of the process just works)

---
## Simulation Environment
### Gazebo-Classic-Ver
<div align="center">
  <img src="https://github.com/user-attachments/assets/8c5bf891-49dd-4b4c-a5af-27febe81c357" alt="env-gazebo-classic">
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
- Install the following package for SLAM Mapping before starting out, for the source installation go [here](https://docs.nav2.org/tutorials/docs/navigation2_with_slam.html) otherwise use the cmd below for binary installation
``` bash
sudo apt install ros-${DISTRO}-slam-toolbox
```

- **Teleop SLAM**
  - A manual control based SLAM mapping
![teleop_slam](https://github.com/user-attachments/assets/9c42b915-349d-4f29-9cc3-e5bd41bd26b3)

- **Auto SLAM**
  - Method-1 (Using only the base local planners of NAV2)
    - This method involves nothing but just the local planners help in navigating to waypoints in the unknown/unexplored regions so that the map regions within the planned path gets mapped out. Not a very efficient auto SLAM method but it works, given below is a sample demo of the same.

  <div align="center">
    <img src="https://github.com/user-attachments/assets/fe07f899-c0fe-4c2f-b768-753b4aa0b3f2" alt="auto_slam_ver1">
  </div>

  - Method-2 (WaveFrontier Exploration)
    - This method involves the usage of the wavefrontier exploration alogrithm, which way more effective than method-1 as the bot moves into the interface between the explored and unknown regions with the help of wave_frontiers which act as goal poses for the bot's local planner/controller and does the SLAM routine. All creds to this repo and the guy who made it; [m-explore-ros2](https://github.com/robo-friends/m-explore-ros2). All the instructions pertaining to the installation and usage is given in this repo for your refrence, for my implemenation take a look [here](https://github.com/Labeeb1234/diff_bot/tree/main/diff_bot_navigation/launch), the config file has the exploration params and also a separate launch file to launch the exploration node. Also for more info about the working of this alogrithm in mobile robotics application please refer to this [paper-creds to the guys who_wrote_it](https://arxiv.org/pdf/1806.03581).
    
    <div align="center">
      <img src="" alt="auto_slam_ver1">
    </div>
    
- While doing SLAM mapping(Auto/Teleop) make sure to move the bot slowly to get the map updates properly (quite resource intensive; more over make sure the update_rates of the base feedbacks that go to the ekf node are appropriates to ensure less errors between map->odom tf data
- For Auto SLAM just adjust the local planner velocity and acceleration parameters for the SLAM mapping task, like how I did it [here](https://github.com/Labeeb1234/diff_bot/blob/main/diff_bot_navigation/nav_params/auto_slam_nav_params.yaml). Scroll down to the controller section of [this param file](https://github.com/Labeeb1234/diff_bot/blob/main/diff_bot_navigation/nav_params/auto_slam_nav_params.yaml)
- Given below is the portion of the environment mapped out in Gazebo-Classic environment using the SLAM-Toolbox of NAV2
<div align="center">
  <img src="https://github.com/user-attachments/assets/51d03756-9e9b-485b-a1dd-ea5995abc0f6" alt="map2">
</div>

- Map created using Auto SLAM (Method-2)
<div align="center">
  <img src="https://github.com/user-attachments/assets/585180ed-99e7-48fd-aec0-2b2cb0b76859" alt="auto_slam_map">
</div>

- For saving a map either do this:
``` bash
ros2 run nav2_map_server map_saver_cli -f ~/<your_map_name>
```
- or do this:
-> Go to "panels" on RViz and click on "add new panel" and add the thing shown below
<div align="center">
  <img src="https://github.com/user-attachments/assets/52eccbc3-e1bb-4a2f-aae8-a3922129d9cc" alt="slam_toolbox_panel">
</div>
-> After adding the slam tool box panel type in your map name in the save_map and serialize_map section as shown below:
<div align="center">
  <img src="https://github.com/user-attachments/assets/a9671111-04f0-4c03-96f2-24f2910253d1" alt="slam_toolbox_panel">
</div>
-> Then click on save_map after that click on serialize_map(optional as long as .yaml and .pgm files are present its okay) 





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


## ================= WORK---IN---PROGRESS ==================

