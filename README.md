# An Open Source Documentation of NAV2 Stack Integrated on our very friendly neighbourhood DiffBot with a modified ros2_controller for diff-drive 😊
---
## Robot Model (A generic rough differential drive bot)
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
## Drive Controller and Sensor Setup (Simulation)
- A custom ros2_control based diff-drive controller was developed from scratch which is responsible for the drive commands (based on wheel kinematics ofc)
- cmd_vel topic exposer
- Given below is the wheel kinematics equation used for building the controller for the Differential Drive
<div align="center">
  <img src="https://github.com/user-attachments/assets/c5f844a7-2789-4eb5-a41c-c62dba525b27" alt="4wd-kinematics">
</div>

> **Note**: The ros2_control controller can even be used for hardware interface (after you develop the hardware_interface code for it that is  :-) [here is a tutorial explaining this](https://youtu.be/J02jEKawE5U?si=voKkIPAWuf_jeQ7E) ---> this guy explains everything for the hardware part and even has the code for the hardware interface using Arduino Boards....

## WORK---IN---PROGRESS

---
Ignition Gazebo Navigation Simulation......
