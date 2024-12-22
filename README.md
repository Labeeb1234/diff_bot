# An Open Source Documentation of NAV2 Stack Integrated on our very friendly neighbourhood DiffBot with a modified ros2_controller for diff-drive 😊
---
## Robot Model (A generic rough differential drive bot)
- Kinematics of a 2-wheeled diff drive
- sensors:
  ---> wheel encoders
  ---> IMU
  ---> LIDAR

> **Note**: I get it the bot model is bad and needs improvements, this repo is for those starting out (just use any bot you have but the rest of the process just works)
<div align="center">
  <img src="(https://www.google.com/url?sa=i&url=https%3A%2F%2Ftenor.com%2Fview%2Ftodd-howard-it-just-works-bethesda-this-all-just-works-gif-20598651&psig=AOvVaw29puXEf6Nxwy_3y0Hxg_O1&ust=1734968129675000&source=images&cd=vfe&opi=89978449&ved=0CBQQjRxqFwoTCJDyiozau4oDFQAAAAAdAAAAABAE)" alt="todd howard" width="300">
</div>

---
## Base Controller
- A custom ros2_control based diff-drive controller was developed from scratch which is responsible for the drive commands (based on wheel kinematics ofc)
- cmd_vel topic exposer
- 

## WORK---IN---PROGRESS
