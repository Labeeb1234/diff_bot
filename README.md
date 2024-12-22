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
  <img src="https://media.tenor.com/images/5b5b8a8ebed1e4318af5b726007ad7b8/tenor.gif" alt="todd howard - it just works" width="300">
</div>

---
## Base Controller
- A custom ros2_control based diff-drive controller was developed from scratch which is responsible for the drive commands (based on wheel kinematics ofc)
- cmd_vel topic exposer
- 

## WORK---IN---PROGRESS
