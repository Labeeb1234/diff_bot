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
  <img src="https://media.tenor.com/images/5b5b8a8ebed1e4318af5b726007ad7b8/tenor.gif](https://tenor.com/view/todd-howard-it-just-works-bethesda-this-all-just-works-gif-20598651](https://www.google.com/url?sa=i&url=https%3A%2F%2Fimgur.com%2Fgallery%2Fjust-works-zf7yY6E&psig=AOvVaw2gH32CCQL0tqVvlToir10F&ust=1734968438103000&source=images&cd=vfe&opi=89978449&ved=0CBQQjRxqFwoTCLjOmJ7bu4oDFQAAAAAdAAAAABAE" alt="todd howard - it just works" width="300">
</div>

---
## Base Controller
- A custom ros2_control based diff-drive controller was developed from scratch which is responsible for the drive commands (based on wheel kinematics ofc)
- cmd_vel topic exposer
- 

## WORK---IN---PROGRESS
