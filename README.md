# Goldfish-Piloted Navigation (GPN)

## Description
  
The purpose of this repo is to develop a system that allows a goldfish to pilot an [iRobot Create 3](https://edu.irobot.com/what-we-offer/create3) using the exposed ROS2 Galactic [interface](https://iroboteducation.github.io/create3_docs/api/ros2/). At a high level, this system works by using computer vision/image processing techniques to monitor and estimate the position of a goldfish within a fish bowl mounted to the [Create 3](https://edu.irobot.com/what-we-offer/create3) via a camera positioned directly above it. The continuously updating position estimates are then used to set both the speed and heading of the [Create 3](https://edu.irobot.com/what-we-offer/create3), with the exception of when obstacles or hazards are detected.
  
## Dependencies

- [ros2 galactic](https://docs.ros.org/en/galactic/Installation/Ubuntu-Install-Debians.html)
- [create3_sim](https://github.com/iRobotEducation/create3_sim/tree/galactic)
- [aws-robomaker-small-house-world](https://github.com/aws-robotics/aws-robomaker-small-house-world/tree/ros2)

## Launch Sim

To launch empty world sim:
```bash
ros2 launch irobot_create_gazebo_bringup create3_gazebo.launch.py
```

To launch aws house sim:
```bash
ros2 launch irobot_create_gazebo_bringup create3_gazebo_aws_small.launch.py
```

## References
  
This project was inspired by [this](https://doi.org/10.1016/j.bbr.2021.113711) similar "Fish Operated Vehicle" built for a research project at Ben-Gurion University. Cited as:  
  
  **Givon, Shachar et al**. *From fish out of water to new insights on navigation mechanisms in animals*. Behavioural Brain Research 419 (2022). https://doi.org/10.1016/j.bbr.2021.113711.
