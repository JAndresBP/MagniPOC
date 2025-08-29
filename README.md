# MAGNI POC
<img width="1920" height="1080" alt="image" src="https://github.com/user-attachments/assets/aa4f2c91-4ecf-4997-8ca8-5e0579111072" />

The purpose of this project is to create a custom implementation of the controllers for MAGNI platfomr created by ubiquity robotics (https://www.ubiquityrobotics.com/). The objective is to stablish a ROS2 project that allows to control the MAGNI Robot and perform SLAM, navigation, image recognition enabling the platform for future more complex tasks.

## Summary

- **magni_description**: Adapted ubiquity's URDFs files and using with ROS2 launch file
- **magni_webots**: this package spawn MAGNI robot in Webots and simulate camera lidar sensors
- **video_recorder**: this package is intended to perform video recording of robot cameras

As part of the purpose of this project we want to study how to leverage docker for better development and deployment experiece so we are using devcontainers for development evironemnt setup and github actions for package creation and deployment. 

<img width="3794" height="1042" alt="image" src="https://github.com/user-attachments/assets/cbeafbb2-b91e-4081-80f5-4d45f17b1604" />



