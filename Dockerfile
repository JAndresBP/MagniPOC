FROM arm64v8/ros:jazzy

RUN apt-get update && apt-get upgrade -y

RUN apt-get install -y python3-pip \
    ros-jazzy-ros2-control \
    ros-jazzy-ros2-controllers \
    ros-jazzy-nav2-bringup \
    ros-jazzy-cartographer-ros

RUN mkdir -p /home/ws/src
WORKDIR /home/ws

COPY ./src/ubiquity_motor_ros2 /home/ws/src/ubiquity_motor_ros2
COPY ./src/magni_description /home/ws/src/magni_description

RUN . /opt/ros/jazzy/setup.sh && \ 
    colcon build --base-paths \
    src/ubiquity_motor_ros2 \
    src/ubiquity_motor_ros2/ubiquity_motor_ros2_msgs \
    src/magni_description \
    --cmake-args --event-handlers console_direct+

RUN sed --in-place --expression \
	'$isource "/home/ws/install/setup.bash"' \
	/ros_entrypoint.sh

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["ros2", "launch", "ubiquity_motor_ros2", "motors.launch.py"]
