on konsole:

docker run -it --rm --privileged --network=host -v /dev:/dev -v /run/udev:/run/udev:ro -v /home/admin/ros2_ws:/root/ros2_ws my_robot

Usage inside Docker:
ros2 launch robot_bringup robot.launch.py

on another terminal
ros2 run joy joy_node
