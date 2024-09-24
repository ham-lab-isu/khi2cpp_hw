# khi2cpp_hw
Engineers pretending to be software devs

Humble Driver for Khi CX-series robots equipped with KRNX API for programmatic control.

Jakob Hamilton, Iowa State University, 2024

Requires installation of
- colcon: sudo apt install python3-colcon-common-extensions
- rosdep: sudo apt install python3-rosdep2
- ros2_control: sudo apt install ros-humble-ros2-control - build from source?
- ros2 controllers: sudo apt install ros-humble-ros2-controllers - is this still true? Do you need to build from source?
- vcstool: sudo apt install python3-vcstool

## Instructions
1. Configure the controller IP address in ```/khi2cpp_hw/description/ros2_control/cx110l.ros2_control.xacro```, for example
2. Source ROS2 Humble: ```source /opt/ros/humble/setup.bash```
3. Move into your workspace directory: ```cd ~/ros2/khi_jdh```, for example
4. Build the package: ```colcon build --symlink-install```
5. Source the package: ```source install/setup.bash```
6. Launch a robot: ```ros2 launch khi2cpp_hw cx110l_controller.launch.py```, for example