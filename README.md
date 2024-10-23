# khi2cpp_hw
Engineers pretending to be software devs

Humble Driver for Khi CX-series robots equipped with KRNX API for programmatic control.

Jakob Hamilton, Iowa State University, 2024

Requires installation of
- colcon: ```sudo apt install python3-colcon-common-extensions```
- rosdep: ```sudo apt install python3-rosdep2```
- BUILD FROM SOURCE ros2_controllers: 
## Steps to install ros2_controllers:
```bash
    pip uninstall em 
    pip uninstall empy 
    pip install empy==3.3.4 
    pip install lark --prefer-binary 
    mkdir -p ros2_controllers_ws/src  
    cd ros2_controllers_ws/src  
    git clone https://github.com/ros-controls/ros2_controllers.git -b humble 
    vcs import < ros2_controllers/ros2_controllers.humble.repos 
    cd ~/ros2_controllers_ws 
    sudo apt update  
    rosdep update --rosdistro humble 
    rosdep install --from-paths src --ignore-src -iry 
    source /opt/ros/humble/setup.bash
    colcon build --symlink-install
```
- vcstool: ```sudo apt install python3-vcstool```
- khi2cpp_hw_description from source -> move into the workspace/src folder (```cd ~/ros2/khi_jdh``` for example) and ```git clone git@github.com:ham-lab-isu/khi2cpp_hw_description.git```

## Instructions
1. Configure the controller IP address in ```/khi2cpp_hw/description/ros2_control/cx110l.ros2_control.xacro```, for example
2. Source ROS2 Humble: ```source /opt/ros/humble/setup.bash```
3. Move into your workspace directory: ```cd ~/ros2/khi_jdh```, for example
4. Build the package: ```colcon build --symlink-install```
5. Source the package: ```source install/setup.bash```
6. Launch a robot: ```ros2 launch khi2cpp_hw cx110l_controller.launch.py```, for example

## How it works
The khi2cpp_hw package creates ros2_control nodes for simulating the Kawasaki CX-series robots in RViz and sending movement commands via the Kawasaki KRNX driver. Two interfaces are initiated when launching a controller, e.g., ```ros2 launch khi2cpp_hw cx110l_controller.launch.py```:
- A derived controller_interface::ControllerInterface called **KhiController**. The class and its members are defined in ```khi_cnt_interface.h``` and ```khi_cnt_interface.cpp```.
- A derived hardware_interface::SystemInterface called **KhiSystem**. The class and its members are defined in ```khi_hw_interface.h``` and ```khi_hw_interface.cpp```.

The **KhiController** object directly interfaces with ROS2. Specifically, it creates a virtual robot controller (read: a topic with nested topics for sending and observing robot states/commands). This controller adds functionality for getting and setting movement and system state values in memory to be accessed by the hardware side of the package, i.e., **KhiSystem**. The getter and setter members are loaned from **KhiSystem** and named ```state_interface``` and ```command_interface```, respectively. These allow for position and velocity values to be sent to or read from the hardware.

The **KhiSystem** object bridges the ROS2 side of the system, i.e., the **KhiController**, with the KRNX C++ library found in ```/khi2cpp_hw/lib/``` and declared in ```krnx.h```. The **KhiSystem** has member functions to interact with the KRNX driver (```krnx_robot_driver.h``` and ```krnx_robot_driver.cpp```) including:
- ```on_init()```
- ```export_state_interfaces()```
- ```export_command_interfaces()```
- ```read()```
- ```write()```
- ```close()```
- ```hold()```
- ```update()```
- ```getPeriodDiff()```
- ```getStateName()```

These member functions perform a variety of operations to either get or set values within the state and command interfaces for joint positions and velocities. ```read()``` and ```write()``` perform the bulk of the heavy lifting, directly reading or writing the joint positions and velocities to the KRNX robot driver. The member variables for communicating the state and command for joint position and velocities with **KhiController** are vectors of doubles named ```joint_positions_``` and ```joint_velocities_```. The member variables for communicating motion data with the KRNX driver are ```data_``` and ```driver_```. ```data_``` is a KhiRobotData object that is the carrier for current robot information and is shared within the KRNX driver. ```driver_``` is the KhiRobotKRNXDriver object that calls the member functions for opening, closing, holding, etc. These members call the requisite functions within ```krnx_robot_driver.cpp``` to communicate with the KRNX library and the physical robot controller.
