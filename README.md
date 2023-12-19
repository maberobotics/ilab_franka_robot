# pandaman
ROS2 packages for the Panda robot


## Getting started
### libfranka
Franka hardware interface is based on `libfranka` that needs to be installed using the following steps:
```shell
    $ sudo apt install build-essential cmake git libpoco-dev libeigen3-dev
    $ git clone --recursive https://github.com/frankaemika/libfranka --branch fr3-develop
    $ cd libfranka
    $ mkdir build
    $ cd build
    $ cmake -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTS=OFF ..
    $ cmake --build .
    $ cpack -G DEB
    $ sudo dpkg -i libfranka*.deb
```
### ROS2
***Required setup : Ubuntu 22.04 LTS***

1.  Install `ros2` packages. The current development is based of `ros2 humble`. Installation steps are described [here](https://docs.ros.org/en/humble/Installation.html).
2. Source your `ros2` environment:
    ```shell
    source /opt/ros/humble/setup.bash
    ```
    **NOTE**: The ros2 environment needs to be sources in every used terminal. If only one distribution of ros2 is used, it can be added to the `~/.bashrc` file.
3. Install `colcon` and its extensions :
    ```shell
    sudo apt install python3-colcon-common-extensions
     ```
3. Create a new ros2 workspace:
    ```shell
    mkdir ~/ros2_ws/src
    ```
4. Pull relevant packages, install dependencies, compile, and source the workspace by using:
    ```shell
    cd ~/ros2_ws
    git clone https://github.com/maberobotics/ilab_franka_robot.git src/ilab_franka_robot
    vcs import src < src/ilab_franka_robot.repos
    rosdep install --ignore-src --from-paths . -y -r
    colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release --symlink-install
    source install/setup.bash
    ```
**NOTE:** The `ilab_franka_robot.repos` file contains links to ros2 packages that need to be source-built to use their newest features.

**NOTE:** If you experience unexpected behavior with the `joint_trajectory_controller`, you are probably shadowing the default one with the custom version provided by `franka_ros2`.


## Usage
The package comes with a general purpose launch file, which can be run using 
```shell
$ ros2 launch ilab_franka_bringup ilab_franka.launch.py
```
Several launch arguments can be used to adapt the launch. These arguments can be shown by running 
```shell
ros2 launch ilab_franka_bringup ilab_franka.launch.py --show-args
```

For example, to start the Franka real robot with Moveit2 planning and Rviz2, run
```shell
ros2 launch ilab_franka_bringup ilab_franka.launch.py use_fake_hardware:=false use_planning:=true start_rviz:=true robot_ip:=172.16.0.2
```