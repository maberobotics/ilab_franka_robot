# pandaman
ROS2 packages for the Panda robot


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

### Running with Gazebo
In order for Gazebo to find the robot model from the `pandaman` stack it needs to be referenced in the `GAZEBO_MODEL_PATH` environment variable. To do so, run:
```shell
$ source /usr/share/gazebo/setup.sh
$ export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/path/to/pandaman
```
**NOTE**: If you encounter issues with spawning the robot to Gazebo making it crash, make sure your models are well referenced.