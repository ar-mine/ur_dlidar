# ur_dlidar

## User manual

```
# source global ros(optional)
$ source /opt/ros/<your_ros_version>/setup.bash

# create a catkin workspace
$ mkdir -p catkin_ws/src && cd catkin_ws/src

# Clone repo 
git clone git@github.com:ar-mine/ur_dlidar.git

# Install by using wstool
wstool init .
wstool merge -t . ur_dlidar/ur_dlidar.rosinstall
wstool update -t .

# install dependencies
$ sudo apt update -qq
$ sudo apt dist-upgrade
$ rosdep update
$ rosdep install --from-paths src --ignore-src -y

# build the workspace
$ catkin build

# activate the workspace (ie: source it)
$ source devel/setup.bash
```