## ROS2 Workspace

> Note: Using Galactic currently

### Installation
```bash
sudo apt install ros-galactic-gazebo-*

sudo apt install ros-galactic-cartographer
sudo apt install ros-galactic-cartographer-ros

sudo apt install ros-galactic-navigation2
sudo apt install ros-galactic-nav2-bringup

sudo apt install ros-galactic-dynamixel-sdk
sudo apt install ros-galactic-turtlebot3-msgs
sudo apt install ros-galactic-turtlebot3
```

```bash
cd src
git clone -b galactic-devel https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git
cd ..
colcon build --symlink-install
. install/local_setup.bash
. install/setup.bash

export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_gazebo empty_world.launch.py
```

## Wall Following

```bash
source install/setup.bash

ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
ros2 run my_navigation_pkg wall_follow_node
```

## Navigation

```bash
## Map ENV
ros2 launch turtlebot3_cartographer cartographer.launch.py use_sim_time:=true

## Start Navigation
ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=true

## Save Map
ros2 run nav2_map_server map_saver_cli -f ./map/
```