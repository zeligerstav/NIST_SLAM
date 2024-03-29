Do all the steps for LIO-SAM first.

# Installation

### Dependencies
  ```
  sudo apt install ros-humble-perception-pcl \
		   ros-humble-pcl-msgs \
		   ros-humble-vision-opencv \
		   ros-humble-xacro \
		   ros-humble-octomap \
		   ros-humble-octomap-server \
       ros-humble-octomap-rviz-plugins
  ```
  ```
  sudo snap install micro-xrce-dds-agent --edge
  ```

### PX4 Toolchain 
https://docs.px4.io/main/en/dev_setup/dev_env_linux_ubuntu.html#simulation-and-nuttx-pixhawk-targets
  ```
  cd ~/
  git clone https://github.com/PX4/PX4-Autopilot.git --recursive
  cd PX4-Autopilot
  git checkout bb53781
  bash ./Tools/setup/ubuntu.sh
  ```
Now, reboot your machine. AFTER the reboot:
```
cd ~/PX4-Autopilot
make
```
You will need to type 'y' and hit enter several times.

### Checkout sim branch
  ```
  cd ~/ros2_ws/src/NIST_SLAM
  git checkout sim
  ```

### Install rosdep
```
sudo apt install -y python3-rosdep
```

### Install PX4 Messages
```
cd ~/ros2_ws/src
git clone https://github.com/PX4/px4_msgs.git
cd ~/ros2_ws
colcon build --packages-select px4_msgs
```

### Install ros_gz bridge
```
cd ~/ros2_ws/src
git clone https://github.com/gazebosim/ros_gz.git -b humble
export GZ_VERSION=garden
cd ~/ros2_ws
rosdep init
rosdep update
rosdep install -r --from-paths src -i -y --rosdistro humble
colcon build
```

### Install our custom drone model and world
```
cp -R ~/ros2_ws/src/NIST_SLAM/nist ~/PX4-Autopilot/Tools/simulation/gz/models/
cp -R ~/ros2_ws/src/NIST_SLAM/nist_base ~/PX4-Autopilot/Tools/simulation/gz/models/
cp ~/ros2_ws/src/NIST_SLAM/worlds/default.sdf ~/PX4-Autopilot/Tools/simulation/gz/worlds/default.sdf
```

# Running the Simulation
```
bash ~/ros2_ws/src/NIST_SLAM/sim.bash
```
