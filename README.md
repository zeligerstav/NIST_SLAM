Do all the steps for LIO-SAM first.

# Installation
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

### Install our custom drone model
```
cp -R ~/ros2_ws/src/NIST_SLAM/nist ~/PX4-Autopilot/Tools/simulation/gz/models/
cp -R ~/ros2_ws/src/NIST_SLAM/nist_base ~/PX4-Autopilot/Tools/simulation/gz/models/
```

# Running the Simulation
### Start Gazebo with PX4
```
PX4_SYS_AUTOSTART=4001 PX4_SIM_MODEL=nist PX4_GZ_MODEL_POSE=0,0,2,0,0,0 ~/PX4-Autopilot/build/px4_sitl_default/bin/px4
```

### Start the Gazebo-to-ROS2 bridge
```
~/ros2_ws/src/NIST_SLAM/bridge.sh
```
