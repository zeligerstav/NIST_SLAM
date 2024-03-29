PX4_SYS_AUTOSTART=4001 PX4_SIM_MODEL=nist PX4_GZ_MODEL_POSE=0,0,1,0,0,0 ~/PX4-Autopilot/build/px4_sitl_default/bin/px4 &
~/ros2_ws/src/NIST_SLAM/QGroundControl.AppImage &
sh ~/ros2_ws/src/NIST_SLAM/bridge.sh & 
ros2 launch lio_sam run.launch.py
