PX4_SYS_AUTOSTART=4001 PX4_GZ_MODEL=nist PX4_GZ_MODEL_POSE=0,0,2,0,0,0 ${px4tg} &
wait 2
bash ~/ros2_ws/src/NIST_SLAM/bridge.sh &
wait 2
ros2 launch lio_sam run.launch.py
