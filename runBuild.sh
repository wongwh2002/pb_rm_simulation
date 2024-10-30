colcon build --symlink-install --packages-select small_ramp
source ~/.bashrc
ros2 launch small_ramp bringup_sim.launch.py
