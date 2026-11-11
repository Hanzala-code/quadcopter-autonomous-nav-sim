https://github.com/user-attachments/assets/55fb3e4f-cc38-4589-84d8-67e12df87e1a

# quadcopter-autonomous-nav-sim
# Autonomous Quadcopter Simulation (ROS 2 Jazzy + Gazebo Harmonic)

Point-to-point autonomous navigation with obstacle detection and replanning for a PX4 x500-style quadcopter in a 20×20 m arena.







<img width="1536" height="1024" alt="8ad5ab6b-8ff9-401b-8dcb-157e4da98c54" src="https://github.com/user-attachments/assets/f32a211c-3cb0-4121-9557-f99c7602596f" />



## Features
- GPS (NavSat) → local odometry conversion, dynamic TF (`world → base_link → camera`)
- Front RGBD camera → point cloud → voxel clustering → 3D obstacle bounding boxes
- D* Lite planner (A* for fresh searches) on a 0.5 m grid with inflation margin
- Event-driven replanning (only when obstacles block the current path)
- P-controller waypoint follower, smooth keyboard teleop with emergency brake
- RViz2 visualization + Gazebo GUI panels

## Stack
Ubuntu 24.04 · ROS 2 Jazzy · Gazebo Harmonic · `ros_gz_bridge`

## Build & run
​```bash
cd ~/program_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select quad_sim4
source install/setup.bash
ros2 launch quad_sim4 quad_sim4_launch.py
​```
Set a target via the Gazebo "Set Target Position" panel (`/quadcopter/target_position`, `gz.msgs.Pose`).

## Tip for WiFi drops
​```bash
export GZ_IP=127.0.0.1
export ROS_LOCALHOST_ONLY=1
​```

## Nodes
| Node | Role |
|---|---|
| `gps_odom_converter` | NavSatFix → Odometry |
| `odom_tf_broadcaster` | world → base_link TF |
| `obstacle_detector` | Point cloud → obstacle markers |
| `path_planner` | D* Lite / A* planning |
| `go_to_position` | Waypoint-following P-controller |
| `quad_teleop` | Manual override |
