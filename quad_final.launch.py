import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, GroupAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory('quad_final')
    world_path = os.path.join(pkg_share, 'worlds', 'world_maze1.sdf')
    models_path = os.path.join(pkg_share, 'models')
    rviz_path = os.path.join(pkg_share, 'rviz', 'quad_view.rviz')

    install_prefix = os.path.dirname(os.path.dirname(pkg_share))
    lib_dir = os.path.join(install_prefix, 'lib', 'quad_final')

    def script(name):
        return os.path.join(lib_dir, name)

    use_teleop = LaunchConfiguration('use_teleop')
    use_dynamic_demo = LaunchConfiguration('use_dynamic_demo')

    gz_sim = ExecuteProcess(
        cmd=['gz', 'sim', '-r', world_path],
        output='screen',
        additional_env={'GZ_SIM_RESOURCE_PATH': models_path},
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/quadcopter/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
            '/quadcopter/enable@std_msgs/msg/Bool@gz.msgs.Boolean',
            '/quadcopter/odometry@nav_msgs/msg/Odometry@gz.msgs.Odometry',
            '/quadcopter/target_position@geometry_msgs/msg/Pose@gz.msgs.Pose',
            '/quadcopter/navsat@sensor_msgs/msg/NavSatFix@gz.msgs.NavSat',
            '/quadcopter/camera/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked',
            '/quadcopter/camera/image@sensor_msgs/msg/Image@gz.msgs.Image',
            '/quadcopter/camera/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo',
            '/quadcopter/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
        ],
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    gps_converter = ExecuteProcess(
        cmd=['python3', script('gps_odom_converter.py'), '--ros-args', '-p', 'use_sim_time:=true'],
        output='screen',
    )
    odom_tf = ExecuteProcess(
        cmd=['python3', script('odom_tf_broadcaster.py'), '--ros-args', '-p', 'use_sim_time:=true'],
        output='screen',
    )
    obstacle_detector = ExecuteProcess(
        cmd=['python3', script('obstacle_detector.py'), '--ros-args', '-p', 'use_sim_time:=true'],
        output='screen',
    )
    obstacle_tracker = ExecuteProcess(
        cmd=['python3', script('obstacle_tracker.py'), '--ros-args', '-p', 'use_sim_time:=true'],
        output='screen',
    )
    planner = ExecuteProcess(
        cmd=['python3', script('path_planner.py'), '--ros-args', '-p', 'use_sim_time:=true'],
        output='screen',
    )
    navigator = ExecuteProcess(
        cmd=['python3', script('go_to_position.py'), '--ros-args', '-p', 'use_sim_time:=true'],
        output='screen',
    )
    goal_bridge = ExecuteProcess(
        cmd=['python3', script('goal_bridge.py'), '--ros-args', '-p', 'use_sim_time:=true'],
        output='screen',
    )

    camera_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=[
            '--x', '0.2', '--y', '0', '--z', '0',
            '--roll', '0', '--pitch', '0', '--yaw', '0',
            '--frame-id', 'quadcopter/base_link',
            '--child-frame-id', 'quadcopter/base_link/front_rgbd_camera',
        ],
        output='screen',
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_path],
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    teleop = GroupAction(
        condition=IfCondition(use_teleop),
        actions=[
            ExecuteProcess(
                cmd=[
                    'gnome-terminal', '--', 'bash', '-c',
                    'python3 ' + script('quad_teleop.py')
                    + '; echo; echo "[teleop exited, press Enter to close]"; read',
                ],
                output='screen',
            )
        ],
    )

    dynamic_demo = GroupAction(
        condition=IfCondition(use_dynamic_demo),
        actions=[
            ExecuteProcess(
                cmd=['python3', script('dynamic_obstacle_demo.py'), '--ros-args', '-p', 'use_sim_time:=true'],
                output='screen',
            )
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_teleop', default_value='false',
            description='Launch keyboard teleop in a separate terminal.'
        ),
        DeclareLaunchArgument(
            'use_dynamic_demo', default_value='false',
            description='Move dynamic_test_box through the maze for testing.'
        ),
        gz_sim,
        bridge,
        gps_converter,
        odom_tf,
        obstacle_detector,
        obstacle_tracker,
        planner,
        navigator,
        goal_bridge,
        camera_tf,
        rviz,
        teleop,
        dynamic_demo,
    ])
