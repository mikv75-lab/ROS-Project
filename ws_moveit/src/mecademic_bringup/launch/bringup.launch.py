# mecademic_bringup/launch/robot_and_servo.launch.py
#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Bessere Logs im selben Terminal
    env = [
        SetEnvironmentVariable('RCUTILS_LOGGING_USE_STDOUT', '1'),
        SetEnvironmentVariable('RCUTILS_COLORIZED_OUTPUT', '1'),
        SetEnvironmentVariable('FASTDDS_SHM_DEFAULT', '0'),
    ]

    moveit_pkg = FindPackageShare('mecademic_moveit_config')
    nodes_pkg   = FindPackageShare('mecademic_nodes_cpp')

    # 1) Robot (URDF + controllers + move_group etc.)
    robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([moveit_pkg, '/launch/robot.launch.py']),
        # Falls dein robot.launch.py Argumente hat, hier durchreichen:
        launch_arguments={
            # 'use_fake_hardware': 'true',
            # 'start_rviz': 'false',
        }.items()
    )

    # 2) Servo – bewusst verzögert nach Robot
    servo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([nodes_pkg, '/launch/servo.launch.py']),
        launch_arguments={
            'servo_mode': 'cartesian',
            'servo_frame': 'tcp',
            # 'cart_lin_mm_s': '100.0',
            # 'cart_ang_deg_s': '30.0',
        }.items()
    )

    #delayed_servo = TimerAction(period=3.0, actions=[servo])

    return LaunchDescription(env + [robot]) #, delayed_servo])
