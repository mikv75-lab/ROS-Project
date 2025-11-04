from launch import LaunchDescription
from launch.actions import OpaqueFunction
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    urdf_xacro = PathJoinSubstitution([
        FindPackageShare("omron_viper_description"),
        "urdf",
        "omron_viper.urdf.xacro",
    ])
    rviz_config = PathJoinSubstitution([
        FindPackageShare("omron_viper_description"),
        "rviz",
        "omron_display.rviz",
    ])

    def launch_setup(context):
        robot_description = {
            "robot_description": ParameterValue(
                Command(["xacro ", urdf_xacro]),
                value_type=str,
            )
        }

        rsp = Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[robot_description],
            output="screen",
        )

        # >>> nur die GUI <<<
        jsp_gui = Node(
            package="joint_state_publisher_gui",
            executable="joint_state_publisher_gui",
            parameters=[robot_description],
            output="screen",
        )

        rviz = Node(
            package="rviz2",
            executable="rviz2",
            arguments=["-d", rviz_config],
            output="screen",
        )

        static_tf = Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='tf_world_to_robot_mount',
            arguments=[
                "--x","0","--y","0","--z","0",
                "--qx","0","--qy","0","--qz","0","--qw","1",
                "--frame-id","world",
                "--child-frame-id","world_mount",
            ],
        )
        return [static_tf, rsp, jsp_gui, rviz]

    return LaunchDescription([OpaqueFunction(function=launch_setup)])
