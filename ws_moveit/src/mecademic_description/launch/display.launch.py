from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution, TextSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    use_tool1_arg = DeclareLaunchArgument(
    name="use_tool1",
    default_value="false",
    description="Tool 1 aktivieren"
    )
    use_tool2_arg = DeclareLaunchArgument(
        name="use_tool2",
        default_value="false",
        description="Tool 2 aktivieren"
    )

    world_frame = "world"
    base_frame  = "meca_mount"

    urdf_xacro = PathJoinSubstitution([
        FindPackageShare("mecademic_description"),
        "urdf",
        "meca_500_r3_with_tool.xacro"
    ])

    rviz_config = PathJoinSubstitution([
        FindPackageShare("mecademic_description"),
        "rviz",
        "meca_display.rviz"
    ])

    def launch_setup(context):
        use_tool1 = LaunchConfiguration("use_tool1").perform(context)
        use_tool2 = LaunchConfiguration("use_tool2").perform(context)

        robot_description = {
            "robot_description": Command([
                "xacro", " ",
                urdf_xacro, " ",
                "use_tool1:=" + use_tool1, " ",
                "use_tool2:=" + use_tool2
            ])
        }

        rsp = Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[robot_description],
            output="screen"
        )

        jsp = Node(
            package="joint_state_publisher_gui",
            executable="joint_state_publisher_gui",
            output="screen"
        )

        rviz = Node(
            package="rviz2",
            executable="rviz2",
            arguments=["-d", rviz_config],
            output="screen"
        )

        static_tf = Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='tf_world_to_mecamount',
            arguments=[
                "--x", "0", "--y", "0", "--z", "0",
                "--qx", "0", "--qy", "0", "--qz", "0", "--qw", "1",
                "--frame-id", world_frame,
                "--child-frame-id", base_frame
            ]
        )

        return [static_tf, rsp, jsp, rviz]

    return LaunchDescription([
        SetEnvironmentVariable("XDG_RUNTIME_DIR", "/tmp/runtime-root"),
        use_tool1_arg,
        use_tool2_arg,
        OpaqueFunction(function=launch_setup),
    ])