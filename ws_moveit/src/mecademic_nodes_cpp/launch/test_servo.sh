export FASTDDS_SHM_DEFAULT=0

SRV=/servo/switch_command_type
JOINT=0
TWIST=1

clear

# === FRAME: tool_mount ===
ros2 topic pub -1 /servo/set_frame std_msgs/msg/String "{data: tool_mount}"

# JOINT mode setzen (MoveIt Servo!)
ros2 service call $SRV moveit_msgs/srv/ServoCommandType "{command_type: $JOINT}"
ros2 topic pub --times 20 -r 20 /servo/joint_jog control_msgs/msg/JointJog \
  "{joint_names: ['meca_axis_1_joint'], velocities: [0.2], displacements: [], duration: 0.0}"

# TWIST mode setzen (MoveIt Servo!)
ros2 service call $SRV moveit_msgs/srv/ServoCommandType "{command_type: $TWIST}"
ros2 topic pub --times 10 -r 10 /servo/cartesian_mm geometry_msgs/msg/TwistStamped \
  "{twist: {linear: {x: 50.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 10.0}}}"

# === FRAME: world ===
ros2 topic pub -1 /servo/set_frame std_msgs/msg/String "{data: world}"

# JOINT
ros2 service call $SRV moveit_msgs/srv/ServoCommandType "{command_type: $JOINT}"
ros2 topic pub --times 20 -r 20 /servo/joint_jog control_msgs/msg/JointJog \
  "{joint_names: ['meca_axis_1_joint'], velocities: [0.2], displacements: [], duration: 0.0}"

# TWIST
ros2 service call $SRV moveit_msgs/srv/ServoCommandType "{command_type: $TWIST}"
ros2 topic pub --times 10 -r 10 /servo/cartesian_mm geometry_msgs/msg/TwistStamped \
  "{twist: {linear: {x: 50.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 10.0}}}"
