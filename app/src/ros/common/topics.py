# ros/common/topics.py

from typing import Optional

# Kanonische Publish-Topics (von App -> ROS)
PUBLISH_TOPICS = {
    # Tool
    "tool_set": "meca/tool/set",               # std_msgs/String (App -> ToolManager)
    "tool_status": "meca/tool/status",   # std_msgs/Empty (App -> ToolManager)

    # Posen
    "poses_status": "meca/poses/republish",  # std_msgs/Empty (App -> PosesManager)
    "poses_set_from_tcp": "meca/poses/set_from_tcp",  # std_msgs/String (App -> PosesManager)

    # Substrate / Mount / Environment
    "substrate_load": "meca/substrate/load",   # std_msgs/String (Pfad zur STL) (App -> SubstrateManager)
    "substrate_remove": "meca/substrate/remove",  # std_msgs/Empty (App -> SubstrateManager)
    "mount_load": "meca/mount/load",           # std_msgs/String (Pfad zur STL) (App -> MountManager)
    "environment_load": "meca/environment/load",  # std_msgs/String (Pfad zur STL) (App -> EnvironmentManager)

    # Spray Path
    "spray_path_set": "meca/spray_path/set",   # geometry_msgs/PoseArray (App -> SprayPathManager)
    "spray_path_clear": "meca/spray_path/clear",  # std_msgs/Empty (App -> SprayPathManager)

    # Motion / Controller
    "joint_trajectory": "meca/joint_trajectory",  # trajectory_msgs/JointTrajectory (App -> Controller)
    "fjt_action": "meca/follow_joint_trajectory",  # control_msgs/action/FollowJointTrajectory (App -> Controller)
}

# Kanonische Subscribe-Topics (von ROS -> App)
SUBSCRIBE_TOPICS = {
    # Tool
    "tool_current": "meca/tool/current",        # std_msgs/String (ToolManager -> App, latched)

    # Posen
    "pose_current": "meca/poses/current",       # geometry_msgs/PoseStamped (latched)

    # Substrate / Mount / Environment
    "substrate_current": "meca/substrate/current",  # std_msgs/String (latched)
    "mount_current": "meca/mount/current",        # std_msgs/String (latched)
    "environment_current": "meca/environment/current",  # std_msgs/String (latched)

    # Spray Path
    "spray_path_current": "meca/spray_path/current",  # geometry_msgs/PoseArray (latched)

    # Motion / Controller
    "joint_states": "meca/joint_states",  # sensor_msgs/JointState (Controller -> App)
}

# Hilfsfunktionen zum Auflösen der Topics
def resolve_topic(node, name: str) -> str:
    """Gibt den vollständig aufgelösten Topic-Namen (inkl. Namespace/Remaps) zurück."""
    return node.resolve_topic_name(name)

def resolve_service(node, name: str) -> str:
    """Gibt den vollständig aufgelösten Service-Namen (inkl. Namespace/Remaps) zurück."""
    return node.resolve_service_name(name)

__all__ = ["PUBLISH_TOPICS", "SUBSCRIBE_TOPICS", "resolve_topic", "resolve_service"]




