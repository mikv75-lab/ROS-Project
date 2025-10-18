# common/topics.py

"""
Zentrale Definition aller ROS Topics.
Keine Hardcoded Strings mehr im Code!
"""

# --- Tool control topics ---
TOPIC_TOOL_SET = "/tool/set"
TOPIC_TOOL_CURRENT = "/tool/current"
TOPIC_SCENE_TOOL_ATTACH = "/scene/tool/attach"
TOPIC_SCENE_TOOL_DETACH = "/scene/tool/detach"

# --- Pose topics ---
TOPIC_POSES_PREFIX = "/poses"
TOPIC_POSE_SET_FROM_TCP = "/poses/set_from_tcp"

# --- Scene topics ---
TOPIC_SCENE_MOUNT_SET   = "/scene/mount/set"
TOPIC_SCENE_MOUNT_CLEAR = "/scene/mount/clear"
TOPIC_SCENE_MOUNT_INFO  = "/scene/mount/info"

TOPIC_SCENE_SUBSTRATE_SET   = "/scene/substrate/set"
TOPIC_SCENE_SUBSTRATE_CLEAR = "/scene/substrate/clear"
TOPIC_SCENE_SUBSTRATE_INFO  = "/scene/substrate/info"

TOPIC_SCENE_ENV_SET   = "/scene/environment/set"
TOPIC_SCENE_ENV_CLEAR = "/scene/environment/clear"
TOPIC_SCENE_ENV_INFO  = "/scene/environment/info"

# --- Spray path topics ---
TOPIC_SCENE_SPRAY_SET   = "/scene/spraypath/set"
TOPIC_SCENE_SPRAY_CLEAR = "/scene/spraypath/clear"
TOPIC_SCENE_SPRAY_INFO  = "/scene/spraypath/info"

# --- Visualization topics ---
TOPIC_VISUALIZATION_MARKER = "visualization_marker"  # default RViz Marker
TOPIC_SPRAY_PATH_MARKERS = "spray_path/markers"      # MarkerArray visualization
