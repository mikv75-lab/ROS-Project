# common/frames.py

# --- Global reference frames ---
FRAME_WORLD = "world"
FRAME_MECA_MOUNT = "meca_mount"  # Roboteraufspannpunkt

# --- Robot frames ---
FRAME_MECA_BASE = "meca_base"  # Roboterbasis
FRAME_TCP = "tcp"              # Tool Center Point (aktives Tool)

# --- Defined poses from fixed_positions.yaml ---
FRAME_POSE_HOME = "meca_pose_home"
FRAME_POSE_PREDISPENSE = "meca_pose_predispense"
FRAME_POSE_SERVICE = "meca_pose_service"

# --- Scene frames (Workspace / Produktionszelle) ---
FRAME_SUBSTRATE_MOUNT = "substrate_mount"  # Halterung/Fixture auf Tisch
FRAME_SCENE = "scene"  # war workspace_center – TF für Arbeitsbereich
FRAME_SUBSTRATE = "substrate"  # aufgelegtes Teil, Wafer etc.

# --- Registry of valid frames (zentral für Prüfung & Konsistenz) ---
VALID_FRAMES = {
    FRAME_WORLD,
    FRAME_MECA_MOUNT,
    FRAME_MECA_BASE,
    FRAME_TCP,
    FRAME_POSE_HOME,
    FRAME_POSE_PREDISPENSE,
    FRAME_POSE_SERVICE,
    FRAME_SUBSTRATE_MOUNT,
    FRAME_SCENE,
    FRAME_SUBSTRATE,
}
