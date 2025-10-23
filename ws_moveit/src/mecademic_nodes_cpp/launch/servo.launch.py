# mecademic_nodes_cpp/launch/servo.launch.py
import os
import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory, PackageNotFoundError

# --- kleine URI-Auflösung: package://, file://, absolut ---
def resolve_uri(uri: str) -> str:
    if not isinstance(uri, str) or not uri:
        return uri
    if uri.startswith("package://"):
        pkg_and_rel = uri[len("package://"):]
        if "/" not in pkg_and_rel:
            raise ValueError(f"Ungültige package-URI: {uri}")
        pkg, rel = pkg_and_rel.split("/", 1)
        base = get_package_share_directory(pkg)  # wirft PackageNotFoundError bei Bedarf
        return os.path.join(base, rel)
    if uri.startswith("file://"):
        return uri[len("file://"):]
    return uri  # schon ein normaler Pfad

def launch_setup(context, *args, **kwargs):
    # 1) paths.yaml finden (Argument oder Fallbacks)
    paths_arg = LaunchConfiguration("paths_file").perform(context)

    # Fallback-Reihenfolge:
    #   a) explizites Argument
    #   b) package://mecademic_bringup/config/paths.yaml
    #   c) package://mecademic_nodes_cpp/config/paths.yaml
    candidates = []
    if paths_arg:
        candidates.append(paths_arg)
    candidates.append("package://mecademic_bringup/config/paths.yaml")
    candidates.append("package://mecademic_nodes_cpp/config/paths.yaml")

    paths_file = None
    last_err = None
    for cand in candidates:
        try:
            resolved = resolve_uri(cand)
            if resolved and os.path.exists(resolved):
                paths_file = resolved
                break
        except PackageNotFoundError as e:
            last_err = e
            continue

    if not paths_file:
        msg = "paths_file konnte nicht gefunden werden. Probiert: " + ", ".join(candidates)
        if last_err:
            msg += f" (letzter Fehler: {last_err})"
        raise RuntimeError(msg)

    # 2) paths.yaml laden und Einträge auflösen
    with open(paths_file, "r") as f:
        cfg = yaml.safe_load(f) or {}

    def need(key):
        if key not in cfg:
            raise RuntimeError(f"'{key}' fehlt in {paths_file}")
        p = resolve_uri(cfg[key])
        if not os.path.exists(p):
            raise RuntimeError(f"Pfad aus '{key}' existiert nicht: {p}")
        return p

    frames_file = need("frames_file")
    topics_file = need("topics_file")
    qos_file    = need("qos_file")

    # optionale Overrides aus Launch-Args
    servo_mode       = LaunchConfiguration("servo_mode").perform(context)
    servo_frame      = LaunchConfiguration("servo_frame").perform(context)
    cart_lin_mm_s    = float(LaunchConfiguration("cart_lin_mm_s").perform(context))
    cart_ang_deg_s   = float(LaunchConfiguration("cart_ang_deg_s").perform(context))
    joint_scale      = float(LaunchConfiguration("joint_scale").perform(context))

    node = Node(
        package="mecademic_nodes_cpp",
        executable="servo",
        name="servo",
        output="screen",
        parameters=[
            {
                "frames_file": frames_file,
                "topics_file": topics_file,
                "qos_file":    qos_file,

                # optionale Runtime-Overrides
                "servo.default_mode":  servo_mode,
                "servo.default_frame": servo_frame,
                "servo.speed_limits.cart_linear_mm_s":  cart_lin_mm_s,
                "servo.speed_limits.cart_angular_deg_s": cart_ang_deg_s,
                "servo.speed_limits.joint_scale":        joint_scale,
            }
        ],
    )

    return [
        # SHM in Containern oft problematisch → UDPv4
        SetEnvironmentVariable("FASTDDS_BUILTIN_TRANSPORTS", "UDPv4"),
        node,
    ]

def generate_launch_description():
    return LaunchDescription([
        # Argumente
        DeclareLaunchArgument(
            "paths_file",
            default_value="",  # leer = wir nutzen die Fallbacks (bringup → nodes_cpp)
            description="Pfad oder package://… zu einer paths.yaml"
        ),
        DeclareLaunchArgument("servo_mode",       default_value="cartesian"),
        DeclareLaunchArgument("servo_frame",      default_value="tcp"),
        DeclareLaunchArgument("cart_lin_mm_s",    default_value="100.0"),
        DeclareLaunchArgument("cart_ang_deg_s",   default_value="30.0"),
        DeclareLaunchArgument("joint_scale",      default_value="0.5"),

        # dynamischer Teil (wir brauchen LaunchContext → OpaqueFunction)
        OpaqueFunction(function=launch_setup),
    ])
