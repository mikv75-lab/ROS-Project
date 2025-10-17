#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    OpaqueFunction,
    SetEnvironmentVariable,
    RegisterEventHandler,
)
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.event_handlers import OnProcessStart, OnProcessExit

import os, math, yaml

# --- YAML defaults laden ---
DEFAULT_YAML = os.environ.get("MEC_STARTUP_YAML", "/root/app/config/startup.yaml")

def _fmt3(v): return " ".join(str(float(x)) for x in v)

def _yaml_defaults():
    d = {}
    try:
        with open(DEFAULT_YAML, "r") as f:
            d = yaml.safe_load(f) or {}
    except Exception as e:
        print(f"[bringup.launch] WARN: cannot read '{DEFAULT_YAML}': {e}")
        d = {}

    wtm = d.get("world_to_meca_mount") or {}
    wm  = d.get("workspace_mount") or {}
    cg  = d.get("cage") or {}

    def _res(p):
        if not p: return ""
        return p if os.path.isabs(p) else os.path.abspath(os.path.join("/root/app", p))

    return {
        "world_to_meca_xyz":       _fmt3(wtm.get("xyz", [0,0,0])),
        "world_to_meca_rpy_deg":   _fmt3(wtm.get("rpy_deg", [0,0,90])),

        "workspace_mount_enable":  "true" if wm.get("enable", False) else "false",
        "workspace_mount_mesh":    _res(wm.get("mesh_path", "")),
        "workspace_mount_xyz":     _fmt3(wm.get("xyz", [0.2,0,0])),
        "workspace_mount_rpy_deg": _fmt3(wm.get("rpy_deg", [0,0,0])),

        "cage_enable":             "true" if cg.get("enable", False) else "false",
        "cage_mesh":               _res(cg.get("mesh_path", "")),
        "cage_xyz":                _fmt3(cg.get("xyz", [0,0,0])),
        "cage_rpy_deg":            _fmt3(cg.get("rpy_deg", [0,0,0])),
    }

_DEFAULTS = _yaml_defaults()

def _parse_vec3(s: str, name: str):
    parts = [p for p in s.replace(",", " ").split() if p]
    if len(parts) != 3:
        raise RuntimeError(f"{name} erwartet genau 3 Werte, bekommen: {s!r}")
    return (float(parts[0]), float(parts[1]), float(parts[2]))

def _rpy_deg_to_quat(r, p, y):
    # Roll(X), Pitch(Y), Yaw(Z) -> Quaternion (normiert)
    rx, ry, rz = map(math.radians, (r, p, y))
    cr, sr = math.cos(rx * 0.5), math.sin(rx * 0.5)
    cp, sp = math.cos(ry * 0.5), math.sin(ry * 0.5)
    cy, sy = math.cos(rz * 0.5), math.sin(rz * 0.5)

    qw = cr*cp*cy + sr*sp*sy
    qx = sr*cp*cy - cr*sp*sy
    qy = cr*sp*cy + sr*cp*sy
    qz = cr*cp*sy - sr*sp*cy
    n = math.sqrt(qx*qx + qy*qy + qz*qz + qw*qw)
    return (qx/n, qy/n, qz/n, qw/n)

def generate_launch_description():
    # world -> meca_mount
    world_xyz_arg = DeclareLaunchArgument("world_to_meca_xyz",     default_value=_DEFAULTS["world_to_meca_xyz"])
    world_rpy_arg = DeclareLaunchArgument("world_to_meca_rpy_deg", default_value=_DEFAULTS["world_to_meca_rpy_deg"])

    # workspace_mount
    wm_enable_arg = DeclareLaunchArgument("workspace_mount_enable", default_value=_DEFAULTS["workspace_mount_enable"])
    wm_mesh_arg   = DeclareLaunchArgument(
        "workspace_mount_mesh",
        default_value=_DEFAULTS["workspace_mount_mesh"] or PathJoinSubstitution(
            [FindPackageShare("mecademic_bringup"), "resource", "world", "base_platform_200x200x50.stl"])
    )
    wm_xyz_arg    = DeclareLaunchArgument("workspace_mount_xyz",     default_value=_DEFAULTS["workspace_mount_xyz"])
    wm_rpy_arg    = DeclareLaunchArgument("workspace_mount_rpy_deg", default_value=_DEFAULTS["workspace_mount_rpy_deg"])

    # cage
    cg_enable_arg = DeclareLaunchArgument("cage_enable", default_value=_DEFAULTS["cage_enable"])
    cg_mesh_arg   = DeclareLaunchArgument(
        "cage_mesh",
        default_value=_DEFAULTS["cage_mesh"] or PathJoinSubstitution(
            [FindPackageShare("mecademic_bringup"), "resource", "world", "cage_example.stl"])
    )
    cg_xyz_arg    = DeclareLaunchArgument("cage_xyz",     default_value=_DEFAULTS["cage_xyz"])
    cg_rpy_arg    = DeclareLaunchArgument("cage_rpy_deg", default_value=_DEFAULTS["cage_rpy_deg"])

    # übrige
    tools_yaml_arg = DeclareLaunchArgument(
        "tools_yaml",
        default_value=PathJoinSubstitution([FindPackageShare("mecademic_bringup"), "config", "tools.yaml"]),
    )
    persist_file_arg = DeclareLaunchArgument(
        "persist_file",
        default_value=PathJoinSubstitution([FindPackageShare("mecademic_bringup"), "config", "active_tool.txt"]),
    )
    positions_yaml_arg = DeclareLaunchArgument(
        "positions_yaml",
        default_value=PathJoinSubstitution([FindPackageShare("mecademic_bringup"), "config", "fixed_positions.yaml"]),
    )
    workpiece_file_arg = DeclareLaunchArgument(
        "workpiece_file",
        default_value=PathJoinSubstitution([FindPackageShare("mecademic_bringup"), "config", "current_workpiece.txt"]),
    )

    def launch_setup(context):
        parent_frame = "meca_mount"

        world_xyz = _parse_vec3(LaunchConfiguration("world_to_meca_xyz").perform(context), "world_to_meca_xyz")
        world_rpy = _parse_vec3(LaunchConfiguration("world_to_meca_rpy_deg").perform(context), "world_to_meca_rpy_deg")
        qx, qy, qz, qw = _rpy_deg_to_quat(*world_rpy)

        wm_enable = LaunchConfiguration("workspace_mount_enable").perform(context).lower() == "true"
        wm_mesh   = LaunchConfiguration("workspace_mount_mesh").perform(context)
        wm_xyz    = _parse_vec3(LaunchConfiguration("workspace_mount_xyz").perform(context), "workspace_mount_xyz")
        wm_rpy    = _parse_vec3(LaunchConfiguration("workspace_mount_rpy_deg").perform(context), "workspace_mount_rpy_deg")

        cg_enable = LaunchConfiguration("cage_enable").perform(context).lower() == "true"
        cg_mesh   = LaunchConfiguration("cage_mesh").perform(context)
        cg_xyz    = _parse_vec3(LaunchConfiguration("cage_xyz").perform(context), "cage_xyz")
        cg_rpy    = _parse_vec3(LaunchConfiguration("cage_rpy_deg").perform(context), "cage_rpy_deg")

        tools_yaml     = LaunchConfiguration("tools_yaml").perform(context)
        persist_file   = LaunchConfiguration("persist_file").perform(context)
        positions_yaml = LaunchConfiguration("positions_yaml").perform(context)
        workpiece_file = LaunchConfiguration("workpiece_file").perform(context)

        print("\n================ MECADIC ROBOT BRINGUP ================")
        print(f"[bringup.launch] world->meca_mount xyz={world_xyz} rpy_deg={world_rpy}")
        print(f"[bringup.launch] workspace_mount enable={wm_enable} xyz={wm_xyz} rpy_deg={wm_rpy}")
        print(f"[bringup.launch] workspace_mount mesh={wm_mesh}")
        print(f"[bringup.launch] cage enable={cg_enable} xyz={cg_xyz} rpy_deg={cg_rpy}")
        print(f"[bringup.launch] cage mesh={cg_mesh}\n")

        # 0) Umgebungsvariablen
        env = [
            SetEnvironmentVariable("XDG_RUNTIME_DIR", "/tmp/runtime-root"),
            SetEnvironmentVariable(name="RMW_IMPLEMENTATION", value="rmw_fastrtps_cpp"),
        ]

        # 2) poses_manager NACH static TF
        poses_mgr = Node(
            package="mecademic_bringup", executable="poses_manager", name="poses_manager",
            output="screen", parameters=[{"positions_yaml": positions_yaml, "parent_frame": parent_frame}]
        )
        chain1 = RegisterEventHandler(OnProcessStart(target_action=static_tf, on_start=[poses_mgr]))

        # 3) tool_manager NACH poses_manager (startet Roboter/MoveIt)
        tool_mgr = Node(
            package="mecademic_bringup", executable="tool_manager", name="tool_manager",
            output="screen", parameters=[{
                "tools_yaml": tools_yaml, "persist_file": persist_file,
                # restliche Parameter werden intern ignoriert (neu aufgebauter ToolManager)
                "auto_spawn_robot": True,
                "robot_launch_pkg": "mecademic_bringup",
                "robot_launch_file": "robot_with_tool.launch.py",
                "ros_setup_bash": "/opt/ros/rolling/setup.bash",
                "ws_setup_bash": "/root/ws_moveit/install/setup.bash",
                "spawn_log_path": "/tmp/robot_with_tool.log",
            }]
        )
        chain2 = RegisterEventHandler(OnProcessStart(target_action=poses_mgr, on_start=[tool_mgr]))

        # 4) Environment NACHDEM move_group vom ToolManager läuft:
        #    Wir starten die Spawner in Reihenfolge; jeder Spawner beendet sich, wenn fertig.
        env_chain = []

        if cg_enable:
            cage_spawner = Node(
                package="mecademic_bringup", executable="spawn_mesh", name="cage_spawner", output="screen",
                parameters=[{
                    "parent_frame": parent_frame,
                    "object_id": "cage",
                    "mesh_path": cg_mesh,
                    "xyz": list(cg_xyz),
                    "rpy_deg": list(cg_rpy),
                    "publish_static_tf": True,
                    "child_frame": "cage",
                    "scale": 1.0,
                    "fallback_box": [1.2, 1.2, 1.2],
                    "wait_for_service": True,       # <— wartet selbst auf /apply_planning_scene
                    "wait_log_period": 5.0,
                }]
            )
            env_chain.append(cage_spawner)

        if wm_enable:
            workspace_spawner = Node(
                package="mecademic_bringup", executable="spawn_mesh", name="workspace_mount_spawner", output="screen",
                parameters=[{
                    "parent_frame": parent_frame,
                    "object_id": "workspace_mount",
                    "mesh_path": wm_mesh,
                    "xyz": list(wm_xyz),
                    "rpy_deg": list(wm_rpy),
                    "publish_static_tf": True,
                    "child_frame": "workspace_mount",
                    "scale": 1.0,
                    "fallback_box": [0.2, 0.2, 0.05],
                    "wait_for_service": True,       # <— wartet selbst auf /apply_planning_scene
                    "wait_log_period": 5.0,
                }]
            )
            env_chain.append(workspace_spawner)

        workpiece_mgr = Node(
            package="mecademic_bringup", executable="workpiece_manager", name="workpiece_manager",
            output="screen", parameters=[{"workpiece_file": workpiece_file}]
        )

        # Reihenfolge: tool_mgr (startet move_group) → cage → workspace → workpiece
        # Wir nutzen OnProcessStart/Exit um deterministisch ohne Timer zu sequenzieren.
        chain3_actions = []
        if env_chain:
            # Starte ersten Spawner sobald der tool_manager läuft (move_group kommt gleich danach)
            first = env_chain[0]
            chain3_actions.append(RegisterEventHandler(OnProcessStart(target_action=tool_mgr, on_start=[first])))

            # Kette die restlichen Spawner an das "Exit" des vorherigen
            for prev, nxt in zip(env_chain, env_chain[1:]):
                chain3_actions.append(RegisterEventHandler(OnProcessExit(target_action=prev, on_exit=[nxt])))

            # Wenn letzter Spawner fertig ist → workpiece_manager
            chain3_actions.append(RegisterEventHandler(OnProcessExit(target_action=env_chain[-1], on_exit=[workpiece_mgr])))
        else:
            # Kein Cage/Workspace → starte Workpiece direkt nach tool_manager
            chain3_actions.append(RegisterEventHandler(OnProcessStart(target_action=tool_mgr, on_start=[workpiece_mgr])))

        return env + [static_tf, chain1, chain2] + chain3_actions

    return LaunchDescription([
        world_xyz_arg, world_rpy_arg,
        wm_enable_arg, wm_mesh_arg, wm_xyz_arg, wm_rpy_arg,
        cg_enable_arg, cg_mesh_arg, cg_xyz_arg, cg_rpy_arg,
        tools_yaml_arg, persist_file_arg, positions_yaml_arg, workpiece_file_arg,
        OpaqueFunction(function=launch_setup),
    ])
