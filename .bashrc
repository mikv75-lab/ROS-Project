# ====== ROS + XLaunch helpers (safe) ======

# Sourcen ohne Lärm (nur wenn Dateien existieren)
_source_ros() {
  [ -f /opt/ros/rolling/setup.bash ] && source /opt/ros/rolling/setup.bash
  [ -f "$HOME/ws_moveit/install/setup.bash" ] && source "$HOME/ws_moveit/install/setup.bash"
}

# XLaunch (:0.0) automatisch setzen, wenn möglich
_xlaunch_setup() {
  # nicht überschreiben, wenn DISPLAY bereits gesetzt ist
  [ -n "$DISPLAY" ] && return 0

  # Host-IP von Docker ermitteln (Windows: host.docker.internal; sonst resolv.conf)
  local HOST="$(awk '/nameserver/{print $2; exit}' /etc/resolv.conf 2>/dev/null)"
  HOST="${HOST:-host.docker.internal}"
  export DISPLAY="${HOST}:0.0"

  # robuste X11/Qt-Flags
  export QT_X11_NO_MITSHM=1
  export LIBGL_ALWAYS_INDIRECT=1
  export QT_OPENGL=software

  # Optional: schnell testen, ob X erreichbar ist (xset ist leichtgewichtiger als xclock)
  if command -v xset >/dev/null 2>&1; then
    xset q >/dev/null 2>&1 || echo "⚠️  DISPLAY $DISPLAY nicht erreichbar (XLaunch gestartet? -ac & :0)."
  fi
}

# Software-OpenGL (llvmpipe) aktivieren – hilfreich für OGRE/RViz in VMs/Containern
_rviz_swgl_env() {
  export LIBGL_ALWAYS_SOFTWARE=1
  export MESA_GL_VERSION_OVERRIDE=3.3
  export MESA_GLSL_VERSION_OVERRIDE=330
  export GALLIUM_DRIVER=llvmpipe
  # Manche Setups brauchen das, um NV-GLX zu umgehen:
  export __GLX_VENDOR_LIBRARY_NAME=mesa
}

# ===== Standard-Umgebung für ROS-Terminals setzen =====
# setze RMW & FastDDS Defaults (wie bei dir)
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE=""
export RMW_FASTRTPS_USE_QOS_FROM_XML=0
export ROS_DISABLE_SHARED_MEMORY=1
export RMW_TRANSPORT_SHARED_MEMORY_ENABLED=0

# Beim Öffnen eines interaktiven Shells:
if [[ $- == *i* ]]; then
  _source_ros         # ROS Env laden (idempotent)
  _xlaunch_setup      # DISPLAY für XLaunch setzen, falls leer
fi

# ===== Deine Helfer/aliases bleiben erhalten =====

# Kill ROS (wie bei dir)
kill_ros() {
  echo "🔪 Stoppe ROS-Prozesse …"
  if [ -f "/root/Spraycoater/src/ros/kill_all_ros.py" ]; then
    python3 /root/Spraycoater/src/ros/kill_all_ros.py || true
  else
    echo "⚠️  kill_all_ros.py nicht gefunden – fallback auf pkill"
  fi
  pkill -f move_group                 2>/dev/null || true
  pkill -f ros2_control_node          2>/dev/null || true
  pkill -f controller_manager         2>/dev/null || true
  pkill -f servo_node                 2>/dev/null || true
  pkill -f rviz2                      2>/dev/null || true
  pkill -f static_transform_publisher 2>/dev/null || true
  pkill -f robot_state_publisher      2>/dev/null || true
  pkill -f gzserver                   2>/dev/null || true
  pkill -f gzclient                   2>/dev/null || true
  pkill -f ros2                       2>/dev/null || true
  sleep 0.3
  echo "✅ ROS beendet"
}
alias kill_ros='kill_ros'

# build_launch (deine Version – unverändert)
build_launch() {
  local WS="${WS:-$HOME/ws_moveit}"
  local pkgs=()
  local launch_pkg=""
  local launch_file=""
  local launch_args=()

  while [[ $# -gt 0 ]]; do
    case "$1" in
      -w|--workspace) shift; WS="$1"; shift;;
      -p|--packages)
        shift
        while [[ $# -gt 0 && "$1" != "-l" && "$1" != "--launch" && "$1" != "--" && "$1" != "-w" && "$1" != "--workspace" && "$1" != "-p" && "$1" != "--packages" && "$1" != -* ]]; do
          if [[ "$1" == *";"* ]]; then IFS=';' read -r -a parts <<< "$1"; for p in "${parts[@]}"; do [[ -n "$p" ]] && pkgs+=("$p"); done
          else pkgs+=("$1"); fi
          shift
        done;;
      -l|--launch) shift; launch_pkg="$1"; shift || true; launch_file="$1"; shift || true;;
      --) shift; launch_args=("$@"); break;;
      *)  if [[ "$1" == *";"* ]]; then IFS=';' read -r -a parts <<< "$1"; for p in "${parts[@]}"; do [[ -n "$p" ]] && pkgs+=("$p"); done
          else pkgs+=("$1"); fi
          shift;;
    esac
  done

  if [[ ${#pkgs[@]} -eq 0 ]]; then
    echo "Usage: build_launch -p <pkg1> [pkg2 ...] [-l <launch_pkg> <launchfile.py>] [-- <launch-args>]"
    return 1
  fi

  clear
  cd "$WS" || { echo "❌ Workspace nicht gefunden: $WS"; return 1; }

  [ -f /opt/ros/rolling/setup.bash ] && source /opt/ros/rolling/setup.bash

  echo "🧹 Clean: ${pkgs[*]}"
  for p in "${pkgs[@]}"; do rm -rf "build/${p}"* "install/${p}"* 2>/dev/null || true; done

  echo "🧱 colcon build --packages-select ${pkgs[*]} --symlink-install"
  colcon build --packages-select "${pkgs[@]}" --symlink-install || { echo "❌ Build failed"; return 1; }

  [ -f install/setup.bash ] && source install/setup.bash

  if [[ -n "$launch_pkg" && -n "$launch_file" ]]; then
    echo "🚀 ros2 launch ${launch_pkg} ${launch_file} ${launch_args[*]}"
    ros2 launch "${launch_pkg}" "${launch_file}" "${launch_args[@]}"
  else
    echo "✅ Build-only abgeschlossen."
  fi
}
alias build_launch='build_launch'
