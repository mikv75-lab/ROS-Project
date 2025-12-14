#!/usr/bin/env bash
clear
set -Eeuo pipefail

# ===== RMW / FastDDS =====
export FASTDDS_SHM_DEFAULT=0
export FASTDDS_BUILTIN_TRANSPORTS=UDPv4

# ===== Topics / Service =====
MODE_TOPIC=/servo/set_mode
FRAME_TOPIC=/servo/set_frame
TWIST_IN=/servo/cartesian_mm
JOINT_IN=/servo/joint_jog
SWITCH_SRV=/servo/switch_command_type   # optional

# Debug-Out (nur zum Beobachten)
TWIST_OUT=/servo_server/delta_twist_cmds
JOINT_OUT=/servo_server/delta_joint_cmds

say() { echo -e "\n\033[1;36m>>> $*\033[0m"; }
sleep_s() { sleep "$1"; }

wait_for_topic() {
  local topic="$1"; local timeout="${2:-10}"
  local t0; t0=$(date +%s)
  say "Warte auf Topic: $topic ..."
  while ! ros2 topic info "$topic" >/dev/null 2>&1; do
    (( $(date +%s) - t0 >= timeout )) && { echo "Timeout: $topic nicht innerhalb ${timeout}s gefunden."; return 1; }
    sleep 0.2
  done
  echo "OK: $topic ist verfügbar."
}

pub_mode()  { ros2 topic pub -1 "$MODE_TOPIC"  std_msgs/msg/String "{data: '$1'}"; }
pub_frame() { ros2 topic pub -1 "$FRAME_TOPIC" std_msgs/msg/String "{data: '$1'}"; }

pub_joint_vel() {
  # Args: joint_name vel_rad_s cycles(=20 -> 1.0s @20Hz)
  local j="$1" v="$2" cycles="${3:-20}"
  ros2 topic pub --times "$cycles" -r 20 "$JOINT_IN" control_msgs/msg/JointJog \
"{joint_names: [\"$j\"], velocities: [$v], displacements: [], duration: 0.0}"
}

pub_twist_mm_s() {
  # Args: x y z wx wy wz cycles hz  (default: 0 / 10 / 10)
  local x="${1:-0.0}"  y="${2:-0.0}"  z="${3:-0.0}"
  local wx="${4:-0.0}" wy="${5:-0.0}" wz="${6:-0.0}"
  local cyc="${7:-10}" hz="${8:-10}"
  ros2 topic pub --times "$cyc" -r "$hz" "$TWIST_IN" geometry_msgs/msg/TwistStamped \
'{
  header: {frame_id: ""},
  twist:  { linear:  { x: '"$x"',  y: '"$y"',  z: '"$z"'  },
            angular: { x: '"$wx"', y: '"$wy"', z: '"$wz"' } }
}'
}

warmup_joint_pose() {
  # kleine Offsets gegen Singularity: J2 +0.25, J3 -0.35, J4 +0.2 (je ~0.5s)
  say "Warmup: kleine JOINT-Offsets (J2/J3/J4)"
  pub_joint_vel meca_axis_2_joint  0.50 10
  pub_joint_vel meca_axis_3_joint -0.70 10
  pub_joint_vel meca_axis_4_joint  0.40 10
}

run_round() {
  local frame="$1"
  say "==== Runde: FRAME = $frame ===="
  pub_frame "$frame"
  # Tipp: Falls 'tool_mount' in der Bridge nicht konfiguriert ist, kommt ein WARN-Log
  #       und der Frame bleibt unverändert (Fallback wäre dann 'tcp').

  say "Modus → JOINT"
  pub_mode joint
  ros2 service call "$SWITCH_SRV" moveit_msgs/srv/ServoCommandType '{command_type: 0}' || true

  warmup_joint_pose

  say "Modus → CARTESIAN (Twist)"
  pub_mode cartesian
  ros2 service call "$SWITCH_SRV" moveit_msgs/srv/ServoCommandType '{command_type: 1}' || true

  say "TWIST +X (30 mm/s) für 1.0s"
  pub_twist_mm_s 30 0 0 0 0 0 10 10

  say "TWIST -X (30 mm/s) für 1.0s"
  pub_twist_mm_s -30 0 0 0 0 0 10 10

  # Zur Sicherheit zurück in JOINT (keine Bewegung, nur Modus)
  pub_mode joint
  ros2 service call "$SWITCH_SRV" moveit_msgs/srv/ServoCommandType '{command_type: 0}' || true
}

# ===== Ablauf =====
wait_for_topic /joint_states 15 || true
sleep_s 0.8

# Runde 1: tool_mount
run_round tool_mount

# Runde 2: world
run_round world

say "Fertig. Debug parallel möglich: 'ros2 topic echo $TWIST_OUT' / '$JOINT_OUT'"
