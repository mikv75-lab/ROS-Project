# ~/.bashrc: executed by bash(1) for non-login shells.
# see /usr/share/doc/bash/examples/startup-files (in the package bash-doc)
# for examples

# If not running interactively, don't do anything
[ -z "$PS1" ] && return

# don't put duplicate lines in the history. See bash(1) for more options
# ... or force ignoredups and ignorespace
HISTCONTROL=ignoredups:ignorespace

# append to the history file, don't overwrite it
shopt -s histappend

# for setting history length see HISTSIZE and HISTFILESIZE in bash(1)
HISTSIZE=1000
HISTFILESIZE=2000

# check the window size after each command and, if necessary,
# update the values of LINES and COLUMNS.
shopt -s checkwinsize

# make less more friendly for non-text input files, see lesspipe(1)
[ -x /usr/bin/lesspipe ] && eval "$(SHELL=/bin/sh lesspipe)"

# set variable identifying the chroot you work in (used in the prompt below)
if [ -z "$debian_chroot" ] && [ -r /etc/debian_chroot ]; then
    debian_chroot=$(cat /etc/debian_chroot)
fi

# set a fancy prompt (non-color, unless we know we "want" color)
case "$TERM" in
    xterm-color) color_prompt=yes;;
esac

#force_color_prompt=yes

if [ -n "$force_color_prompt" ]; then
    if [ -x /usr/bin/tput ] && tput setaf 1 >&/dev/null; then
        color_prompt=yes
    else
        color_prompt=
    fi
fi

if [ "$color_prompt" = yes ]; then
    PS1='${debian_chroot:+($debian_chroot)}\[\033[01;32m\]\u@\h\[\033[00m\]:\[\033[01;34m\]\w\[\033[00m\]\$ '
else
    PS1='${debian_chroot:+($debian_chroot)}\u@\h:\w\$ '
fi
unset color_prompt force_color_prompt

# If this is an xterm set the title to user@host:dir
case "$TERM" in
xterm*|rxvt*)
    PS1="\[\e]0;${debian_chroot:+($debian_chroot)}\u@\h: \w\a\]$PS1"
    ;;
*)
    ;;
esac

# enable color support of ls and also add handy aliases
if [ -x /usr/bin/dircolors ]; then
    test -r ~/.dircolors && eval "$(dircolors -b ~/.dircolors)" || eval "$(dircolors -b)"
    alias ls='ls --color=auto'
    alias grep='grep --color=auto'
    alias fgrep='fgrep --color=auto'
    alias egrep='egrep --color=auto'
fi

# some more ls aliases
alias ll='ls -alF'
alias la='ls -A'
alias l='ls -CF'

# Load user aliases
if [ -f ~/.bash_aliases ]; then
    . ~/.bash_aliases
fi

# --- ROS2 + Workspace Umgebung laden ---
source /opt/ros/rolling/setup.bash
source ~/ws_moveit/install/setup.bash

export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE=""
export RMW_FASTRTPS_USE_QOS_FROM_XML=0
export ROS_DISABLE_SHARED_MEMORY=1
export RMW_TRANSPORT_SHARED_MEMORY_ENABLED=0

# ===== Helper: ROS-Umgebung sauber sourcen =====
_source_ros() {
  # Opt. ROS + Workspace sourcen, ohne Lärm wenn Files fehlen
  [ -f /opt/ros/rolling/setup.bash ] && source /opt/ros/rolling/setup.bash
  [ -f ~/ws_moveit/install/setup.bash ] && source ~/ws_moveit/install/setup.bash
}

# ===== Kill ROS sauber =====
kill_ros() {
  echo "🔪 Stoppe ROS-Prozesse …"
  if [ -f "/root/Spraycoater/src/ros/kill_all_ros.py" ]; then
    python3 /root/Spraycoater/src/ros/kill_all_ros.py || true
  else
    echo "⚠️  kill_all_ros.py nicht gefunden – fallback auf pkill"
  fi

  # Fallback/Rest-Cleanup (idempotent)
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
  echo "✅ ROS beendet in ws_moveit"
}

alias kill_ros='kill_ros'

# ===== build_launch: Build ausgewählter Pakete + optional Launch =====
# Usage:
#   build_launch -p <pkg1> [pkg2 ...] [-l <launch_pkg> <launchfile.py>] [-- <launch-args>]
#   build_launch -p "pkg1;pkg2;pkg3" [-l <launch_pkg> <launchfile.py>] [-- <launch-args>]
#   build_launch -w /anderer/workspace -p pkg1 -l pkg1 file.launch.py -- arg:=val
#
# Flags:
#   -p|--packages   Paketliste (Leerzeichen oder Semikolon)
#   -l|--launch     <launch_pkg> <launchfile.py>
#   -w|--workspace  Workspace (Default: ~/ws_moveit)

build_launch() {
  local WS="${WS:-$HOME/ws_moveit}"
  local pkgs=()
  local launch_pkg=""
  local launch_file=""
  local launch_args=()

  # ---- Argumente parsen ----
  while [[ $# -gt 0 ]]; do
    case "$1" in
      -w|--workspace)
        shift; WS="$1"; shift;;
      -p|--packages)
        shift
        # sammle alle Tokens bis zum nächsten Flag (-l/--launch/--/-w/-p/-*)
        while [[ $# -gt 0 && "$1" != "-l" && "$1" != "--launch" && "$1" != "--" && "$1" != "-w" && "$1" != "--workspace" && "$1" != "-p" && "$1" != "--packages" && "$1" != -* ]]; do
          # Semikolonlisten erlauben
          if [[ "$1" == *";"* ]]; then
            IFS=';' read -r -a parts <<< "$1"
            for p in "${parts[@]}"; do [[ -n "$p" ]] && pkgs+=("$p"); done
          else
            pkgs+=("$1")
          fi
          shift
        done
        ;;
      -l|--launch)
        shift
        launch_pkg="$1"; shift || true
        launch_file="$1"; shift || true
        ;;
      --)
        shift
        launch_args=("$@")
        break;;
      *)
        # Falls jemand Pakete ohne -p angibt: akzeptieren
        if [[ "$1" == *";"* ]]; then
          IFS=';' read -r -a parts <<< "$1"
          for p in "${parts[@]}"; do [[ -n "$p" ]] && pkgs+=("$p"); done
        else
          pkgs+=("$1")
        fi
        shift;;
    esac
  done

  # ---- Validierung ----
  if [[ ${#pkgs[@]} -eq 0 ]]; then
    echo "Usage: build_launch -p <pkg1> [pkg2 ...] [-l <launch_pkg> <launchfile.py>] [-- <launch-args>]"
    return 1
  fi

  clear
  cd "$WS" || { echo "❌ Workspace nicht gefunden: $WS"; return 1; }

  # ROS-Env sourcen (idempotent)
  [ -f /opt/ros/rolling/setup.bash ] && source /opt/ros/rolling/setup.bash

  # ---- Clean + Build ----
  echo "🧹 Clean: ${pkgs[*]}"
  for p in "${pkgs[@]}"; do
    rm -rf "build/${p}"* "install/${p}"* 2>/dev/null || true
  done

  echo "🧱 colcon build --packages-select ${pkgs[*]} --symlink-install"
  colcon build --packages-select "${pkgs[@]}" --symlink-install || { echo "❌ Build failed"; return 1; }

  # Workspace-Env nach Build
  [ -f install/setup.bash ] && source install/setup.bash

  # ---- Optional Launch ----
  if [[ -n "$launch_pkg" && -n "$launch_file" ]]; then
    echo "🚀 ros2 launch ${launch_pkg} ${launch_file} ${launch_args[*]}"
    ros2 launch "${launch_pkg}" "${launch_file}" "${launch_args[@]}"
  else
    echo "✅ Build-only abgeschlossen."
  fi
}

# bequemer Alias
alias build_launch='build_launch'

