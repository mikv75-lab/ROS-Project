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

# --- Alias Build, Kill ---
alias mecademic_colcon_build='cd ~/ws_moveit && clear && rm -rf build/mecademic_* install/mecademic_* log && colcon build --packages-select mecademic_description mecademic_moveit_config mecademic_bringup --cmake-clean-first && source install/setup.bash && ros2 launch mecademic_bringup bringup.launch.py'
alias kill_ros="python3 ~/app/src/app/kill_all_ros.py && echo '✅ ROS beendet in ws_moveit'"
alias servo_build_run= "clear && cd ~/ws_moveit && rm -rf build/mecademic_nodes_cpp* && colcon build --packages-select mecademic_nodes_cpp --cmake-clean-first && source install/setup.bash && ros2 launch mecademic_nodes_cpp servo.launch.py"
alias servo_run='clear && cd ~/ws_moveit && rm -rf build/mecademic_nodes_cpp* && \
  colcon build --packages-select mecademic_nodes_cpp --cmake-clean-first && \
  source install/setup.bash && ros2 launch mecademic_nodes_cpp servo.launch.py'
