#!/usr/bin/env python3
import os, sys, argparse, subprocess, signal

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--tool", required=True)
    ap.add_argument("--ros-setup", required=True)
    ap.add_argument("--ws-setup", required=True)
    ap.add_argument("--rviz", default="true")
    ap.add_argument("--use-fake-hw", default="true", choices=["true", "false"],
                    help="Simulationsmodus via ros2_control aktivieren (default: true)")
    args = ap.parse_args()

    cmd = f'''
        source "{args.ros_setup}" || true
        source "{args.ws_setup}" || true
        exec ros2 launch mecademic_bringup robot_with_tool.launch.py \
            tool:={args.tool} \
            use_fake_hw:={args.use_fake_hw} \
            rviz:={args.rviz}
    '''
    subprocess.call(["/bin/bash", "-lc", cmd], preexec_fn=os.setsid)

if __name__ == "__main__":
    main()
