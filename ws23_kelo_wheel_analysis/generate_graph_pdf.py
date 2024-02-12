import subprocess
import os

def run_commands_serially(cmds):
    # Append '; true' to each command to ensure the terminal stays open
    cmds = [f"{cmd}; true" for cmd in cmds]
    # Join the commands with ' && ' to ensure they are executed sequentially
    cmd_str = ' && '.join(cmds)
    # Add a read command to pause the terminal before closing
    cmd_str += ' && read -p "Press Enter to close the terminal..."'
    # Open a new gnome-terminal window and run all commands in a single instance
    subprocess.Popen(['gnome-terminal', '--', 'bash', '-c', cmd_str])

# Run commands serially in one subprocess
cmds = [
    'source /opt/ros/rolling/setup.bash',
    'cd /home/saif',
    'cd sdp_wheel',
    'source install/local_setup.bash',
    'echo "running wheel_diag_converter"',
    'ros2 run ws23_kelo_wheel_analysis wheel_diag_converter'
]
run_commands_serially(cmds)

# Run commands serially in one subprocess
cmds = [
    'source /opt/ros/rolling/setup.bash',
    'cd /home/saif',
    'cd sdp_wheel/record_feb_final',
    'source install/local_setup.bash',
    'echo "running wheel_diag_converter"',
    'ros2 run ws23_kelo_wheel_analysis wheel_diag_converter'
]
run_commands_serially(cmds)