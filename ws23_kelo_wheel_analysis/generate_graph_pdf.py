import subprocess
import os

def run_command(cmd):
    process = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, shell=True)
    while True:
        output = process.stdout.readline()
        if output == '' and process.poll() is not None:
            break
        if output:
            print(output.strip())
    rc = process.poll()
    return rc

# Change to the root directory of the user
os.chdir('/home/saif')

# Source .bashrc to load environment variables
run_command('. ~/.bashrc && echo "Environment variables loaded."')

# Navigate to the sdp_wheel directory
os.chdir('sdp_wheel')

# Run ros_rolling
run_command('ros_rolling')

# Source the local_setup.bash
run_command('source install/local_setup.bash')

# Run the wheel_diag_converter
run_command('ros2 run ws23_kelo_wheel_analysis wheel_diag_converter')

# Additional processes can be added here, separated by calls to run_command