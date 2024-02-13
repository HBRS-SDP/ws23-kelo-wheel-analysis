import rclpy
from rclpy.node import Node
from ws23_kelo_wheel_analysis.msg import WheelDiag
from rclpy.qos import qos_profile_sensor_data
import matplotlib.pyplot as plt
from collections import defaultdict
import argparse
import time
import math
import matplotlib.cm as cm
import numpy as np
import random
from matplotlib.patches import Patch
from geometry_msgs.msg import Twist
from rclpy.qos import QoSProfile
from matplotlib.colors import LinearSegmentedColormap
from scipy.signal import cheby1, filtfilt
from scipy.stats import zscore
import numpy as np
from sklearn.cluster import DBSCAN


class LossContact(Node):

    def __init__(self, ethercat_numbers=None, sensors=None, window_size=None, wheels=None, yrange=None, title=None):
        super().__init__('wheel_analysis')
        print("Initializing WheelAnalysis Node...")
        custom_qos_profile = QoSProfile(depth=1000)  # Adjust the depth to a suitable value
        self.subscription = self.create_subscription(
            WheelDiag,
            'wheel_diag_non_json',
            self.listener_callback,
            custom_qos_profile)
        
        # Inside the __init__ method of the WheelAnalysis class
        self.cmd_vel_subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            qos_profile_sensor_data
        )
        self.cmd_vel_subscription  # prevent unused variable warning
        self.subscription  # prevent unused variable warning
        self.yrange = yrange
        self.data = defaultdict(lambda: defaultdict(list))  # {ethercat_number: {sensor: [values]}}
        self.ax_dict = {}  # {ethercat_number: ax}
        self.start_time = None  # Initialize start_time as None
        self.title = title
        self.ethercat_numbers = ethercat_numbers if ethercat_numbers else []
        self.sensors = sensors if sensors else []
        self.window_size = window_size if window_size else float('inf')  # in seconds
        self.ethercat_wheel_map = dict(zip(ethercat_numbers, wheels)) if ethercat_numbers and wheels else {}
        self.start_times = {ethercat_number: None for ethercat_number in ethercat_numbers} if ethercat_numbers else {}
        self.previous_timestamp = time.time()
        self.last_plot_update = {ethercat_number: None for ethercat_number in self.ethercat_numbers}

        # Create a single plot instead of a grid of subplots
        self.fig, ax = plt.subplots(figsize=(19, 10))  # Set figure size to 800x600
        # Create a custom colormap
        colors = ['blue', 'green', 'cyan', 'magenta', 'red', 'purple']
        custom_colormap = LinearSegmentedColormap.from_list('custom_colormap', colors)

        # Now use the custom colormap instead of viridis
        self.colors = custom_colormap(np.linspace(0, 1, len(self.sensors)))
        # After creating the figure and axes, set the suptitle
        print(f"Title set to: '{self.title}'")
        # Correct the suptitle method call to use 'color' instead of 'fontcolor'
        self.fig.suptitle(self.title, fontsize=16, fontweight='bold', y=1.05, color='red')
        self.ax_dict = {ethercat_number: ax for ethercat_number in self.ethercat_numbers}
        ax.set_ylabel('Sensor Data')
        plt.show(block=False)
        print("WheelAnalysis Node initialized.")


    def cmd_vel_callback(self, msg):
        self.cmd_vel_msg = msg

    def listener_callback(self, msg):
        #print(f"Received message with  ethercat_number: {msg.ethercat_number}")
        if msg.ethercat_number in self.ethercat_numbers:
            # Convert nanoseconds to seconds
            sensor_ts = getattr(msg, 'sensor_ts') *  1e-9 if getattr(msg, 'sensor_ts') is not None else None
            
            if self.start_times[msg.ethercat_number] is None:
                self.start_times[msg.ethercat_number] = sensor_ts
            elapsed_time = sensor_ts - self.start_times[msg.ethercat_number] if sensor_ts is not None else None
            
            #print(elapsed_time)
            for sensor in self.sensors:
                if getattr(msg, sensor) is not None:  # Check if sensor data exists
                    self.data[msg.ethercat_number]['time'].append(elapsed_time)
                    self.data[msg.ethercat_number][sensor].append(getattr(msg, sensor))
            # Check if enough time has passed since the last plot update for this  ethercat_number
            current_time = time.time()
            if self.last_plot_update[msg.ethercat_number] is None or current_time - self.last_plot_update[msg.ethercat_number] >=  0.3:
                self.plot_data(msg.ethercat_number)
                self.last_plot_update[msg.ethercat_number] = current_time
        
    



    def plot_data(self, ethercat_number):
        ax = self.ax_dict[ethercat_number]
        wheel_number = self.ethercat_wheel_map.get(ethercat_number, ethercat_number)  # Use ethercat number if wheel number is not found
        for idx, sensor in enumerate(self.sensors):
            sensor_data = self.data[ethercat_number][sensor]
            color = self.colors[idx]
            sensor_label = f"Wheel {wheel_number} {sensor}"
            line = next((line for line in ax.lines if line.get_label() == sensor_label), None)
            if line is None:
                line, = ax.plot(sensor_data, label=sensor_label, color=color)
                ax.legend(loc='upper left', bbox_to_anchor=(0, 1), borderaxespad=0.)
            else:
                line.set_ydata(sensor_data)
                line.set_xdata(range(len(sensor_data)))
        ax.relim()
        ax.autoscale_view()
        ax.set_ylabel('\t'.join([f"Wheel {wheel_number} {sensor}" for sensor in self.sensors]))
        ax.set_xlabel('Time')
        if self.yrange:
            ax.set_ylim(self.yrange)  # Set the y-axis range
        ax.set_title(f"Wheel Number: {wheel_number}")
        self.fig.text(0.5,  0.95, self.title, ha='center', va='baseline', fontsize=16, color='red')
        # Assuming self.cmd_vel_msg is defined and updated in cmd_vel_callback
        if hasattr(self, 'cmd_vel_msg'):
            cmd_vel_text = str(self.cmd_vel_msg)
            self.fig.text(0.5,  0.92, cmd_vel_text, ha='center', va='baseline', fontsize=12, color='black')
        # Calculate elapsed time since the last draw
        current_time = time.time()
        elapsed_time = current_time - self.previous_timestamp
        # Only redraw and flush events if more than  1 second has passed
        if elapsed_time >  1:
            self.fig.canvas.draw()
            self.fig.canvas.flush_events()
            self.previous_timestamp = current_time  # Update the previous timestamp
        self.is_contact_loss_iqr(self.sensors)

    def is_contact_loss_iqr(self, sensors):
        for sensor in sensors:
            # Gather the last value for this sensor across all wheels
            sensor_data = [self.data[ethercat_number][sensor][-1] for ethercat_number in self.ethercat_numbers if ethercat_number in self.data and sensor in self.data[ethercat_number]]
            
            # Calculate IQR
            Q1 = np.percentile(sensor_data, 25)
            Q3 = np.percentile(sensor_data, 75)
            IQR = Q3 - Q1
            
            # Define outliers as observations that fall below Q1 - 1.5*IQR or above Q3 + 1.5*IQR
            outlier_condition = (sensor_data < (Q1 - 1.5 * IQR)) | (sensor_data > (Q3 + 1.5 * IQR))
            
            # Identify outliers
            outliers = np.where(outlier_condition)[0]
            outlier_wheels = [self.ethercat_wheel_map.get(self.ethercat_numbers[i], self.ethercat_numbers[i]) for i in outliers]
            print(f"Outlier wheels for sensor {sensor}: {outlier_wheels}")

    def is_contact_loss_dbscan(self, sensors):
        for sensor in sensors:
            # Gather the last value for this sensor across all wheels
            sensor_data = [self.data[ethercat_number][sensor][-1] for   ethercat_number in self.ethercat_numbers if  ethercat_number in self.data and sensor in self.data[ethercat_number]]
            
            # Reshape the data to be compatible with DBSCAN
            sensor_data = np.array(sensor_data).reshape(-1,   1)
            
            # Apply DBSCAN to detect outliers
            dbscan = DBSCAN(eps=0.5, min_samples=2).fit(sensor_data)
            outlier_labels = dbscan.labels_
            
            # Identify outliers (here, wheels with a label of -1)
            outliers = np.where(outlier_labels == -1)[0]
            outlier_wheels = [self.ethercat_wheel_map.get(self.ethercat_numbers[i], self.ethercat_numbers[i]) for i in outliers]
            print(f"Outlier wheels for sensor {sensor}: {outlier_wheels}")


def main(args=None):
    parser = argparse.ArgumentParser(description='Analyze wheel_diag_non_json data.')
    parser.add_argument('--wheels', metavar='number', type=int, nargs='+', help='list of wheel numbers')
    parser.add_argument('--ethercats', metavar='number', type=int, nargs='+', help='list of ethercat numbers')
    parser.add_argument('--sensors', metavar='name', type=str, nargs='+', help='list of sensor names')
    parser.add_argument('--window', metavar='seconds', type=int, help='window size in seconds for the plot')
    parser.add_argument('--yrange', metavar='min max', type=float, nargs=2, help='minimum and maximum y-axis range')
    parser.add_argument('--title', metavar='title', type=str, help='Title for the plot window')
    args = parser.parse_args()

    rclpy.init(args=None)

    loss_contact = LossContact(args.ethercats, args.sensors, args.window, args.wheels, args.yrange, args.title)

    rclpy.spin(loss_contact)

    loss_contact.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
