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

class WheelAnalysis(Node):

    def __init__(self,   ethercat_numbers=None, sensors=None, window_size=None, wheels=None, yrange=None, title=None):
        super().__init__('wheel_analysis')
        print("Initializing WheelAnalysis Node...")
        self.subscription = self.create_subscription(
            WheelDiag,
            'wheel_diag_non_json',
            self.listener_callback,
            qos_profile_sensor_data)
        self.subscription  # prevent unused variable warning
        self.yrange = yrange
        self.data = defaultdict(lambda: defaultdict(list))  # {ethercat_number: {sensor: [values]}}
        self.ax_dict = {}  # {ethercat_number: ax}
        self.start_time = time.time()  # Added this line
        self.title = title
        self.ethercat_numbers = ethercat_numbers if ethercat_numbers else []
        self.sensors = sensors if sensors else []
        self.window_size = window_size if window_size else float('inf')  # in seconds
        self.ethercat_wheel_map = dict(zip(ethercat_numbers, wheels)) if  ethercat_numbers and wheels else {}
        self.start_time = time.time()  # Start the timer

        # Create a grid of subplots
        num_plots = len(self.ethercat_numbers)
        num_cols = math.ceil(math.sqrt(num_plots))
        num_rows = math.ceil(num_plots / num_cols)
        self.fig, axs = plt.subplots(num_rows, num_cols, figsize=(19,  10), squeeze=False)  # Set figure size to  800x600
        self.colors = cm.viridis(np.linspace(0,  1, num_plots))  # Generate a bright color for each subplot
        # After creating the figure and axes, set the suptitle
        print(f"Title set to: '{self.title}'")
        # Correct the suptitle method call to use 'color' instead of 'fontcolor'
        self.fig.suptitle(self.title, fontsize=16, fontweight='bold', y=1.05, color='red')
        for ax,   ethercat_number in zip(axs.flat, self.ethercat_numbers):
            wheel_number = self.ethercat_wheel_map.get(ethercat_number, None)
            if wheel_number is not None:
                ax.set_title(f"Wheel Number: {wheel_number}")
                ax.text(0.5, -0.1, f"Wheel Number: {wheel_number}", size=12, ha="center",   
                        transform=ax.transAxes)  # Update title to show wheel number
                self.ax_dict[ethercat_number] = ax
                # Set the y-axis label to the first sensor in the list
                if self.sensors:
                    ax.set_ylabel(self.sensors[0])
                else:
                    ax.set_ylabel('Sensor Data')
            else:
                print(f"No wheel mapping found for EtherCAT number: {ethercat_number}")
        plt.show(block=False)
        print("WheelAnalysis Node initialized.")

    def listener_callback(self, msg):
        print(f"Received message with ethercat_number: {msg.ethercat_number}")
        if msg.ethercat_number in self.ethercat_numbers:
            elapsed_time = time.time() - self.start_time
            for sensor in self.sensors:
                if getattr(msg, sensor) is not None:  # Check if sensor data exists
                    self.data[msg.ethercat_number]['time'].append(elapsed_time)
                    self.data[msg.ethercat_number][sensor].append(getattr(msg, sensor))
            print(f"Updated data for ethercat_number: {msg.ethercat_number}")
            self.plot_data(msg.ethercat_number)


    def plot_data(self,  ethercat_number):
        print("Plotting data...")
        ax = self.ax_dict[ethercat_number]
        ax.clear()
        print(f"Processing sensors: {self.sensors}")  # Print the list of sensors being processed
        y_labels = []
        for idx, sensor in enumerate(self.sensors):
            times = self.data[ethercat_number]['time']
            sensor_data = self.data[ethercat_number][sensor]
            # Keep only data within the window size
            if times[-1] - times[0] > self.window_size:
                start_index = next(i for i, t in enumerate(times) if t - times[0] > self.window_size)
                times = times[start_index:]
                sensor_data = sensor_data[:len(times)]  # Ensure sensor_data has the same length as times
            # Use the index to generate a consistent color for each sensor
            color = self.colors[idx]
            y_labels.append((sensor, color))
            ax.set_xlabel("Time(s)")
            # Ensure times and sensor_data have the same length before plotting
            min_length = min(len(times), len(sensor_data))
            times = times[:min_length]
            sensor_data = sensor_data[:min_length]
            # Create a custom legend with different colors for each sensor
            legend_handles = [Patch(facecolor=color, edgecolor='none', label=label) for label, color in y_labels]
            ax.legend(handles=legend_handles, loc='upper left', bbox_to_anchor=(0,  1), borderaxespad=0.)
            ax.set_ylabel('\t'.join([label for label, _ in y_labels]))  # Join labels with tabs
            y_tick_labels = ['\t'.join([label for label, _ in y_labels])]
            for i, (_, color) in enumerate(y_labels):
                ax.get_yticklabels()[i].set_color(color)  # Set color for each y-tick label
            ax.plot(times, sensor_data, label=sensor, color=color)  # Plot with unique color for subplot

            #ax.legend()
            if self.yrange:
                ax.set_ylim(self.yrange)  # Set the y-axis range
            wheel_number = self.ethercat_wheel_map.get(ethercat_number, None)
            ax.set_title(f"Wheel Number: {wheel_number}")
            self.fig.text(0.5,  0.95, self.title, ha='center', va='baseline', fontsize=16, fontweight='bold', color='red')
            self.fig.canvas.draw()
            self.fig.canvas.flush_events()
            print("Data plotted.")

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

    wheel_analysis = WheelAnalysis(args.ethercats, args.sensors, args.window, args.wheels, args.yrange, args.title)

    rclpy.spin(wheel_analysis)

    wheel_analysis.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
