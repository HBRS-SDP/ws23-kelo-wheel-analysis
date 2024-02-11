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

class WheelAnalysis(Node):

    def __init__(self,   ethercat_numbers=None, sensors=None, window_size=None, wheels=None):
        super().__init__('wheel_analysis')
        print("Initializing WheelAnalysis Node...")
        self.subscription = self.create_subscription(
            WheelDiag,
            'wheel_diag_non_json',
            self.listener_callback,
            qos_profile_sensor_data)
        self.subscription  # prevent unused variable warning

        self.data = defaultdict(lambda: defaultdict(list))  # {ethercat_number: {sensor: [values]}}
        self.ax_dict = {}  # {ethercat_number: ax}
        self.start_time = time.time()  # Added this line

        self.ethercat_numbers = ethercat_numbers if ethercat_numbers else []
        self.sensors = sensors if sensors else []
        self.window_size = window_size if window_size else float('inf')  # in seconds
        self.ethercat_wheel_map = dict(zip(ethercat_numbers, wheels)) if  ethercat_numbers and wheels else {}
        self.start_time = time.time()  # Start the timer

        # Create a grid of subplots
        num_plots = len(self.ethercat_numbers)
        num_cols = math.ceil(math.sqrt(num_plots))
        num_rows = math.ceil(num_plots / num_cols)
        self.fig, axs = plt.subplots(num_rows, num_cols, figsize=(8,  6), squeeze=False)  # Set figure size to  800x600
        self.color_dict = {sensor: cm.rainbow(i / len(self.sensors)) for i, sensor in enumerate(self.sensors)}
        colors = cm.rainbow(np.linspace(0,  1, len(self.sensors)))  # Generate a color for each sensor
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


    def plot_data(self,   ethercat_number):
        print("Plotting data...")
        ax = self.ax_dict[ethercat_number]
        ax.clear()
        for sensor in self.sensors:
            times = self.data[ethercat_number]['time']
            sensor_data = self.data[ethercat_number][sensor]
            # Keep only data within the window size
            if times[-1] - times[0] > self.window_size:
                start_index = next(i for i, t in enumerate(times) if t - times[0] > self.window_size)
                times = times[start_index:]
                sensor_data = sensor_data[:len(times)]  # Ensure sensor_data has the same length as times
            ax.set_ylabel(sensor)
            ax.plot(times, sensor_data, label=sensor, color=self.color_dict[sensor])  # Plot with sensor-specific color

        ax.legend()
        wheel_number = self.ethercat_wheel_map.get(ethercat_number, None)
        ax.set_title(f"Wheel Number: {wheel_number}")
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()
        print("Data plotted.")

def main(args=None):
    parser = argparse.ArgumentParser(description='Analyze wheel_diag_non_json data.')
    parser.add_argument('--wheels', metavar='number', type=int, nargs='+', help='list of wheel numbers')
    parser.add_argument('--ethercats', metavar='number', type=int, nargs='+', help='list of ethercat numbers')
    parser.add_argument('--sensors', metavar='name', type=str, nargs='+', help='list of sensor names')
    parser.add_argument('--window', metavar='seconds', type=int, help='window size in seconds for the plot')
    args = parser.parse_args()

    rclpy.init(args=None)

    wheel_analysis = WheelAnalysis(args.ethercats, args.sensors, args.window, args.wheels)

    rclpy.spin(wheel_analysis)

    wheel_analysis.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
