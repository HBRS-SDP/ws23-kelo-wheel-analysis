import rclpy
from rclpy.node import Node
from ws23_kelo_wheel_analysis.msg import WheelDiag
from rclpy.qos import qos_profile_sensor_data
import matplotlib.pyplot as plt
from collections import defaultdict
import argparse
import matplotlib.animation as animation
import time

plt.ion()

class WheelAnalysis(Node):

    def __init__(self, ethercat_numbers=None, sensors=None, window_size=None):
        super().__init__('wheel_analysis')
        print("Initializing WheelAnalysis Node...")
        self.subscription = self.create_subscription(
            WheelDiag,
            'wheel_diag_non_json',
            self.listener_callback,
            qos_profile_sensor_data)
        self.subscription  # prevent unused variable warning

        self.data = defaultdict(lambda: defaultdict(list))  # {ethercat_number: {sensor: [values]}}
        self.fig_dict = {}  # {ethercat_number: (fig, ax)}
        self.start_time = time.time()

        self.ethercat_numbers = ethercat_numbers if ethercat_numbers else []
        self.sensors = sensors if sensors else []
        self.window_size = window_size if window_size else float('inf')  # in seconds
        print("WheelAnalysis Node initialized.")

    def listener_callback(self, msg):
        print(f"Received message with ethercat_number: {msg.ethercat_number}")
        if msg.ethercat_number in self.ethercat_numbers:
            elapsed_time = time.time() - self.start_time
            self.data[msg.ethercat_number]['time'].append(elapsed_time)
            for sensor in self.sensors:
                self.data[msg.ethercat_number][sensor].append(getattr(msg, sensor))
            print(f"Updated data for ethercat_number: {msg.ethercat_number}")
            self.plot_data(msg.ethercat_number)

    def plot_data(self, ethercat_number):
        print("Plotting data...")
        if ethercat_number not in self.fig_dict:
            self.fig_dict[ethercat_number] = plt.subplots()
        fig, ax = self.fig_dict[ethercat_number]
        ax.clear()
        for sensor in self.sensors:
            times = self.data[ethercat_number]['time']
            sensor_data = self.data[ethercat_number][sensor]
            # Keep only data within the window size
            if times[-1] - times[0] > self.window_size:
                start_index = next(i for i, t in enumerate(times) if t - times[0] > self.window_size)
                times = times[start_index:]
                sensor_data = sensor_data[start_index:]
            ax.plot(times, sensor_data, label=sensor)
        ax.legend()
        ax.set_title(f"Data for Ethercat Number: {ethercat_number}")
        fig.canvas.draw()
        print("Data plotted.")



def main(args=None):
    parser = argparse.ArgumentParser(description='Analyze wheel_diag_non_json data.')
    parser.add_argument('--ethercats', metavar='number', type=int, nargs='+', help='list of ethercat numbers')
    parser.add_argument('--sensors', metavar='name', type=str, nargs='+', help='list of sensor names')
    parser.add_argument('--window', metavar='seconds', type=int, help='window size in seconds for the plot')
    args = parser.parse_args()

    rclpy.init(args=None)

    wheel_analysis = WheelAnalysis(args.ethercats, args.sensors, args.window)

    rclpy.spin(wheel_analysis)

    wheel_analysis.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
