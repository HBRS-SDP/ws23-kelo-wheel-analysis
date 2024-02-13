# Table of Contents

- [Modified Kelo Tulip Package API](#modified-kelo-tulip-package-api)
  - [publishWheelDiag](#publishwheeldiag)
    - [Description](#description)
    - [Syntax](#syntax)
    - [Parameters](#parameters)
    - [Return Value](#return-value)
    - [ROS Topics](#ros-topics)
    - [Custom Data Structures](#custom-data-structures)

- [Wheel Diagnostic Data Visualizer](#wheel-diagnostic-data-visualizer)
  - [Description](#description-1)
  - [Syntax](#syntax-1)
  - [Parameters](#parameters-1)
  - [Return Value](#return-value-1)
  - [Example](#example-1)
- [load_rosbag](#load_rosbag)
  - [Description](#description-3)
  - [Syntax](#syntax-3)
  - [Parameters](#parameters-3)
  - [Return Value](#return-value-3)
  - [Example](#example-3)
- [Plot Data](#plot-data)
  - [Description](#description-3)
  - [Syntax](#syntax-3)
  - [Parameters](#parameters-3)
  - [Return Value](#return-value-3)
  - [Example](#example-3)
- [Wheel Diagnostic Converter](#wheel-diagnostic-converter)
  - [Class Overview](#class-overview)
  - [Features](#features)
  - [Methods](#methods)
  - [Usage](#usage)
- [Wheel Analysis](#wheel-analysis)
  - [Class Overview](#class-overview-1)
  - [Features](#features-1)
  - [Methods](#methods-1)
  - [Usage](#usage-1)
- [Loss Contact Detection](#loss-contact-detection)
  - [Class Overview](#class-overview-2)
  - [Features](#features-2)
  - [Methods](#methods-2)
  - [Usage](#usage-2)

# Modified Kelo Tulip Package API

## publishWheelDiag

### Description

The `publishWheelDiag` function is used to publish wheel diagnostic data in the Kelo Tulip package. It iterates over each wheel configuration and publishes a `WheelDiag` message with the process data for the current wheel.

### Syntax

```cpp
void publishWheelDiag()
```
### Parameters
This function does not take any parameters.

### Return Value
This function does not return a value. It publishes a WheelDiag message for each wheel configuration.

### ROS Topics

- **Publishes to**: The `publishWheelDiag` function publishes `WheelDiag` messages to the `"wheel_diag"` ROS topic. The specific data that gets published depends on the state of each wheel at the time the function is called.

### Custom Data Structures

#### Wheel 1:

- **status1**: Status bits as defined in STAT1_
- **status2**: Status bits as defined in STAT2_
- **sensor_ts**: EtherCAT timestamp (ns) on sensor acquisition 
- **setpoint_ts**: EtherCAT timestamp (ns) of last setpoint data
- **encoder_1**: Encoder 1 value in rad (no wrapping at 2PI)
- **velocity_1**: Encoder 1 velocity in rad/s
- **current_1_d**: Motor 1 current direct in amp
- **current_1_q**: Motor 1 current quadrature in amp
- **current_1_u**: Motor 1 current phase U in amp
- **current_1_v**: Motor 1 current phase V in amp
- **current_1_w**: Motor 1 current phase W in amp
- **voltage_1**: Motor 1 voltage from pwm in volts
- **voltage_1_u**: Motor 1 voltage from phase U in volts
- **voltage_1_v**: Motor 1 voltage from phase V in volts
- **voltage_1_w**: Motor 1 voltage from phase W in volts
- **temperature_1**: Motor 1 estimated temperature in K

#### Wheel 2:

- **encoder_2**: Encoder 2 value in rad (no wrapping at 2PI)
- **velocity_2**: Encoder 2 velocity in rad/s
- **current_2_d**: Motor 2 current direct in amp
- **current_2_q**: Motor 2 current quadrature in amp
- **current_2_u**: Motor 2 current phase U in amp
- **current_2_v**: Motor 2 current phase V in amp
- **current_2_w**: Motor 2 current phase W in amp
- **voltage_2**: Motor 2 voltage from pwm in volts
- **voltage_2_u**: Motor 2 voltage from phase U in volts
- **voltage_2_v**: Motor 2 voltage from phase V in volts
- **voltage_2_w**: Motor 2 voltage from phase W in volts
- **temperature_2**: Motor 2 estimated temperature in K

#### Pivot:

- **encoder_pivot**: Encoder pivot value in rad (wrapping at -PI and +PI)
- **velocity_pivot**: Encoder pivot velocity in rad/s

#### General:

- **voltage_bus**: Bus voltage in volts
- **imu_ts**: EtherCAT timestamp (ns) of IMU sensor acquisition
- **accel_x**: IMU accelerometer X-axis in m/s2
- **accel_y**: IMU accelerometer Y-axis in m/s2
- **accel_z**: IMU accelerometer Z-axis in m/s2
- **gyro_x**: IMU gyro X-axis in rad/s
- **gyro_y**: IMU gyro Y-axis in rad/s
- **gyro_z**: IMU gyro Z-axis in rad/s
- **temperature_imu**: IMU temperature in K	
- **pressure**: Barometric pressure in Pa absolute
- **current_in**: Current input


# Wheel Diagnostic Converter

The Wheel Diagnostic Converter is a ROS node implemented in Python that converts JSON formatted wheel diagnostic data into a custom ROS message format. This conversion allows for easier integration with other ROS nodes and tools that expect data in the standard ROS message format.

## Class Overview

- **Name**: `WheelDiagConverter`
- **Purpose**: To convert JSON wheel diagnostic data into a ROS message format.
- **Implementation**: Defined in the `wheel_diag_converter.py` file.

## Features

- **JSON to ROS Message Conversion**: Transforms JSON data received from a ROS topic into a `WheelDiag` message, which includes all the necessary fields for wheel diagnostic data.
- **Message Publishing**: Publishes the converted `WheelDiag` messages to a new ROS topic, allowing subscribers to receive the data in a format compatible with ROS tools and services.

## Methods

### `__init__(self)`

Initializes the `WheelDiagConverter` node, creating a publisher for the `WheelDiag` messages and a subscription to the incoming JSON data.

### `listener_callback(self, msg)`

Callback method that is triggered when a new message is received on the `'wheel_diag'` topic. It processes the JSON data and converts it into a `WheelDiag` message before publishing it to the `'wheel_diag_non_json'` topic.

## Usage

To run the `WheelDiagConverter`, execute the script with the appropriate ROS environment setup. The node will automatically subscribe to the `'wheel_diag'` topic and begin listening for JSON messages to convert.

# Wheel Analysis

The Wheel Analysis is a ROS node implemented in Python that provides a user-friendly interface for visualizing and analyzing wheel diagnostic data obtained from ROS bag files. It leverages the `load_rosbag` function to import data, allowing users to explore and interpret information related to the Kelo Tulip package's wheel configurations.

## Class Overview

- **Name**: `WheelAnalysis`
- **Purpose**: To visualize and analyze wheel diagnostic data from ROS bag files.
- **Implementation**: Defined in the `wheel_analysis.py` file.

## Features

- **Data Loading**: Utilizes the `load_rosbag` function to efficiently load data from ROS bag files containing wheel diagnostic information.
- **Interactive Plotting**: Employs the `plot_data` function to generate interactive plots, enabling users to visualize specific wheel and sensor data in various plot types, such as line plots, scatter plots, bar plots, and histograms.
- **CSV Export**: Provides functionality to save loaded data to a CSV file using the `save_to_csv` function, facilitating data export for external analysis or use in spreadsheet software.

## Methods

### `__init__(self,   ethercat_numbers=None, sensors=None, window_size=None, wheels=None, yrange=None, title=None)`

Initializes the `WheelAnalysis` node with optional parameters for specifying the EtherCAT numbers, sensors, window size, wheels, y-axis range, and title for the plot.

### `cmd_vel_callback(self, msg)`

Callback method that is triggered when a new message is received on the `'/cmd_vel'` topic. It stores the message for later use in plotting.

### `listener_callback(self, msg)`

Callback method that is triggered when a new message is received on the `'wheel_diag_non_json'` topic. It processes the `WheelDiag` message and updates the plot with the new data.

### `plot_data(self,   ethercat_number)`

Method that generates and updates the plot with the latest data for the specified EtherCAT number.

## Usage Example

### Syntax

```python
#Example Usage of the Wheel Analysis
#Load data from a ROS bag file
data = load_rosbag("path/to/your/rosbag/file.bag")


#Select wheels and sensors to plot
wheels_to_plot = [1, 2] sensors_to_plot = ['velocity', 'temperature']

#Choose the type of plot (e.g., scatter plot)
plot_type_to_use = 'scatter'

#Generate and display the specified plot
plot_data(data, wheels_to_plot, sensors_to_plot, plot_type_to_use)
```
## Usage
To run the `WheelAnalysis`, execute the script with the appropriate ROS environment setup. The node will automatically subscribe to the `'wheel_diag_non_json'` topic and begin listening for `WheelDiag` messages to visualize.

# Loss Contact Detection

The Loss Contact Detection is a ROS node implemented in Python that detects loss of contact in wheel diagnostic data. It uses statistical methods and clustering algorithms to identify outliers in sensor data that may indicate a loss of contact between the wheel and the ground.

## Class Overview

- **Name**: `LossContact`
- **Purpose**: To detect loss of contact in wheel diagnostic data based on sensor readings.
- **Implementation**: Defined in the `loss_contact.py` file.

## Features

- **Data Loading**: Utilizes the `load_rosbag` function to efficiently load data from ROS bag files containing wheel diagnostic information.
- **Interactive Plotting**: Employs the `plot_data` function to generate interactive plots, enabling users to visualize specific wheel and sensor data in various plot types, such as line plots, scatter plots, bar plots, and histograms.
- **Contact Loss Detection**: Implements methods to detect loss of contact using statistical outlier detection and clustering algorithms.

## Methods

### `__init__(self,  ethercat_numbers=None, sensors=None, window_size=None, wheels=None, yrange=None, title=None)`

Initializes the `LossContact` node with optional parameters for specifying the EtherCAT numbers, sensors, window size, wheels, y-axis range, and title for the plot.

### `cmd_vel_callback(self, msg)`

Callback method that is triggered when a new message is received on the `'/cmd_vel'` topic. It stores the message for later use in plotting.

### `listener_callback(self, msg)`

Callback method that is triggered when a new message is received on the `'wheel_diag_non_json'` topic. It processes the `WheelDiag` message and updates the plot with the new data.

### `plot_data(self,  ethercat_number)`

Method that generates and updates the plot with the latest data for the specified EtherCAT number.

### `is_contact_loss_iqr(self, sensors, n, std_threshold)`

Method that detects loss of contact using the Interquartile Range (IQR) method. It calculates the IQR for the latest sensor data and identifies outliers that may indicate a loss of contact.

### `is_contact_loss_dbscan(self, sensors)`

Method that detects loss of contact using the DBSCAN clustering algorithm. It clusters the sensor data and identifies outliers that may indicate a loss of contact.

## Usage Example
```python

#Example Usage of the Loss Contact Detection
#Load data from a ROS bag file
data = load_rosbag("path/to/your/rosbag/file.bag")

#Select wheels and sensors to plot
wheels_to_plot = [1, 2] sensors_to_plot = ['velocity', 'temperature']

#Choose the type of plot (e.g., scatter plot)
plot_type_to_use = 'scatter'

#Generate and display the specified plot
plot_data(data, wheels_to_plot, sensors_to_plot, plot_type_to_use)

#Detect loss of contact using IQR method
is_contact_loss_iqr(sensors_to_plot, 10, 0.3)

#Detect loss of contact using DBSCAN method
is_contact_loss_dbscan(sensors_to_plot)
```

## Usage

To run the `LossContact` detection, execute the script with the appropriate ROS environment setup. The node will automatically subscribe to the `'wheel_diag_non_json'` topic and begin listening for `WheelDiag` messages to detect loss of contact.

bash rosrun ws23_kelo_wheel_analysis loss_contact.py