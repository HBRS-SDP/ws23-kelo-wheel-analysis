# Table of Contents

- [Modified Kelo Tulip Package API](#modified-kelo-tulip-package-api)
  - [publishWheelDiag](#publishwheeldiag)
    - [Description](#description)
    - [Syntax](#syntax)
    - [Parameters](#parameters)
    - [Return Value](#return-value)
    - [ROS Topics](#ros-topics)
    - [Custom Data Structures](#custom-data-structures)
  - [is_loss_contact](#is_loss_contact)
    - [Description](#description-is_loss_contact)
    - [Syntax](#syntax-is_loss_contact)
    - [Parameters](#parameters-3)
    - [Return Value](#return-value-is_loss_contact)
    - [Example](#example-is_loss_contact)
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

# is_loss_contact

## Description

The `is_loss_contact` function is designed to detect a loss in the wheel contact based on the sensor data from the WheelDiag topic. The specific sensor data and threshold for wheel loss detection are still to be determined (TBD).

## Syntax

```python
def is_loss_contact(WheelDiag: topic) -> bool:
```

## Parameters:
    WheelDiag (topic): The WheelDiag topic data.

### Return Value:
    bool: True if there is a loss in the wheel contact, False otherwise.
## Example

```python
WheelDiag = load_rosbag(file_path)
loss = is_loss_structure(WheelDiag)
print(loss)
```
This will print True if there is a loss in the wheel contact and False otherwise. Replace 'sensor1' and 'threshold' with the actual sensor and threshold values once they are determined.

Please note that the function is currently checking for a placeholder condition as the specific sensor data and threshold for wheel loss detection are still to be determined (TBD).

# Wheel Diagnostic Data Visualizer

The Wheel Diagnostic Data Visualizer is a Python program designed to provide a user-friendly interface for visualizing and analyzing wheel diagnostic data obtained from ROS bag files. It leverages the `load_rosbag` function to import data, allowing users to explore and interpret information related to the Kelo Tulip package's wheel configurations.

## Features

- **Data Loading**: Utilizes the `load_rosbag` function to efficiently load data from ROS bag files containing wheel diagnostic information.

- **Interactive Plotting**: Employs the `plot_data` function to generate interactive plots, enabling users to visualize specific wheel and sensor data in various plot types, such as line plots, scatter plots, bar plots, and histograms.

- **CSV Export**: Provides functionality to save loaded data to a CSV file using the `save_to_csv` function, facilitating data export for external analysis or use in spreadsheet software.

## Usage Example

```python
# Example Usage of the Wheel Diagnostic Data Visualizer

# Load data from a ROS bag file
data = load_rosbag("path/to/your/rosbag/file.bag")

# Select wheels and sensors to plot
wheels_to_plot = [1, 2]
sensors_to_plot = ['velocity', 'temperature']

# Choose the type of plot (e.g., scatter plot)
plot_type_to_use = 'scatter'

# Generate and display the specified plot
plot_data(data, wheels_to_plot, sensors_to_plot, plot_type_to_use)
```
# load_rosbag

## Description

The `load_rosbag` function is designed to load data from a ROS bag file containing wheel diagnostic information. It extracts relevant data, including timestamps, sensor values, and other diagnostic information, and stores it in a dictionary for further processing.

## Syntax

```python
def load_rosbag(file_path: str) -> dict:
```

## Parameters:
    file_path (str): The path to the ROS bag file.

### Return Value:
        dict: A dictionary containing the extracted data with the following structure:
        {
            'timestamps': [list of timestamps],
            'sensor_data': {
                'sensor1': [list of sensor1 values],
                'sensor2': [list of sensor2 values],
                # More sensors
            },
            'labels': {
                'label1': {
                    'start_time': timestamp1,
                    'end_time': timestamp2,
                    'description': 'Description of the labeled section'
                },
                # More sensors
            }
        }

## Example

```python
file_path = "path/to/your/rosbag/file.bag"
data = load_rosbag(file_path)
print(data)
```

# Plot Data

### Description

The `plot_data` function is designed to visualize wheel diagnostic data, allowing users to select specific wheels, sensors, and plot types. This function provides flexibility in exploring and comparing data for different scenarios.

### Syntax

```python
def plot_data(data: dict, wheels: list, sensors: list, plot_type: str) -> None:
```

### Parameters

- **data** (`dict`): A dictionary containing the extracted data, typically obtained from the `load_rosbag` function.

- **wheels** (`list`): A list of wheel numbers (1 to 4) to include in the plot.

- **sensors** (`list`): A list of sensor names to include in the plot. Available sensor options are:
  - 'status1'
  - 'status2'
  - 'sensor_ts'
  - 'setpoint_ts'
  - 'encoder_1'
  - 'velocity_1'
  - 'current_1_d'
  - 'current_1_q'
  - 'current_1_u'
  - 'current_1_v'
  - 'current_1_w'
  - 'voltage_1'
  - 'voltage_1_u'
  - 'voltage_1_v'
  - 'voltage_1_w'
  - 'temperature_1'
  - 'encoder_2'
  - 'velocity_2'
  - 'current_2_d'
  - 'current_2_q'
  - 'current_2_u'
  - 'current_2_v'
  - 'current_2_w'
  - 'voltage_2'
  - 'voltage_2_u'
  - 'voltage_2_v'
  - 'voltage_2_w'
  - 'temperature_2'
  - 'encoder_pivot'
  - 'velocity_pivot'
  - 'voltage_bus'
  - 'imu_ts'
  - 'accel_x'
  - 'accel_y'
  - 'accel_z'
  - 'gyro_x'
  - 'gyro_y'
  - 'gyro_z'
  - 'temperature_imu'
  - 'pressure'
  - 'current_in'

- **plot_type** (`str`): The type of plot to generate. Available options include:
  - 'line': Line plot
  - 'scatter': Scatter plot
  - 'bar': Bar plot
  - 'histogram': Histogram

### Return Value

This function does not return any value. It generates and displays the specified plot based on user-selected parameters.

### Example

```python
data = load_rosbag("path/to/your/rosbag/file.bag")
wheels_to_plot = [1, 2]
sensors_to_plot = ['velocity', 'temperature']
plot_type_to_use = 'scatter'

plot_data(data, wheels_to_plot, sensors_to_plot, plot_type_to_use)
```

In this example, the plot_data function is used to generate a scatter plot for velocity and temperature data from wheels 1 and 2.


