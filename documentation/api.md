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
  - [Save to CSV](#save-to-csv)
    - [Description](#description-2)
    - [Syntax](#syntax-2)
    - [Parameters](#parameters-2)
    - [Return Value](#return-value-2)
    - [Example](#example-2)
  - [Plot Data](#plot-data)
    - [Description](#description-3)
    - [Syntax](#syntax-3)
    - [Parameters](#parameters-3)
    - [Return Value](#return-value-3)
    - [Example](#example-3)
  - [play_data](#play_data)
    - [Description](#description-4)
    - [Syntax](#syntax-4)
    - [Parameters](#parameters-4)
    - [Seeker Visualization](#seeker-visualization)
    - [Return Value](#return-value-4)
    - [Example](#example-4)
  - [create_label](#create_label)
    - [Description](#description-5)
    - [Syntax](#syntax-5)
    - [Parameters](#parameters-5)
    - [Visualization](#visualization)
    - [Example](#example-5)

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

- **WheelDiag**: This is a custom ROS message used to publish wheel diagnostic data. It contains the following fields:
  - status1
  - status2
  - sensor_ts
  - setpoint_ts
  - encoder_1
  - velocity_1
  - current_1_d
  - current_1_q
  - current_1_u
  - current_1_v
  - current_1_w
  - voltage_1
  - voltage_1_u
  - voltage_1_v
  - voltage_1_w
  - temperature_1
  - encoder_2
  - velocity_2
  - current_2_d
  - current_2_q
  - current_2_u
  - current_2_v
  - current_2_w
  - voltage_2
  - voltage_2_u
  - voltage_2_v
  - voltage_2_w
  - temperature_2
  - encoder_pivot
  - velocity_pivot
  - voltage_bus
  - imu_ts
  - accel_x
  - accel_y
  - accel_z
  - gyro_x
  - gyro_y
  - gyro_z
  - temperature_imu
  - pressure
  - current_in


### Wheel 1:

- **status1**: Status information 1.
- **status2**: Status information 2.
- **sensor_ts**: Timestamp of the sensor data.
- **setpoint_ts**: Timestamp of the setpoint.
- **encoder_1**: Encoder data for wheel 1.
- **velocity_1**: Velocity data for wheel 1.
- **current_1_d**: Current component *d* for wheel 1.
- **current_1_q**: Current component *q* for wheel 1.
- **current_1_u**: Current component *u* for wheel 1.
- **current_1_v**: Current component *v* for wheel 1.
- **current_1_w**: Current component *w* for wheel 1.
- **voltage_1**: Voltage for wheel 1.

### Wheel 2:

- **encoder_2**: Encoder data for wheel 2.
- **velocity_2**: Velocity data for wheel 2.
- **current_2_d**: Current component *d* for wheel 2.
- **current_2_q**: Current component *q* for wheel 2.
- **current_2_u**: Current component *u* for wheel 2.
- **current_2_v**: Current component *v* for wheel 2.
- **current_2_w**: Current component *w* for wheel 2.
- **voltage_2**: Voltage for wheel 2.

### Pivot:

- **encoder_pivot**: Encoder data for the pivot.
- **velocity_pivot**: Velocity data for the pivot.

### General:

- **voltage_bus**: Bus voltage.
- **imu_ts**: Timestamp for IMU data.



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
                # Add more sensors as needed
            },
            'labels': {
                'label1': {
                    'start_time': timestamp1,
                    'end_time': timestamp2,
                    'description': 'Description of the labeled section'
                },
                # Add more labels as needed
            }
        }

## Example

```python
file_path = "path/to/your/rosbag/file.bag"
data = load_rosbag(file_path)
print(data)
```

## Save to CSV

### Description

The `save_to_csv` function is designed to take data from the Wheel Diagnostic Data Visualizer and save it to a CSV file. This function facilitates the export of relevant information to a format compatible with tools like Excel for further analysis and visualization.

### Syntax

```python
def save_to_csv(data: dict, csv_file_path: str) -> None:
```
### Parameters

- **data** (`dict`): A dictionary containing the extracted data, typically obtained from the `load_rosbag` function.

- **csv_file_path** (`str`): The path to the CSV file where the data will be saved.

### Return Value

This function does not return any value. It saves the data to the specified CSV file.

### Example

```python
data = load_rosbag("path/to/your/rosbag/file.bag")
csv_file_path = "path/to/your/csv/output/file.csv"
save_to_csv(data, csv_file_path)
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

# play_data

## Description

The `play_data` function is designed to visually play through ROS bag data, providing a dynamic representation of the data progression over time. It generates a moving bar, simulating a seeker, to indicate the current position in the dataset.

## Syntax

```python
def play_data(data: dict, speed: float = 1.0):
```

## Parameters

- **data** (`dict`): A dictionary containing the extracted data, typically obtained from the `load_rosbag` function.

- **speed** (`float`, optional): The playback speed of the data visualization. Default is `1.0`, representing real-time speed. Values greater than `1.0` accelerate the playback, while values between `0.0` and `1.0` slow it down.


## Seeker Visualization
The seeker is a dynamic visual element that represents the progression of the ROS bag data. It appears as a moving bar, indicating the current position in the dataset as it plays through.

## Return Value
This function does not return any value. It generates a dynamic visualization of the ROS bag data progression.

## Example
```python
data = load_rosbag("path/to/your/rosbag/file.bag")
play_data(data, speed=1.5)
```
In this example, the play_data function is used to visually play through the ROS bag data at 1.5 times the real-time speed.

# create_label

## Description

The `create_label` function allows users to attach notes and visually mark important sections on the graph timeline. This feature is particularly useful for highlighting significant events or anomalies in the diagnostic data.

## Syntax

```python
def create_label(data: dict, start_time: float, end_time: float, color: str = 'red', text: str = ''):
```

## Parameters

- **data** (`dict`): A dictionary containing the extracted data, typically obtained from the `load_rosbag` function.

- **start_time** (`float`): The start time of the labeled section on the graph timeline.

- **end_time** (`float`): The end time of the labeled section on the graph timeline.

- **color** (`str`, optional): The color to use for marking the labeled section. Default is `'red'`. Other available options may include common color names or hexadecimal codes.

- **text** (`str`, optional): A text description explaining the significance of the labeled section. This text will be displayed alongside the marked area on the graph.

## Visualization

The labeled section will be visually highlighted on the graph timeline, using the specified color. The attached text will provide additional context for the marked area.


## Return Value
This function does not return any value. It modifies the graph visualization to include the labeled section.

## Example

```python
data = load_rosbag("path/to/your/rosbag/file.bag")
create_label(data, start_time=10.0, end_time=15.0, color='yellow', text='Acceleration anomaly detected')
```
