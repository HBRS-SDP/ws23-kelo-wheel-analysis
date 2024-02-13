# WS23 Kelo Wheel Analysis

## Description

WS23 Kelo Wheel Analysis is a Python library and command-line tool designed to analyze wheel data. It provides functionality for generating graphs and performing diagnostics on wheel data, which can be particularly useful for monitoring vehicle performance and identifying potential issues.

## Installation

### Prerequisites

- Python  3.x (ensure you have the correct version installed)
- libraries listed on https://robile-amr.readthedocs.io/en/rolling/getting_started.html

### Installation Steps
1. Follow the instructions: https://robile-amr.readthedocs.io/en/rolling/getting_started.html
1. Clone the repository:
2. ```
   git clone https://github.com/yourusername/ws23-kelo-wheel-analysis.git
   ```
3. build it according to the documentation in step 1

## Usage

### Running the Command-Line Tool

After installation, you can run the command-line tool as follows:
```
ros2 ws23_kelo_wheel_analysis  --wheels 1 2 3 --ethercats 4 5 6 --sensors sensor1 sensor2 --window 10 --yrange 0 100 --title "Wheel Analysis Plot"
```

Replace the `--wheels`, `--ethercats`, `--sensors`, `--window`, `--yrange`, and `--title` options with appropriate values for your setup.

### Using the Library

To use the library in your own Python scripts, you can import the necessary modules and classes:

Initialize the json to non json topic converter 

```
ros2 run ws23_kelo_wheel_analysis wheel_diag_converter
```

Analyze the the graphs using wheel_analysis.py

```
ros2 run ws23_kelo_wheel_analysis wheel_analysis --ethercats 5 3 7 9 --wheels 0 3 1 2 --sensors velocity_1 velocity_2 voltage_1 voltage_2 --window 100 --yrange -6 8 --title 'ideal forward left'
```

Detect Wheel contact loss using loss_contact

```
 ros2 run ws23_kelo_wheel_analysis loss_contact --ethercats 5 3 7 9 --wheels 0 3 1 2 --sensors velocity_1 velocity_2 --window 100 --yrange -6 8 --title 'ideal forward left'
```


