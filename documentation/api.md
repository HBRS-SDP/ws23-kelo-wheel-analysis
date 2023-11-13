# Table of Contents

1. Modified Kelo Tulip Package API
    - publishWheelDiag
        - Description
        - Syntax
        - Parameters
        - Return Value
        - ROS Topics
        - Custom Data Structures

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
