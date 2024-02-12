import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from ws23_kelo_wheel_analysis.msg import WheelDiag
import json

class WheelDiagConverter(Node):

    def __init__(self):
        super().__init__('wheel_diag_converter')
        self.publisher_ = self.create_publisher(WheelDiag, 'wheel_diag_non_json', 10)
        self.subscription = self.create_subscription(
            String,
            'wheel_diag',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        data = json.loads(msg.data)
        wheel_diag_msg = WheelDiag()
        wheel_diag_msg.status1 = data['status1']
        wheel_diag_msg.status2 = data['status2']
        wheel_diag_msg.sensor_ts = data['sensor_ts']
        wheel_diag_msg.setpoint_ts = data['setpoint_ts']
        wheel_diag_msg.encoder_1 = data['encoder_2']
        wheel_diag_msg.velocity_1 = data['velocity_2']
        wheel_diag_msg.current_1_d = data['current_2_d']
        wheel_diag_msg.current_1_q = data['current_2_q']
        wheel_diag_msg.current_1_u = data['current_2_u']
        wheel_diag_msg.current_1_v = data['current_2_v']
        wheel_diag_msg.current_1_w = data['current_2_w']
        wheel_diag_msg.voltage_1 = data['voltage_2']
        wheel_diag_msg.voltage_1_u = data['voltage_2_u']
        wheel_diag_msg.voltage_1_v = data['voltage_2_v']
        wheel_diag_msg.voltage_1_w = data['voltage_2_w']
        wheel_diag_msg.temperature_1 = data['temperature_2']
        wheel_diag_msg.encoder_2 = data['encoder_2']
        wheel_diag_msg.velocity_2 = data['velocity_2']
        wheel_diag_msg.current_2_d = data['current_2_d']
        wheel_diag_msg.current_2_q = data['current_2_q']
        wheel_diag_msg.current_2_u = data['current_2_u']
        wheel_diag_msg.current_2_v = data['current_2_v']
        wheel_diag_msg.current_2_w = data['current_2_w']
        wheel_diag_msg.voltage_2 = data['voltage_2']
        wheel_diag_msg.voltage_2_u = data['voltage_2_u']
        wheel_diag_msg.voltage_2_v = data['voltage_2_v']
        wheel_diag_msg.voltage_2_w = data['voltage_2_w']
        wheel_diag_msg.temperature_2 = data['temperature_2']
        wheel_diag_msg.encoder_pivot = data['encoder_pivot']
        wheel_diag_msg.velocity_pivot = data['velocity_pivot']
        wheel_diag_msg.voltage_bus = data['voltage_bus']
        wheel_diag_msg.imu_ts = data['imu_ts']
        wheel_diag_msg.accel_x = data['accel_x']
        wheel_diag_msg.accel_y = data['accel_y']
        wheel_diag_msg.accel_z = data['accel_z']
        wheel_diag_msg.gyro_x = data['gyro_x']
        wheel_diag_msg.gyro_y = data['gyro_y']
        wheel_diag_msg.gyro_z = data['gyro_z']
        wheel_diag_msg.temperature_imu = data['temperature_imu']
        wheel_diag_msg.pressure = data['pressure']
        wheel_diag_msg.current_in = data['current_in']
        wheel_diag_msg.ethercat_number = data['ethercat_number']
        wheel_diag_msg.x = data['x']
        wheel_diag_msg.y = data['y']
        wheel_diag_msg.a = data['a']
        wheel_diag_msg.critical = data['critical']
        wheel_diag_msg.reverse_velocity = data['reverse_velocity']
        self.publisher_.publish(wheel_diag_msg)


def main(args=None):
    rclpy.init(args=args)

    wheel_diag_converter = WheelDiagConverter()

    rclpy.spin(wheel_diag_converter)

    wheel_diag_converter.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
