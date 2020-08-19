import sys
from math import pi

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor, ParameterType
from std_msgs.msg import Float32
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist
from std_srvs.srv import Trigger

import odrive


class ODriveNode(Node):
    def __init__(self):
        super().__init__('odrive_node')
        self.declare_parameter('axis0_as_right', True, ParameterDescriptor(
            type=ParameterType.PARAMETER_BOOL, description='Use axis0 as right axis?'))
        self.declare_parameter('wheel.track', 0.400, ParameterDescriptor(
            type=ParameterType.PARAMETER_DOUBLE, description='Distance between wheel centers in meter'))
        self.declare_parameter('wheel.radius', 0.165 / 2, ParameterDescriptor(
            type=ParameterType.PARAMETER_DOUBLE, description='Wheel radius in meter'))
        self.declare_parameter('battery.max_voltage', 4.2 * 6, ParameterDescriptor(
            type=ParameterType.PARAMETER_DOUBLE, description='Max battery voltage'))
        self.declare_parameter('battery.min_voltage', 3.2 * 6, ParameterDescriptor(
            type=ParameterType.PARAMETER_DOUBLE, description='Min battery voltage'))
        self.declare_parameter('joint_state.topic', 'joint_state', ParameterDescriptor(
            type=ParameterType.PARAMETER_STRING, description='Joint State publisher topic'))

        self.cmd_vel_subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_listener_callback,
            2
        )

        self.connect_odrive_service = self.create_service(
            Trigger,
            'connect_odrive',
            self.connect_odrive_callback
        )

        self.battery_percentage_publisher_ = self.create_publisher(
            Float32,
            'barrery_percentage',
            1
        )
        self.battery_percentage_publisher_timer = self.create_timer(
            10,
            self.battery_percentage_publisher_callback
        )

        self.joint_state_publisher_ = self.create_publisher(
            JointState,
            self.get_parameter(
                'joint_state.topic').get_parameter_value().string_value,
            10
        )
        self.joint_state_publisher_timer = self.create_timer(
            0.1,
            self.joint_state_publisher_callback
        )

        self.driver: odrive.fibre.remote_object.RemoteObject = None
        self.left_axis = None
        self.right_axis = None

    def distance_per_turn(self):
        return 2 * pi * self.get_parameter('wheel.radius').get_parameter_value().double_value

    def cmd_vel_listener_callback(self, msg: Twist):
        self.get_logger().info(
            f'lin_vel: {msg.linear.x} angle_vel: {msg.angular.z}')
        diff_vel = msg.angular.z * \
            self.get_parameter(
                'wheel.track').get_parameter_value().double_value / 2
        left_turns = (msg.linear.x - diff_vel) / self.distance_per_turn()
        right_turns = (msg.linear.x + diff_vel) / self.distance_per_turn()
        self.left_axis.input_vel = left_turns
        self.right_axis.input_vel = right_turns

        self.left_axis.watchdog_feed()
        self.right_axis.watchdog_feed()

    def connect_odrive_callback(self, request: Trigger.Request, response: Trigger.Response):
        try:
            self.get_logger().info('Connecting to ODrive')
            self.driver = odrive.find_any(timeout=15)
            self.get_logger().info('ODrive connected')
            response.success = True
            response.message = f'Connected to {self.driver.serial_number}'
            if self.get_parameter('axis0_as_right').get_parameter_value().bool_value:
                self.left_axis = self.driver.axis1
                self.right_axis = self.driver.axis0
            else:
                self.left_axis = self.driver.axis0
                self.right_axis = self.driver.axis1
            if not self.driver.user_config_loaded:
                self.get_logger().warn('ODrive user config not loaded')
        except TimeoutError:
            response.success = False
            response.message = 'Timeout'
        except:
            response.success = False
            response.message = 'Unexpected error:', sys.exc_info()[0]

        return response

    def battery_percentage_publisher_callback(self):
        msg = Float32()
        if self.driver:
            msg.data = (
                self.driver.vbus_voltage -
                self.get_parameter(
                    'battery.min_voltage').get_parameter_value().double_value
            ) / (
                self.get_parameter('battery.max_voltage').get_parameter_value().double_value -
                self.get_parameter('battery.min_voltage').get_parameter_value().double_value)
            self.battery_percentage_publisher_.publish(msg)
            if msg.data < 0.2:
                self.get_logger().warn(
                    f'ODrive battery percentage low: {msg.data:0.2f}')
        else:
            self.get_logger().debug('ODrive not connected')

    def joint_state_publisher_callback(self):
        if self.driver:
            msg = JointState()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.position = [self.driver.axis0.encoder.pos_estimate,
                            self.driver.axis1.encoder.pos_estimate]
            msg.velocity = [self.driver.axis0.encoder.vel_estimate,
                            self.driver.axis1.encoder.vel_estimate]
            self.joint_state_publisher_.publish(msg)
        else:
            self.get_logger().debug('ODrive not connected')


def main():
    print('Hi from odrive_ros2.')
    rclpy.init()
    node = ODriveNode()
    rclpy.spin(node)


if __name__ == '__main__':
    main()
