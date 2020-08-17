import sys
from math import pi

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor, ParameterType
from geometry_msgs.msg import Twist
from std_srvs.srv import Trigger

import odrive

class ODriveNode(Node):
    def __init__(self):
        super().__init__('odrive_node')
        self.driver: odrive.fibre.remote_object.RemoteObject = None
        self.declare_parameter('axis0_as_right', True, ParameterDescriptor(type=ParameterType.PARAMETER_BOOL, description='Use axis0 as right axis?'))
        self.declare_parameter('wheel.track', 0.400, ParameterDescriptor(type=ParameterType.PARAMETER_DOUBLE, description='Distance between wheel centers in meter')) # m
        self.declare_parameter('wheel.radius', 0.165 / 2, ParameterDescriptor(type=ParameterType.PARAMETER_DOUBLE, description='Wheel radius in meter'))
        

        self.cmd_vel_subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_listener_callback,
            2)

        self.connect_odrive_service = self.create_service(Trigger, 'connect_odrive', self.connect_odrive_callback)

    def cmd_vel_listener_callback(self, msg: Twist):
        self.get_logger().info(f'lin_vel: {msg.linear.x} angle_vel: {msg.angular.z}')
        diff_vel = msg.angular.z * self.get_parameter('wheel.track').get_parameter_value().double_value / 2
        left_turns = (msg.linear.x - diff_vel) / (2 * pi * self.get_parameter('wheel.radius').get_parameter_value().double_value)
        right_turns = (msg.linear.x + diff_vel) / (2 * pi * self.get_parameter('wheel.radius').get_parameter_value().double_value)
        
        if self.get_parameter('axis0_as_right').get_parameter_value().bool_value:
            self.driver.axis0.input_vel = right_turns
            self.driver.axis1.input_vel = left_turns
        else:
            self.driver.axis0.input_vel = left_turns
            self.driver.axis1.input_vel = left_turns
        

    def connect_odrive_callback(self, request: Trigger.Request, response: Trigger.Response):
        try:
            self.driver = odrive.find_any(timeout=10)
            response.success = True
            response.message = "Connected"
        except TimeoutError:
            response.success = False
            response.message = "Timeout"
        except:
            response.success = False
            response.message = "Unexpected error:", sys.exc_info()[0]

        return response

def main():
    print('Hi from odrive_ros2.')
    rclpy.init()
    node = ODriveNode()
    rclpy.spin(node)


if __name__ == '__main__':
    main()
