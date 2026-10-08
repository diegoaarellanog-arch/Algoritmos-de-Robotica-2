"""Convierte las orientaciones de las dos IMU en Hombro y Codo del URDF."""
import math

import rclpy
from geometry_msgs.msg import Vector3Stamped
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger

from .core import wrap_angle


class RobotTeleopNode(Node):
    def __init__(self):
        super().__init__('robot_teleop_node')
        self.declare_parameter('component', 'pitch')
        self.declare_parameter('home_shoulder_deg', 89.0)
        self.declare_parameter('home_elbow_deg', -89.0)
        self.component = self.get_parameter('component').value
        self.home_shoulder = math.radians(self.get_parameter('home_shoulder_deg').value)
        self.home_elbow = math.radians(self.get_parameter('home_elbow_deg').value)
        self.latest = {1: None, 2: None}
        self.reference = {1: None, 2: None}
        self.publisher = self.create_publisher(JointState, '/joint_states', 20)
        self.euler_subscriptions = [self.create_subscription(
            Vector3Stamped, f'/imu{index}/euler',
            lambda message, i=index: self.receive(i, message), 20) for index in (1, 2)]
        self.zero_service = self.create_service(Trigger, '/robot_teleop/zero', self.zero)
        self.timer = self.create_timer(1.0/30.0, self.publish_joints)

    def selected(self, message):
        return {'roll': message.vector.x, 'pitch': message.vector.y,
                'yaw': message.vector.z}.get(self.component, message.vector.y)

    def receive(self, index, message):
        value = self.selected(message)
        self.latest[index] = value
        if self.reference[index] is None:
            self.reference[index] = value

    def zero(self, request, response):
        del request
        self.reference = dict(self.latest)
        response.success = all(value is not None for value in self.reference.values())
        response.message = ('Postura corporal actual tomada como referencia.' if response.success
                            else 'Aún faltan datos de una o ambas IMU.')
        return response

    @staticmethod
    def clamp(value):
        return max(-1.57, min(1.57, value))

    def publish_joints(self):
        if any(self.latest[i] is None or self.reference[i] is None for i in (1, 2)):
            return
        upper_delta = wrap_angle(self.latest[1]-self.reference[1])
        forearm_delta = wrap_angle(self.latest[2]-self.reference[2])
        shoulder = self.clamp(self.home_shoulder + upper_delta)
        elbow = self.clamp(self.home_elbow + wrap_angle(forearm_delta-upper_delta))
        message = JointState()
        message.header.stamp = self.get_clock().now().to_msg()
        message.name = ['Cadera', 'Hombro', 'Codo', 'Antebrazo', 'Muneca', 'DedoIzq']
        message.position = [0.0, shoulder, elbow, 0.0, 0.0, 0.0]
        self.publisher.publish(message)


def main(args=None):
    rclpy.init(args=args)
    node = RobotTeleopNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
