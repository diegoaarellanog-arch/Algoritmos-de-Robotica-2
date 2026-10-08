"""Filtro complementario de Roll, Pitch y Yaw para una IMU."""
import math

import rclpy
from geometry_msgs.msg import Vector3Stamped
from rclpy.node import Node
from sensor_msgs.msg import Imu, MagneticField

from .core import tilt_compensated_yaw, wrap_angle


# --------------------------------------------------------------------------------------------
# NODO DE ROS 2 PARA ESTIMACIÓN DE ORIENTACIÓN (EULER) MEDIANTE FILTRO COMPLEMENTARIO
# --------------------------------------------------------------------------------------------
class OrientationNode(Node):
    # Configura los parámetros del filtro complementario, declara las variables de estado, suscribe los sensores e inicializa el publicador de ángulos de Euler.
    def __init__(self):
        super().__init__('orientation_node')
        
        # Declaración de parámetros ROS 2 para la ID de la IMU y constantes del filtro
        self.declare_parameter('imu_id', 1)
        self.declare_parameter('alpha_roll_pitch', 0.60)
        self.declare_parameter('alpha_yaw', 0.90)
        self.declare_parameter('fallback_dt', 0.01)
        
        # Asignación de parámetros a variables locales
        self.index = int(self.get_parameter('imu_id').value)
        self.alpha = float(self.get_parameter('alpha_roll_pitch').value)
        self.alpha_yaw = float(self.get_parameter('alpha_yaw').value)
        self.fallback_dt = float(self.get_parameter('fallback_dt').value)
        
        # Estado interno de la orientación (Roll, Pitch, Yaw) y registros de tiempo
        self.roll = self.pitch = self.yaw = None
        self.last_stamp = None
        self.magnetic = None
        
        # Configuración del publicador de resultados y suscripciones a tópicos de sensores
        prefix = f'/imu{self.index}'
        self.publisher = self.create_publisher(Vector3Stamped, prefix + '/euler', 20)
        self.imu_subscription = self.create_subscription(
            Imu, prefix + '/calibrated', self.receive_imu, 50)
        self.mag_subscription = self.create_subscription(
            MagneticField, prefix + '/magnetic_field', self.receive_mag, 50)

    # Actualiza el estado interno guardando la última lectura recibida del magnetómetro.
    def receive_mag(self, message):
        self.magnetic = (message.magnetic_field.x, message.magnetic_field.y,
                         message.magnetic_field.z)

    # Calcula el delta de tiempo, estima los ángulos de inclinación y orientación fusionando giroscopio, acelerómetro y magnetómetro, y publica los ángulos de Euler finales (Roll, Pitch, Yaw).
    def receive_imu(self, message):
        ax, ay, az = (message.linear_acceleration.x, message.linear_acceleration.y,
                      message.linear_acceleration.z)
        gx, gy, gz = (message.angular_velocity.x, message.angular_velocity.y,
                      message.angular_velocity.z)
        
        # Ignora la muestra si los datos del acelerómetro son nulos o no válidos
        if math.sqrt(ax*ax + ay*ay + az*az) < 1e-8:
            return
            
        # Estimación de Roll y Pitch a partir del vector de aceleración inercial
        roll_acc = math.atan2(ay, az)
        pitch_acc = math.atan2(-ax, math.hypot(ay, az))
        
        # Cálculo y validación del intervalo de tiempo (dt) entre lecturas
        stamp = message.header.stamp.sec + message.header.stamp.nanosec * 1e-9
        dt = self.fallback_dt if self.last_stamp is None else stamp - self.last_stamp
        if not 0.0001 <= dt <= 0.2:
            dt = self.fallback_dt
        self.last_stamp = stamp
        
        # Actualización de Roll y Pitch aplicando el filtro complementario (integración del giroscopio + corrección del acelerómetro)
        if self.roll is None:
            self.roll, self.pitch = roll_acc, pitch_acc
        else:
            roll_predicted = wrap_angle(self.roll + gx*dt)
            roll_error = wrap_angle(roll_acc-roll_predicted)
            self.roll = wrap_angle(roll_predicted + (1.0-self.alpha)*roll_error)
            self.pitch = self.alpha*(self.pitch+gy*dt) + (1.0-self.alpha)*pitch_acc
            
        # Estimación del Yaw compensado por inclinación mediante las lecturas del magnetómetro
        yaw_mag = (tilt_compensated_yaw(self.magnetic, self.roll, self.pitch)
                   if self.magnetic is not None else None)
                   
        # Actualización de Yaw integrando la velocidad angular y corrigiendo con la brújula magnética
        if self.yaw is None:
            self.yaw = yaw_mag if yaw_mag is not None else 0.0
        else:
            predicted = wrap_angle(self.yaw + gz*dt)
            self.yaw = (predicted if yaw_mag is None else
                        wrap_angle(predicted + (1.0-self.alpha_yaw)*wrap_angle(yaw_mag-predicted)))
                        
        # Construcción del mensaje estamado y publicación de la orientación estimada
        output = Vector3Stamped()
        output.header = message.header
        output.vector.x = self.roll
        output.vector.y = self.pitch
        output.vector.z = self.yaw
        self.publisher.publish(output)


# Punto de entrada principal para arrancar y coordinar el ciclo de vida del nodo de orientación en ROS 2.
def main(args=None):
    rclpy.init(args=args)
    node = OrientationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()