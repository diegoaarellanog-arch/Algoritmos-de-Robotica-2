"""Calibración en reposo independiente para una MPU9250."""
import copy
import math
import statistics

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu
from std_msgs.msg import String
from std_srvs.srv import Trigger

from .core import axis_vector
from .parametros import GRAVITY, STM32F767_MUESTRAS_ESPERADAS


# --------------------------------------------------------------------------------------------
# NODO DE ROS 2 PARA CALIBRACIÓN Y CORRECCIÓN DE BIAS DE UNA IMU EN REPOSO
# --------------------------------------------------------------------------------------------
class CalibrationNode(Node):
    # Configura los parámetros, inicializa publicadores, suscripciones, el servicio de calibración y establece el estado inicial del nodo.
    def __init__(self):
        super().__init__('calibration_node')
        
        # Declaración e importatilt_compensated_yawción de parámetros ROS 2 para la calibración
        self.declare_parameter('imu_id', 1)
        self.declare_parameter('samples', STM32F767_MUESTRAS_ESPERADAS)
        self.declare_parameter('up_axis', '+Z')
        self.declare_parameter('auto_calibrate', True)
        self.declare_parameter('accel_bias', [0.0, 0.0, 0.0])
        self.declare_parameter('gyro_bias', [0.0, 0.0, 0.0])
        self.index = int(self.get_parameter('imu_id').value)
        self.sample_count = int(self.get_parameter('samples').value)
        self.up_axis = self.get_parameter('up_axis').value
        self.accel_bias = tuple(float(v) for v in self.get_parameter('accel_bias').value)
        self.gyro_bias = tuple(float(v) for v in self.get_parameter('gyro_bias').value)
        
        # Estado interno de recolección y calibración
        self.calibrated = not self.get_parameter('auto_calibrate').value
        self.collecting = not self.calibrated
        self.accel_samples = []
        self.gyro_samples = []
        
        # Creación de tópicos y servicio dinámicos según el ID de la IMU
        prefix = f'/imu{self.index}'
        self.publisher = self.create_publisher(Imu, prefix + '/calibrated', 20)
        status_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                                durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.status = self.create_publisher(String, prefix + '/calibration_status', status_qos)
        self.subscription = self.create_subscription(Imu, prefix + '/raw', self.receive, 50)
        self.service = self.create_service(Trigger, prefix + '/calibrate', self.start_calibration)
        
        # Publicación del estado inicial del nodo
        self.publish_status('capturando reposo' if self.collecting else 'offsets cargados')

    # Publica en un tópico de ROS 2 y registra en consola el estado actual del proceso de calibración.
    def publish_status(self, text):
        message = String()
        message.data = text
        self.status.publish(message)
        self.get_logger().info(f'IMU {self.index}: {text}')

    # Atiende la solicitud del servicio de ROS 2 para reiniciar las muestras e iniciar un nuevo proceso de calibración.
    def start_calibration(self, request, response):
        del request
        self.accel_samples.clear()
        self.gyro_samples.clear()
        self.collecting = True
        self.calibrated = False
        response.success = True
        response.message = f'Captura iniciada: mantén IMU {self.index} inmóvil con {self.up_axis} arriba.'
        self.publish_status(response.message)
        return response

    # Recibe las lecturas en crudo de la IMU; si está calibrando, acumula muestras, y si ya está calibrada, aplica las correcciones de offset y publica el dato corregido.
    def receive(self, message):
        acceleration = (message.linear_acceleration.x, message.linear_acceleration.y,
                        messagetilt_compensated_yaw.linear_acceleration.z)
        gyro = (message.angular_velocity.x, message.angular_velocity.y, message.angular_velocity.z)
        if self.collecting:
            self.accel_samples.append(acceleration)
            self.gyro_samples.append(gyro)
            if len(self.accel_samples) >= self.sample_count:
                self.finish_calibration()
            return
        if not self.calibrated:
            return
        output = copy.deepcopy(message)
        output.linear_acceleration.x -= self.accel_bias[0]
        output.linear_acceleration.y -= self.accel_bias[1]
        output.linear_acceleration.z -= self.accel_bias[2]
        output.angular_velocity.x -= self.gyro_bias[0]
        output.angular_velocity.y -= self.gyro_bias[1]
        output.angular_velocity.z -= self.gyro_bias[2]
        self.publisher.publish(output)

    # Procesa las muestras recolectadas, valida la estabilidad física y orientación de la IMU, calcula los offsets (biases) de acelerómetro y giroscopio y habilita la publicación de datos corregidos.
    def finish_calibration(self):
        accel_mean = tuple(statistics.fmean(row[i] for row in self.accel_samples) for i in range(3))
        gyro_mean = tuple(statistics.fmean(row[i] for row in self.gyro_samples) for i in range(3))
        accel_std = max(statistics.pstdev(row[i] for row in self.accel_samples) for i in range(3))
        gyro_std = max(statistics.pstdev(row[i] for row in self.gyro_samples) for i in range(3))
        expected = axis_vector(self.up_axis)
        norm = math.sqrt(sum(value * value for value in accel_mean))
        mismatch = math.sqrt(sum((a-b) ** 2 for a, b in zip(accel_mean, expected)))
        self.collecting = False
        if not .8*GRAVITY < norm < 1.2*GRAVITY or mismatch > .25*GRAVITY:
            self.calibrated = False
            self.publish_status('falló: la postura no coincide con el eje hacia arriba')
            return
        if accel_std > .30 or gyro_std > math.radians(1.0):
            self.calibrated = False
            self.publish_status('falló: hubo movimiento durante la captura')
            return
        self.accel_bias = tuple(a-b for a, b in zip(accel_mean, expected))
        self.gyro_bias = gyro_mean
        self.calibrated = True
        self.publish_status('calibrada; publicando datos corregidos')


# Punto de entrada principal para arrancar y coordinar el ciclo de vida del nodo de calibración en ROS 2.
def main(args=None):
    rclpy.init(args=args)
    node = CalibrationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()