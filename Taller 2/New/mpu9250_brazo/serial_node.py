"""Lee las tramas de 20 campos de la STM32 y publica ambas MPU9250."""

# --------------------------------------------------------------------------------------------
# LIBRERIAS
# --------------------------------------------------------------------------------------------
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, MagneticField
import serial

from .core import parse_dual_line, raw_to_si, split_imus

# --------------------------------------------------------------------------------------------
# CONFIGURACIÓN Y CONSTANTES POR DEFECTO
# Configuración inicial del puerto serie, lecturas e IMUs.
# --------------------------------------------------------------------------------------------
UART_PUERTO_STM = '/dev/ttyACM0'
UART_BAUDIOS = 9600
UART_COMANDO = 'H'
UART_TIEMPO_INTERVALO = 1.5

MPU_ESCALA_ACEL = 2.0
MPU_ESCALA_GIRO = 250.0

TOTAL_IMUS = 2

# --------------------------------------------------------------------------------------------
# NODO DE ROS 2 PARA COMUNICACIÓN SERIAL CON STM32
# --------------------------------------------------------------------------------------------
class Stm32SerialNode(Node):
    # Inicializa el nodo, configura los parámetros, registra los publicadores ROS 2 y activa el temporizador periódico.
    def __init__(self):
        super().__init__('stm32_serial_node')

        # Declaración de parámetros ROS 2 configurables
        self.declare_parameter('port', UART_PUERTO_STM)
        self.declare_parameter('baud', UART_BAUDIOS)
        self.declare_parameter('accel_range_g', MPU_ESCALA_ACEL)
        self.declare_parameter('gyro_range_dps', MPU_ESCALA_GIRO)
        self.declare_parameter('request_command', UART_COMANDO)
        self.declare_parameter('request_interval_s', UART_TIEMPO_INTERVALO)

        # Asignación de variables de estado y parámetros de trabajo
        self.port = self.get_parameter('port').value
        self.baud = self.get_parameter('baud').value
        self.accel_range = self.get_parameter('accel_range_g').value
        self.gyro_range = self.get_parameter('gyro_range_dps').value
        self.request = self.get_parameter('request_command').value.encode('ascii')
        self.request_interval = self.get_parameter('request_interval_s').value

        self.serial = None
        self.pending = b''
        self.last_valid = 0.0
        self.last_request = 0.0
        self.next_open = 0.0
        self.invalid = 0
        self.imu_publishers = {}
        self.mag_publishers = {}

        # Creación de publicadores para los temas de datos inerciales y campo magnético de cada IMU
        for index in range(1, TOTAL_IMUS + 1):
            self.imu_publishers[index] = self.create_publisher(Imu, f'/imu{index}/raw', 20)
            self.mag_publishers[index] = self.create_publisher(
                MagneticField, f'/imu{index}/magnetic_field', 20)

        # Creación del temporizador para ejecutar la verificación de datos periódicamente
        self.timer = self.create_timer(0.01, self.poll)
        
        self.get_logger().info(f'Preparado para {self.port} a {self.baud} baudios.')

    # Abre el puerto de comunicación serial con la STM32 si no está conectado.
    def open_port(self):
        now = time.monotonic()
        if self.serial is not None or now < self.next_open:
            return
        try:
            self.serial = serial.Serial(self.port, self.baud, timeout=0, write_timeout=.5)
            self.pending = b''
            self.last_valid = now
            self.last_request = now - self.request_interval + 1.0
            self.get_logger().info(f'Puerto {self.port} abierto.')
        except (OSError, serial.SerialException) as error:
            self.next_open = now + 2.0
            self.get_logger().error(f'No se pudo abrir {self.port}: {error}')

    # Envía el comando a la STM32 para solicitar una nueva ráfaga de datos.
    def request_batch(self, now):
        if now - self.last_valid < self.request_interval or now - self.last_request < self.request_interval:
            return
        self.serial.write(self.request)
        self.last_request = now
        self.get_logger().info('Comando H enviado; esperando lote de la STM32.')

    # Revisa el puerto serial, solicita datos, extrae tramas completas y las distribuye para su publicación.
    def poll(self):
        self.open_port()
        if self.serial is None:
            return
        now = time.monotonic()
        try:
            self.request_batch(now)
            available = self.serial.in_waiting
            if available:
                self.pending += self.serial.read(min(available, 8192))
            if len(self.pending) > 65536:
                self.pending = b''
                raise ValueError('Búfer descartado: no se encontraron saltos de línea.')
            while b'\n' in self.pending:
                raw_line, self.pending = self.pending.split(b'\n', 1)
                line = raw_line.decode('utf-8', errors='replace').strip()
                try:
                    values = parse_dual_line(line)
                except (ValueError, OverflowError):
                    self.invalid += 1
                    continue
                stamp = self.get_clock().now().to_msg()
                for index, separated in enumerate(split_imus(values), start=1):
                    self.publish_imu(index, separated, stamp)
                self.last_valid = time.monotonic()
        except (OSError, ValueError, serial.SerialException) as error:
            self.get_logger().error(f'Error serial: {error}')
            try:
                self.serial.close()
            except Exception:
                pass
            self.serial = None
            self.next_open = time.monotonic() + 2.0

    # Convierte las lecturas de una IMU a unidades estándar y emite los mensajes ROS 2 correspondiente.
    def publish_imu(self, index, values, stamp):
        acceleration, gyro, magnetic = raw_to_si(values, self.accel_range, self.gyro_range)
        frame = f'imu{index}_link'
        imu = Imu()
        imu.header.stamp = stamp
        imu.header.frame_id = frame
        imu.orientation_covariance[0] = -1.0
        imu.linear_acceleration.x, imu.linear_acceleration.y, imu.linear_acceleration.z = acceleration
        imu.angular_velocity.x, imu.angular_velocity.y, imu.angular_velocity.z = gyro
        self.imu_publishers[index].publish(imu)
        mag = MagneticField()
        mag.header = imu.header
        mag.magnetic_field.x, mag.magnetic_field.y, mag.magnetic_field.z = magnetic
        self.mag_publishers[index].publish(mag)

    # Cierra el puerto serial de forma segura antes de destruir el nodo.
    def destroy_node(self):
        if self.serial is not None:
            self.serial.close()
        return super().destroy_node()


# Punto de entrada principal para arrancar y coordinar el ciclo de vida del nodo en ROS 2.
def main(args=None):
    rclpy.init(args=args)
    node = Stm32SerialNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()