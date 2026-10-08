"""GUI ROS2 para dos MPU9250, estado de nodos y articulaciones del brazo 2R."""
from collections import deque
import math
import sys
import time

from PyQt5 import QtCore, QtWidgets
import pyqtgraph as pg
import rclpy
from geometry_msgs.msg import Vector3Stamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu, JointState, MagneticField
from std_msgs.msg import String
from std_srvs.srv import Trigger


EXPECTED_NODES = (
    ('stm32_serial_node', 'UART STM32 y separación de las dos IMU', '/imu1/raw, /imu2/raw'),
    ('imu1_calibration_node', 'Offsets y escala de IMU 1', '/imu1/calibrated'),
    ('imu2_calibration_node', 'Offsets y escala de IMU 2', '/imu2/calibrated'),
    ('imu1_orientation_node', 'Filtro Roll, Pitch y Yaw de IMU 1', '/imu1/euler'),
    ('imu2_orientation_node', 'Filtro Roll, Pitch y Yaw de IMU 2', '/imu2/euler'),
    ('robot_teleop_node', 'Mapeo corporal a Hombro y Codo', '/joint_states'),
    ('imu_gui_node', 'Monitor y gráficas ROS2', 'Interfaz'),
)

MEMBERS = (
    'Diego Alejandro Arellano Gutierrez - 110847',
    'Edwar Felipe García Patiño - 142911',
    'Javier Bohorquez Gaitán - 98587',
    'Johan Montejo - 124077',
    'Sergio Iván Jaimes Garzón - 133238',
)


def stamp_seconds(header):
    return header.stamp.sec + header.stamp.nanosec * 1e-9


class RosGuiNode(Node):
    def __init__(self):
        super().__init__('imu_gui_node')
        names = ('raw_accel', 'accel', 'gyro', 'mag', 'euler')
        self.series = {index: {name: deque(maxlen=300) for name in names} for index in (1, 2)}
        self.arrivals = {1: deque(maxlen=100), 2: deque(maxlen=100)}
        self.calibration = {1: 'esperando', 2: 'esperando'}
        self.joints = {}
        self.ros_subscriptions = []
        status_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                                durability=DurabilityPolicy.TRANSIENT_LOCAL)
        for index in (1, 2):
            prefix = f'/imu{index}'
            self.ros_subscriptions.extend([
                self.create_subscription(Imu, prefix + '/raw',
                    lambda msg, i=index: self.receive_raw(i, msg), 50),
                self.create_subscription(Imu, prefix + '/calibrated',
                    lambda msg, i=index: self.receive_calibrated(i, msg), 50),
                self.create_subscription(MagneticField, prefix + '/magnetic_field',
                    lambda msg, i=index: self.receive_mag(i, msg), 50),
                self.create_subscription(Vector3Stamped, prefix + '/euler',
                    lambda msg, i=index: self.receive_euler(i, msg), 50),
                self.create_subscription(String, prefix + '/calibration_status',
                    lambda msg, i=index: self.receive_status(i, msg), status_qos),
            ])
        self.ros_subscriptions.append(
            self.create_subscription(JointState, '/joint_states', self.receive_joints, 50))
        self.calibration_clients = {
            'imu1': self.create_client(Trigger, '/imu1/calibrate'),
            'imu2': self.create_client(Trigger, '/imu2/calibrate'),
            'zero': self.create_client(Trigger, '/robot_teleop/zero'),
        }

    def append(self, index, name, stamp, values):
        self.series[index][name].append((stamp, tuple(values)))

    def receive_raw(self, index, message):
        self.append(index, 'raw_accel', stamp_seconds(message.header),
                    (message.linear_acceleration.x, message.linear_acceleration.y,
                     message.linear_acceleration.z))
        self.arrivals[index].append(time.monotonic())

    def receive_calibrated(self, index, message):
        stamp = stamp_seconds(message.header)
        self.append(index, 'accel', stamp, (message.linear_acceleration.x,
                    message.linear_acceleration.y, message.linear_acceleration.z))
        self.append(index, 'gyro', stamp, tuple(math.degrees(value) for value in
                    (message.angular_velocity.x, message.angular_velocity.y,
                     message.angular_velocity.z)))

    def receive_mag(self, index, message):
        self.append(index, 'mag', stamp_seconds(message.header), tuple(value*1e6 for value in
                    (message.magnetic_field.x, message.magnetic_field.y,
                     message.magnetic_field.z)))

    def receive_euler(self, index, message):
        self.append(index, 'euler', stamp_seconds(message.header), tuple(math.degrees(value) for value in
                    (message.vector.x, message.vector.y, message.vector.z)))

    def receive_status(self, index, message):
        self.calibration[index] = message.data

    def receive_joints(self, message):
        self.joints = dict(zip(message.name, message.position))

    def frequency(self, index):
        samples = self.arrivals[index]
        if len(samples) < 2 or samples[-1] == samples[0]:
            return 0.0
        return (len(samples)-1)/(samples[-1]-samples[0])


class SensorCard(QtWidgets.QGroupBox):
    def __init__(self, index, parent=None):
        super().__init__(f'IMU {index}', parent)
        self.index = index
        layout = QtWidgets.QVBoxLayout(self)
        self.status = QtWidgets.QLabel('Esperando tópicos ROS2…')
        layout.addWidget(self.status)
        self.lines = {}
        specs = (
            ('accel', 'Acelerómetro calibrado', 'm/s²', ('X', 'Y', 'Z')),
            ('gyro', 'Giroscopio calibrado', '°/s', ('X', 'Y', 'Z')),
            ('mag', 'Magnetómetro', 'µT', ('X', 'Y', 'Z')),
            ('euler', 'Ángulos de Euler', '°', ('Roll', 'Pitch', 'Yaw')),
        )
        for key, title, unit, names in specs:
            plot = pg.PlotWidget(title=title)
            plot.setMinimumHeight(165)
            plot.setLabel('left', unit)
            plot.setLabel('bottom', 'Tiempo', units='s')
            plot.showGrid(x=True, y=True, alpha=.25)
            plot.addLegend()
            self.lines[key] = [plot.plot(pen=pg.mkPen(color, width=1.4), name=name)
                               for color, name in zip(('b', 'g', 'r'), names)]
            layout.addWidget(plot)

    def refresh(self, node):
        self.status.setText(f'{node.frequency(self.index):.2f} Hz · '
                            f'Calibración: {node.calibration[self.index]}')
        for key, lines in self.lines.items():
            rows = node.series[self.index][key]
            if not rows:
                continue
            start = rows[0][0]
            x = [row[0]-start for row in rows]
            for axis, line in enumerate(lines):
                line.setData(x, [row[1][axis] for row in rows])


class RosWindow(QtWidgets.QTabWidget):
    def __init__(self, node):
        super().__init__()
        self.node = node
        self.setWindowTitle('MPU9250 · STM32 · ROS2 Humble · Brazo 2R')
        self.resize(1500, 900)
        self.build_overview()
        self.build_sensors()
        self.build_robot()
        self.build_nodes()
        self.spin_timer = QtCore.QTimer(self)
        self.spin_timer.timeout.connect(lambda: rclpy.spin_once(self.node, timeout_sec=0.0))
        self.spin_timer.start(5)
        self.draw_timer = QtCore.QTimer(self)
        self.draw_timer.timeout.connect(self.refresh)
        self.draw_timer.start(100)
        self.graph_timer = QtCore.QTimer(self)
        self.graph_timer.timeout.connect(self.refresh_graph)
        self.graph_timer.start(1000)

    @staticmethod
    def with_footer(layout):
        footer = QtWidgets.QLabel('<b>Integrantes</b><br>' + '<br>'.join(MEMBERS))
        footer.setTextInteractionFlags(QtCore.Qt.TextSelectableByMouse)
        layout.addWidget(footer)

    def build_overview(self):
        page = QtWidgets.QWidget()
        layout = QtWidgets.QVBoxLayout(page)
        title = QtWidgets.QLabel('Sistema de teleoperación con dos MPU9250')
        title.setStyleSheet('font-size: 22px; font-weight: bold;')
        layout.addWidget(title)
        layout.addWidget(QtWidgets.QLabel(
            'STM32 → /dev/ttyACM0 → nodos ROS2 → Roll/Pitch/Yaw → Hombro/Codo. '
            'La adquisición de ambas IMU ocurre en la misma trama.'))
        self.summary = QtWidgets.QLabel('Esperando datos ROS2…')
        self.summary.setStyleSheet('font-size: 16px; padding: 12px; background: #eef4fb;')
        self.summary.setWordWrap(True)
        layout.addWidget(self.summary)
        buttons = QtWidgets.QHBoxLayout()
        for text, key in (('Calibrar IMU 1', 'imu1'), ('Calibrar IMU 2', 'imu2'),
                          ('Tomar postura cero', 'zero')):
            button = QtWidgets.QPushButton(text)
            button.clicked.connect(lambda checked=False, k=key: self.call_service(k))
            buttons.addWidget(button)
        graph = QtWidgets.QPushButton('Abrir rqt_graph')
        graph.clicked.connect(lambda: QtCore.QProcess.startDetached('rqt_graph', []))
        buttons.addWidget(graph)
        layout.addLayout(buttons)
        self.action_status = QtWidgets.QLabel('Los botones llaman servicios ROS2 reales.')
        layout.addWidget(self.action_status)
        layout.addStretch()
        self.with_footer(layout)
        self.addTab(page, 'Interfaz ROS2 / STM32')

    def build_sensors(self):
        content = QtWidgets.QWidget()
        layout = QtWidgets.QHBoxLayout(content)
        self.sensor_cards = {index: SensorCard(index) for index in (1, 2)}
        for card in self.sensor_cards.values():
            layout.addWidget(card, 1)
        scroll = QtWidgets.QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setWidget(content)
        self.addTab(scroll, 'Sensores corporales')

    def build_robot(self):
        page = QtWidgets.QWidget()
        layout = QtWidgets.QVBoxLayout(page)
        layout.addWidget(QtWidgets.QLabel(
            '<b>Brazo 2R</b><br>IMU 1 controla Hombro. La orientación relativa IMU 2 − IMU 1 '
            'controla Codo. Las demás articulaciones permanecen fijas para esta entrega.'))
        self.joint_table = QtWidgets.QTableWidget(6, 3)
        self.joint_table.setHorizontalHeaderLabels(['Articulación', 'Radianes', 'Grados'])
        self.joint_names = ('Cadera', 'Hombro', 'Codo', 'Antebrazo', 'Muneca', 'DedoIzq')
        for row, name in enumerate(self.joint_names):
            self.joint_table.setItem(row, 0, QtWidgets.QTableWidgetItem(name))
        self.joint_table.horizontalHeader().setSectionResizeMode(QtWidgets.QHeaderView.Stretch)
        layout.addWidget(self.joint_table)
        self.joint_status = QtWidgets.QLabel('Esperando /joint_states…')
        layout.addWidget(self.joint_status)
        layout.addStretch()
        self.addTab(page, 'Robot 2R')

    def build_nodes(self):
        page = QtWidgets.QWidget()
        layout = QtWidgets.QVBoxLayout(page)
        layout.addWidget(QtWidgets.QLabel(
            '<b>Monitor del grafo ROS2</b><br>Verde significa que el nodo está activo en este dominio ROS.'))
        self.node_table = QtWidgets.QTableWidget(len(EXPECTED_NODES), 4)
        self.node_table.setHorizontalHeaderLabels(['Estado', 'Nodo', 'Función', 'Salida principal'])
        for row, (name, purpose, output) in enumerate(EXPECTED_NODES):
            for column, value in enumerate(('—', '/' + name, purpose, output)):
                self.node_table.setItem(row, column, QtWidgets.QTableWidgetItem(value))
        self.node_table.horizontalHeader().setSectionResizeMode(QtWidgets.QHeaderView.Stretch)
        layout.addWidget(self.node_table)
        self.topic_label = QtWidgets.QPlainTextEdit()
        self.topic_label.setReadOnly(True)
        self.topic_label.setMaximumHeight(220)
        layout.addWidget(QtWidgets.QLabel('Tópicos activos:'))
        layout.addWidget(self.topic_label)
        self.addTab(page, 'Nodos ROS2')

    def call_service(self, key):
        client = self.node.calibration_clients[key]
        if not client.service_is_ready():
            self.action_status.setText(f'El servicio {client.srv_name} no está disponible.')
            return
        self.action_status.setText(f'Llamando {client.srv_name}…')
        future = client.call_async(Trigger.Request())
        future.add_done_callback(lambda result, k=key: self.service_done(k, result))

    def service_done(self, key, future):
        try:
            response = future.result()
            self.action_status.setText(f'{key}: {response.message}')
        except Exception as error:
            self.action_status.setText(f'Error en {key}: {error}')

    def refresh_graph(self):
        active = {name.lstrip('/') for name in self.node.get_node_names()}
        for row, (name, purpose, output) in enumerate(EXPECTED_NODES):
            item = self.node_table.item(row, 0)
            running = name in active
            item.setText('ACTIVO' if running else 'INACTIVO')
            item.setBackground(pg.mkColor('#b7efc5' if running else '#ffc9c9'))
        topics = sorted(name for name, types in self.node.get_topic_names_and_types())
        self.topic_label.setPlainText('\n'.join(topics))

    def refresh(self):
        for card in self.sensor_cards.values():
            card.refresh(self.node)
        rates = ', '.join(f'IMU {i}: {self.node.frequency(i):.2f} Hz' for i in (1, 2))
        self.summary.setText(f'ROS2 Humble · /dev/ttyACM0 · {rates}<br>'
            f'IMU 1: {self.node.calibration[1]}<br>IMU 2: {self.node.calibration[2]}')
        for row, name in enumerate(self.joint_names):
            value = self.node.joints.get(name)
            self.joint_table.setItem(row, 1, QtWidgets.QTableWidgetItem(
                '—' if value is None else f'{value:.4f}'))
            self.joint_table.setItem(row, 2, QtWidgets.QTableWidgetItem(
                '—' if value is None else f'{math.degrees(value):.2f}°'))
        if self.node.joints:
            self.joint_status.setText('Recibiendo /joint_states. Home esperado: Hombro 89°, Codo −89°.')


def main(args=None):
    rclpy.init(args=args)
    node = RosGuiNode()
    app = QtWidgets.QApplication([sys.argv[0]])
    pg.setConfigOption('background', 'w')
    pg.setConfigOption('foreground', 'k')
    window = RosWindow(node)
    window.showMaximized()
    try:
        app.exec_()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
