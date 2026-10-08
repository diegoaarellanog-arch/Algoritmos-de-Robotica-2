"""Vista simultánea de las dos IMU presentes en la trama serial de 20 campos."""
from dataclasses import replace

from PyQt5 import QtWidgets
import pyqtgraph as pg

from procesamiento_imu import Configuracion, ProcesadorIMU


class VistaIMU(QtWidgets.QGroupBox):
    def __init__(self, index, parent=None):
        super().__init__(f'IMU {index}', parent)
        self.index = index
        layout = QtWidgets.QVBoxLayout(self)
        self.estado = QtWidgets.QLabel('Sin captura de dos IMU.')
        self.estado.setWordWrap(True)
        layout.addWidget(self.estado)
        self.plots = {}
        specs = (
            ('acel_g', 'Acelerómetro calibrado', 'g', ('X', 'Y', 'Z')),
            ('giro_dps', 'Giroscopio calibrado', '°/s', ('X', 'Y', 'Z')),
            ('euler', 'Ángulos de Euler', '°', ('Roll', 'Pitch', 'Yaw')),
        )
        for key, title, unit, names in specs:
            plot = pg.PlotWidget(title=title)
            plot.setMinimumHeight(185)
            plot.setLabel('left', unit)
            plot.setLabel('bottom', 'Tiempo', units='s')
            plot.showGrid(x=True, y=True, alpha=.25)
            plot.addLegend()
            lines = [plot.plot(pen=pg.mkPen(color, width=1.4), name=name)
                     for color, name in zip(('b', 'g', 'r'), names)]
            self.plots[key] = lines
            layout.addWidget(plot)

    def limpiar(self, message):
        for lines in self.plots.values():
            for line in lines:
                line.setData([], [])
        self.estado.setText(message)

    def dibujar(self, samples, calibrated):
        if not samples:
            self.limpiar('No hay muestras válidas para este sensor.')
            return
        times = [sample.tiempo for sample in samples]
        for key in ('acel_g', 'giro_dps'):
            for axis, line in enumerate(self.plots[key]):
                line.setData(times, [getattr(sample, key)[axis] for sample in samples])
        angles = ([sample.roll[2] for sample in samples],
                  [sample.pitch[2] for sample in samples],
                  [float('nan') if sample.yaw is None else sample.yaw for sample in samples])
        for values, line in zip(angles, self.plots['euler']):
            line.setData(times, values)
        state = 'calibrada' if calibrated else 'sin calibrar'
        self.estado.setText(f'{len(samples)} muestras mostradas · {state}. '
                            'Azul/verde/rojo representan los tres ejes o Roll/Pitch/Yaw.')


class PanelSensoresCorporales(QtWidgets.QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.ultima_sesion = None
        self.perfiles = {}
        outer = QtWidgets.QVBoxLayout(self)
        explanation = QtWidgets.QLabel(
            '<b>Captura simultánea de las dos MPU9250</b><br>'
            'La STM32 entrega ambas IMU en cada trama de 20 campos. Esta pestaña muestra al mismo '
            'tiempo sus valores físicos y sus ángulos. La adquisición y la calibración se inician '
            'en la primera pestaña; aquí no se abre nuevamente el puerto serial.'
        )
        explanation.setWordWrap(True)
        outer.addWidget(explanation)

        self.tabla = QtWidgets.QTableWidget(2, 5)
        self.tabla.setHorizontalHeaderLabels(
            ['ID en firmware', 'Ubicación', 'Segmento medido', 'Lado', 'Link URDF'])
        defaults = (
            ('IMU 1', 'Hombro', 'Brazo superior', 'Por definir', 'Hombro'),
            ('IMU 2', 'Codo', 'Antebrazo', 'Por definir', 'Codo'),
        )
        for row, values in enumerate(defaults):
            for column, value in enumerate(values):
                self.tabla.setItem(row, column, QtWidgets.QTableWidgetItem(value))
        self.tabla.setSelectionBehavior(QtWidgets.QAbstractItemView.SelectRows)
        self.tabla.horizontalHeader().setSectionResizeMode(QtWidgets.QHeaderView.Stretch)
        self.tabla.setMaximumHeight(145)
        outer.addWidget(self.tabla)

        actions = QtWidgets.QHBoxLayout()
        add = QtWidgets.QPushButton('Añadir sensor corporal')
        remove = QtWidgets.QPushButton('Eliminar sensor seleccionado')
        add.clicked.connect(self.agregar_sensor)
        remove.clicked.connect(self.eliminar_sensor)
        actions.addWidget(add)
        actions.addWidget(remove)
        actions.addStretch()
        outer.addLayout(actions)

        self.mensaje = QtWidgets.QLabel(
            'Aún no hay una captura. Para esta entrega, IMU 1 e IMU 2 corresponden a hombro y codo. '
            'Las filas adicionales sirven para preparar sensores de una entrega posterior; el firmware '
            'actual solamente transmite dos IMU.')
        self.mensaje.setWordWrap(True)
        outer.addWidget(self.mensaje)

        cards = QtWidgets.QHBoxLayout()
        self.vistas = {index: VistaIMU(index) for index in (1, 2)}
        for view in self.vistas.values():
            cards.addWidget(view, 1)
        outer.addLayout(cards, 1)

    def agregar_sensor(self):
        row = self.tabla.rowCount()
        self.tabla.insertRow(row)
        values = (f'IMU {row+1}', 'Por definir', 'Por definir', 'Por definir', '')
        for column, value in enumerate(values):
            self.tabla.setItem(row, column, QtWidgets.QTableWidgetItem(value))
        self.tabla.selectRow(row)
        self.mensaje.setText('Sensor añadido a la planificación. El firmware actual solo entrega IMU 1 e IMU 2.')

    def eliminar_sensor(self):
        rows = sorted({index.row() for index in self.tabla.selectionModel().selectedRows()}, reverse=True)
        if not rows and self.tabla.currentRow() >= 0:
            rows = [self.tabla.currentRow()]
        if not rows:
            self.mensaje.setText('Selecciona primero una fila completa para eliminarla.')
            return
        for row in rows:
            self.tabla.removeRow(row)
        self.mensaje.setText(f'Se eliminaron {len(rows)} sensor(es) de la tabla de asignación.')

    def actualizar(self, session, profiles=None):
        self.ultima_sesion = session
        self.perfiles = dict(profiles or {})
        if not session.raw or not all(len(row) == 20 for row in session.raw):
            for index, view in self.vistas.items():
                if index == session.config.imu_index:
                    view.dibujar(session.samples[-300:], session.calibracion is not None)
                else:
                    view.limpiar('La captura recibida contiene una sola IMU; no hay datos para este panel.')
            self.mensaje.setText('La vista simultánea requiere las tramas de 20 campos del firmware de dos IMU.')
            return
        rows = session.raw[-300:]
        for index, view in self.vistas.items():
            profile = self.perfiles.get(index)
            if profile is not None:
                config = replace(profile.config, dt=session.config.dt, alpha=session.config.alpha,
                                 rango_acel=session.config.rango_acel,
                                 rango_giro=session.config.rango_giro, imu_index=index)
                calibrated = True
            elif session.config.imu_index == index:
                config = session.config
                calibrated = session.calibracion is not None
            else:
                c = session.config
                config = Configuracion(c.dt, c.alpha, c.rango_acel, c.rango_giro,
                                       (0., 0., 0.), (0., 0., 0.), index)
                calibrated = False
            processor = ProcesadorIMU(config)
            samples = tuple(processor.procesar(row) for row in rows)
            view.dibujar(samples, calibrated)
        self.mensaje.setText(
            f'Captura común: {len(session.raw)} tramas. Se muestran las últimas {len(rows)} de ambas IMU. '
            'Cada panel indica si está usando su propio perfil de calibración.')