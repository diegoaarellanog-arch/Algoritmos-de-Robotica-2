class PanelAdquisicion(QtWidgets.QGroupBox):
    def __init__(self, nodos, abrir_nodos, parent=None):
    def __init__(self, nodos, abrir_nodos, parent=None, abrir_sensores=None):
        super().__init__('Adquisición → calibración → nodos', parent)
        self.nodos, self.abrir_nodos = nodos, abrir_nodos
        self.abrir_sensores = abrir_sensores or (lambda: None)
        self.sesion = None
        self.envio_pendiente = False
        self.ambas_pendiente = False
        layout = QtWidgets.QVBoxLayout(self)
        self.imu = QtWidgets.QComboBox()
        self.imu.addItems(['IMU 1', 'IMU 2'])
        self.imu.setToolTip('Trama Keil: IMU 1 = 0x68; IMU 2 = 0x69. La captura conserva ambas.')
        self.imu.addItem('IMU 1', 1)
        self.imu.addItem('IMU 2', 2)
        self.imu.addItem('Ambas IMU', 0)
        self.imu.setToolTip('IMU 1/2 permite calibrar una; Ambas IMU abre la vista simultánea al terminar.')
        form.addWidget(self.imu, 0, 3)
        accel_range, gyro_range = float(self.rango_acel.currentText()), float(self.rango_giro.currentText())
        imu_index = self.imu.currentIndex()+1
        selected = self.imu.currentData()
        imu_index = 1 if selected == 0 else selected
        profile = self.perfiles.get(imu_index)
        self.sesion = None
        self.ambas_pendiente = selected == 0
        self.envio_pendiente = profile is not None and self.auto_nodos.isChecked()
        self.envio_pendiente = selected != 0 and profile is not None and self.auto_nodos.isChecked()
        self.nodos.iniciar_trabajador(HiloAdquisicion(config, source, self.nodos))
        self.estado.setText('Capturando. Las gráficas muestran las muestras; espera a que finalice.')
        self.estado.setText('Capturando ambas IMU; al terminar se abrirá Sensores corporales.'
                            if selected == 0 else
                            'Capturando. Las gráficas muestran las muestras; espera a que finalice.')

            return  
        if self.imu.currentData() == 0:
            self.estado.setText('Para calibrar, selecciona IMU 1 o IMU 2. Cada sensor necesita sus propios offsets.')
            return
        name = 'sesion_'+datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S_%fZ')+'.json'
            return
        index = self.imu.currentIndex()+1
        index = self.imu.currentData()
        if index == 0:
            self.estado.setText('Vista de ambas IMU seleccionada. Se usa la captura común de 20 campos.')
            self.abrir_sensores()
            return
        c = self.sesion.config
        self.sesion = session
        if not self.ambas_pendiente:
        self.imu.blockSignals(True)
        self.imu.setCurrentIndex(session.config.imu_index-1)
        self.imu.blockSignals(False)
            self.imu.blockSignals(True)
            self.imu.setCurrentIndex(self.imu.findData(session.config.imu_index))
            self.imu.blockSignals(False)
        if session.calibracion is None:
        ready = not busy and self.sesion is not None
        individual = self.imu.currentData() in (1, 2)
        self.calibrar.setEnabled(ready)
        self.calibrar.setEnabled(ready and individual)
        self.guardar.setEnabled(ready)
        self.enviar.setEnabled(ready and self.sesion.calibracion is not None)
        self.enviar.setEnabled(ready and individual and self.sesion.calibracion is not None)
        if not busy:
                    self.enviar_datos()
            if self.ambas_pendiente:
                self.ambas_pendiente = False
                successful = isinstance(self.nodos.worker, HiloAdquisicion) and self.sesion is not None
                if successful and all(len(row) == 20 for row in self.sesion.raw):
                    self.estado.setText(f'Captura simultánea lista: {len(self.sesion.raw)} tramas con ambas IMU.')
                    self.abrir_sensores()
                elif successful:
                    self.estado.setText('La fuente terminó, pero no entregó tramas de 20 campos para ambas IMU.')