# -*- coding: utf-8 -*-
"""Interfaz, visor URDF y etapas de procesamiento IMU con dos hilos de aplicación."""
import sys
from pathlib import Path
from PyQt5 import QtCore, QtWidgets
import pyqtgraph as pg
from pyqtgraph import PlotWidget
from visor_urdf import VisorURDF
from nodos_interfaz import PanelNodos
from adquisicion_interfaz import PanelAdquisicion
from sensores_corporales import PanelSensoresCorporales

pg.setConfigOption('background', 'w')
pg.setConfigOption('foreground', 'k')


class Ui_Form:
    def setupUi(self, Form):
        Form.setObjectName("Form")
        Form.resize(1500, 712)
        self.label = QtWidgets.QLabel(Form)
        self.label.setGeometry(QtCore.QRect(1070, 570, 420, 121))
        self.label.setAlignment(QtCore.Qt.AlignBottom|QtCore.Qt.AlignLeading|QtCore.Qt.AlignLeft)
        self.label.setObjectName("label")
        self.groupBox = QtWidgets.QGroupBox(Form)
        self.groupBox.setGeometry(QtCore.QRect(10, 40, 341, 661))
        self.groupBox.setObjectName("groupBox")
        self.GiroscopioNoCalibrado = PlotWidget(self.groupBox)
        self.GiroscopioNoCalibrado.setGeometry(QtCore.QRect(10, 40, 321, 191))
        self.GiroscopioNoCalibrado.setObjectName("GiroscopioNoCalibrado")
        self.label_4 = QtWidgets.QLabel(self.groupBox)
        self.label_4.setGeometry(QtCore.QRect(10, 20, 71, 16))
        self.label_4.setObjectName("label_4")
        self.label_5 = QtWidgets.QLabel(self.groupBox)
        self.label_5.setGeometry(QtCore.QRect(10, 230, 71, 16))
        self.label_5.setObjectName("label_5")
        self.label_6 = QtWidgets.QLabel(self.groupBox)
        self.label_6.setGeometry(QtCore.QRect(10, 440, 71, 16))
        self.label_6.setObjectName("label_6")
        self.AcelerometroNoCalibrado = PlotWidget(self.groupBox)
        self.AcelerometroNoCalibrado.setGeometry(QtCore.QRect(10, 250, 321, 191))
        self.AcelerometroNoCalibrado.setObjectName("AcelerometroNoCalibrado")
        self.MagnetometroNoCalibrado = PlotWidget(self.groupBox)
        self.MagnetometroNoCalibrado.setGeometry(QtCore.QRect(10, 460, 321, 191))
        self.MagnetometroNoCalibrado.setObjectName("MagnetometroNoCalibrado")
        self.label_2 = QtWidgets.QLabel(Form)
        self.label_2.setGeometry(QtCore.QRect(20, 0, 301, 41))
        self.label_2.setAlignment(QtCore.Qt.AlignLeading|QtCore.Qt.AlignLeft|QtCore.Qt.AlignVCenter)
        self.label_2.setObjectName("label_2")
        self.label_3 = QtWidgets.QLabel(Form)
        self.label_3.setGeometry(QtCore.QRect(10, 0, 1500, 41))
        self.label_3.setObjectName("label_3")
        self.groupBox_2 = QtWidgets.QGroupBox(Form)
        self.groupBox_2.setGeometry(QtCore.QRect(360, 40, 341, 661))
        self.groupBox_2.setObjectName("groupBox_2")
        self.GiroscopioNoCalibrado_2 = PlotWidget(self.groupBox_2)
        self.GiroscopioNoCalibrado_2.setGeometry(QtCore.QRect(10, 40, 321, 191))
        self.GiroscopioNoCalibrado_2.setObjectName("GiroscopioNoCalibrado_2")
        self.label_7 = QtWidgets.QLabel(self.groupBox_2)
        self.label_7.setGeometry(QtCore.QRect(10, 20, 71, 16))
        self.label_7.setObjectName("label_7")
        self.label_8 = QtWidgets.QLabel(self.groupBox_2)
        self.label_8.setGeometry(QtCore.QRect(10, 230, 71, 16))
        self.label_8.setObjectName("label_8")
        self.label_9 = QtWidgets.QLabel(self.groupBox_2)
        self.label_9.setGeometry(QtCore.QRect(10, 440, 71, 16))
        self.label_9.setObjectName("label_9")
        self.AcelerometroNoCalibrado_2 = PlotWidget(self.groupBox_2)
        self.AcelerometroNoCalibrado_2.setGeometry(QtCore.QRect(10, 250, 321, 191))
        self.AcelerometroNoCalibrado_2.setObjectName("AcelerometroNoCalibrado_2")
        self.MagnetometroNoCalibrado_2 = PlotWidget(self.groupBox_2)
        self.MagnetometroNoCalibrado_2.setGeometry(QtCore.QRect(10, 460, 321, 191))
        self.MagnetometroNoCalibrado_2.setObjectName("MagnetometroNoCalibrado_2")
        self.groupBox_3 = QtWidgets.QGroupBox(Form)
        self.groupBox_3.setGeometry(QtCore.QRect(710, 40, 351, 661))
        self.groupBox_3.setObjectName("groupBox_3")
        self.GiroscopioNoCalibrado_4 = PlotWidget(self.groupBox_3)
        self.GiroscopioNoCalibrado_4.setGeometry(QtCore.QRect(10, 40, 321, 191))
        self.GiroscopioNoCalibrado_4.setObjectName("GiroscopioNoCalibrado_4")
        self.AcelerometroNoCalibrado_4 = PlotWidget(self.groupBox_3)
        self.AcelerometroNoCalibrado_4.setGeometry(QtCore.QRect(10, 250, 321, 191))
        self.AcelerometroNoCalibrado_4.setObjectName("AcelerometroNoCalibrado_4")
        self.MagnetometroNoCalibrado_4 = PlotWidget(self.groupBox_3)
        self.MagnetometroNoCalibrado_4.setGeometry(QtCore.QRect(10, 460, 321, 191))
        self.MagnetometroNoCalibrado_4.setObjectName("MagnetometroNoCalibrado_4")
        self.label_19 = QtWidgets.QLabel(self.groupBox_3)
        self.label_19.setGeometry(QtCore.QRect(10, 20, 71, 16))
        self.label_19.setObjectName("label_19")
        self.label_20 = QtWidgets.QLabel(self.groupBox_3)
        self.label_20.setGeometry(QtCore.QRect(10, 230, 71, 16))
        self.label_20.setObjectName("label_20")
        self.label_21 = QtWidgets.QLabel(self.groupBox_3)
        self.label_21.setGeometry(QtCore.QRect(10, 440, 71, 16))
        self.label_21.setObjectName("label_21")
        self.groupBox_4 = QtWidgets.QGroupBox(Form)
        self.groupBox_4.setGeometry(QtCore.QRect(1070, 40, 420, 111))
        self.groupBox_4.setObjectName("groupBox_4")
        self.verticalLayoutWidget = QtWidgets.QWidget(self.groupBox_4)
        self.verticalLayoutWidget.setGeometry(QtCore.QRect(10, 20, 400, 81))
        self.verticalLayoutWidget.setObjectName("verticalLayoutWidget")
        self.verticalLayout = QtWidgets.QVBoxLayout(self.verticalLayoutWidget)
        self.verticalLayout.setContentsMargins(0, 0, 0, 0)
        self.verticalLayout.setObjectName("verticalLayout")
        self.comboBox = QtWidgets.QComboBox(self.verticalLayoutWidget)
        self.comboBox.setCurrentText("")
        self.comboBox.setObjectName("comboBox")
        self.verticalLayout.addWidget(self.comboBox)
        self.horizontalLayout_2 = QtWidgets.QHBoxLayout()
        self.horizontalLayout_2.setObjectName("horizontalLayout_2")
        self.label_13 = QtWidgets.QLabel(self.verticalLayoutWidget)
        self.label_13.setObjectName("label_13")
        self.horizontalLayout_2.addWidget(self.label_13)
        self.comboBox_2 = QtWidgets.QComboBox(self.verticalLayoutWidget)
        self.comboBox_2.setObjectName("comboBox_2")
        self.horizontalLayout_2.addWidget(self.comboBox_2)
        self.label_14 = QtWidgets.QLabel(self.verticalLayoutWidget)
        self.label_14.setObjectName("label_14")
        self.horizontalLayout_2.addWidget(self.label_14)
        self.doubleSpinBox = QtWidgets.QDoubleSpinBox(self.verticalLayoutWidget)
        self.doubleSpinBox.setObjectName("doubleSpinBox")
        self.horizontalLayout_2.addWidget(self.doubleSpinBox)
        self.verticalLayout.addLayout(self.horizontalLayout_2)
        self.horizontalLayout = QtWidgets.QHBoxLayout()
        self.horizontalLayout.setObjectName("horizontalLayout")
        self.pushButton_3 = QtWidgets.QPushButton(self.verticalLayoutWidget)
        self.pushButton_3.setObjectName("pushButton_3")
        self.horizontalLayout.addWidget(self.pushButton_3)
        self.pushButton_2 = QtWidgets.QPushButton(self.verticalLayoutWidget)
        self.pushButton_2.setObjectName("pushButton_2")
        self.horizontalLayout.addWidget(self.pushButton_2)
        self.verticalLayout.addLayout(self.horizontalLayout)
        self.groupBox_5 = QtWidgets.QGroupBox(Form)
        self.groupBox_5.setGeometry(QtCore.QRect(1070, 150, 420, 415))
        self.groupBox_5.setObjectName("groupBox_5")
        self.plainTextEdit = QtWidgets.QPlainTextEdit(self.groupBox_5)
        self.plainTextEdit.setGeometry(QtCore.QRect(10, 110, 400, 295))
        self.plainTextEdit.setObjectName("plainTextEdit")
        self.verticalLayoutWidget_2 = QtWidgets.QWidget(self.groupBox_5)
        self.verticalLayoutWidget_2.setGeometry(QtCore.QRect(10, 20, 400, 86))
        self.verticalLayoutWidget_2.setObjectName("verticalLayoutWidget_2")
        self.verticalLayout_3 = QtWidgets.QVBoxLayout(self.verticalLayoutWidget_2)
        self.verticalLayout_3.setContentsMargins(0, 0, 0, 0)
        self.verticalLayout_3.setObjectName("verticalLayout_3")
        self.horizontalLayout_3 = QtWidgets.QHBoxLayout()
        self.horizontalLayout_3.setObjectName("horizontalLayout_3")
        self.label_18 = QtWidgets.QLabel(self.verticalLayoutWidget_2)
        self.label_18.setObjectName("label_18")
        self.horizontalLayout_3.addWidget(self.label_18)
        self.pushButton_4 = QtWidgets.QPushButton(self.verticalLayoutWidget_2)
        self.pushButton_4.setObjectName("pushButton_4")
        self.horizontalLayout_3.addWidget(self.pushButton_4)
        self.pushButton = QtWidgets.QPushButton(self.verticalLayoutWidget_2)
        self.pushButton.setObjectName("pushButton")
        self.horizontalLayout_3.addWidget(self.pushButton)
        self.pushButton_8 = QtWidgets.QPushButton(self.verticalLayoutWidget_2)
        self.pushButton_8.setObjectName("pushButton_8")
        self.horizontalLayout_3.addWidget(self.pushButton_8)
        self.verticalLayout_3.addLayout(self.horizontalLayout_3)
        self.horizontalLayout_4 = QtWidgets.QHBoxLayout()
        self.horizontalLayout_4.setObjectName("horizontalLayout_4")
        self.label_17 = QtWidgets.QLabel(self.verticalLayoutWidget_2)
        self.label_17.setObjectName("label_17")
        self.horizontalLayout_4.addWidget(self.label_17)
        self.pushButton_6 = QtWidgets.QPushButton(self.verticalLayoutWidget_2)
        self.pushButton_6.setObjectName("pushButton_6")
        self.horizontalLayout_4.addWidget(self.pushButton_6)
        self.pushButton_7 = QtWidgets.QPushButton(self.verticalLayoutWidget_2)
        self.pushButton_7.setObjectName("pushButton_7")
        self.horizontalLayout_4.addWidget(self.pushButton_7)
        self.pushButton_5 = QtWidgets.QPushButton(self.verticalLayoutWidget_2)
        self.pushButton_5.setObjectName("pushButton_5")
        self.horizontalLayout_4.addWidget(self.pushButton_5)
        self.pushButton_9 = QtWidgets.QPushButton(self.verticalLayoutWidget_2)
        self.pushButton_9.setObjectName("pushButton_9")
        self.horizontalLayout_4.addWidget(self.pushButton_9)
        self.verticalLayout_3.addLayout(self.horizontalLayout_4)
        self.horizontalLayout_5 = QtWidgets.QHBoxLayout()
        self.horizontalLayout_5.setObjectName("horizontalLayout_5")
        self.pushButton_10 = QtWidgets.QPushButton(self.verticalLayoutWidget_2)
        self.pushButton_10.setObjectName("pushButton_10")
        self.horizontalLayout_5.addWidget(self.pushButton_10)
        self.label_15 = QtWidgets.QLabel(self.verticalLayoutWidget_2)
        self.label_15.setObjectName("label_15")
        self.horizontalLayout_5.addWidget(self.label_15)
        self.comboBox_3 = QtWidgets.QComboBox(self.verticalLayoutWidget_2)
        self.comboBox_3.setObjectName("comboBox_3")
        self.horizontalLayout_5.addWidget(self.comboBox_3)
        self.label_16 = QtWidgets.QLabel(self.verticalLayoutWidget_2)
        self.label_16.setObjectName("label_16")
        self.horizontalLayout_5.addWidget(self.label_16)
        self.doubleSpinBox_2 = QtWidgets.QDoubleSpinBox(self.verticalLayoutWidget_2)
        self.doubleSpinBox_2.setObjectName("doubleSpinBox_2")
        self.horizontalLayout_5.addWidget(self.doubleSpinBox_2)
        self.verticalLayout_3.addLayout(self.horizontalLayout_5)
        self.label.raise_()
        self.groupBox.raise_()
        self.label_3.raise_()
        self.groupBox_2.raise_()
        self.groupBox_3.raise_()
        self.groupBox_4.raise_()
        self.groupBox_5.raise_()
        self.label_2.raise_()

        self.retranslateUi(Form)
        Form.setWindowTitle("MPU · STM · URDF — Base de interfaz")
        # El recurso IMG_rc y el logo no fueron incluidos en el archivo recibido.
        self.label_2.setText("UNIVERSIDAD ECCI")
        self.label_3.setText('<p align="center" style="font-size:16pt">MPU · STM · URDF</p>')
        self.plainTextEdit.setReadOnly(True)
        self.plainTextEdit.setPlainText(
            "Base de interfaz.\n"
            "Modelo de referencia del código: MPU9250.\n"
            "STM32 confirmada; referencia exacta y protocolo pendientes.\n"
            "La adquisición, calibración y estimación de orientación aún no están conectadas."
        )
        self.comboBox.addItem("Puerto STM32 pendiente de configurar")
        self.comboBox_2.addItems(["9600", "115200"])
        self.comboBox_3.addItems(["50", "100", "200", "300"])
        self.doubleSpinBox.setValue(0.5)
        self.doubleSpinBox_2.setValue(20.0)
        for button in Form.findChildren(QtWidgets.QPushButton):
            button.setEnabled(False)
            button.setToolTip("Control visual reservado para la futura integración con la STM32.")
        for widget in (self.comboBox, self.comboBox_2, self.comboBox_3,
                       self.doubleSpinBox, self.doubleSpinBox_2):
            widget.setEnabled(False)
        for plot in Form.findChildren(PlotWidget):
            plot.showGrid(x=True, y=True, alpha=0.2)
        QtCore.QMetaObject.connectSlotsByName(Form)

    def retranslateUi(self, Form):
        _translate = QtCore.QCoreApplication.translate
        Form.setWindowTitle(_translate("Form", "Form"))
        self.label.setText(_translate("Form", "<!DOCTYPE HTML PUBLIC \"-//W3C//DTD HTML 4.0//EN\" \"http://www.w3.org/TR/REC-html40/strict.dtd\">\n"
"<html><head><meta name=\"qrichtext\" content=\"1\" /><style type=\"text/css\">\n"
"p, li { white-space: pre-wrap; }\n"
"</style></head><body style=\" font-family:\'MS Shell Dlg 2\'; font-size:8pt; font-weight:400; font-style:normal;\">\n"
"<p style=\" margin-top:12px; margin-bottom:12px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\">Integrantes: <br />Diego Alejandro Arellano Gutierrez - 110847<br />Edwar Felipe García Patiño - 142911<br />Javier Bohorquez Gaitán - 98587<br />Johan Montejo - 124077<br />Sergio Iván Jaimes Garzón - 133238</p></body></html>"))
        self.groupBox.setTitle(_translate("Form", "Datos No Calibrados "))
        self.label_4.setText(_translate("Form", "Giroscopio"))
        self.label_5.setText(_translate("Form", "Acelerometro"))
        self.label_6.setText(_translate("Form", "Magnetometro"))
        self.label_2.setText(_translate("Form", "<html><head/><body><p><img src=\":/newPrefix/UNIVERSIDAD_ECCI.png\" width=\"150\"/></p></body></html>"))
        self.label_3.setText(_translate("Form", "<html><head/><body><p align=\"center\"><span style=\" font-size:16pt;\">MPU9250 - ADQUISICION, CALIBRACION Y GRAFICACION DE DATOS</span></p></body></html>"))
        self.groupBox_2.setTitle(_translate("Form", "Datos Calibrados "))
        self.label_7.setText(_translate("Form", "Giroscopio"))
        self.label_8.setText(_translate("Form", "Acelerometro"))
        self.label_9.setText(_translate("Form", "Magnetometro"))
        self.groupBox_3.setTitle(_translate("Form", "Ángulos de Euler"))
        self.groupBox_2.setTitle(_translate("Form", "Datos Calibrados "))
        self.label_19.setText(_translate("Form", "Roll"))
        self.label_20.setText(_translate("Form", "Pitch"))
        self.label_21.setText(_translate("Form", "Yaw"))
        self.groupBox_4.setTitle(_translate("Form", "Comunicación Serial"))
        self.label_13.setText(_translate("Form", "Velocidad"))
        self.label_14.setText(_translate("Form", "Timeout"))
        self.pushButton_3.setText(_translate("Form", "Desconectar"))
        self.pushButton_2.setText(_translate("Form", "Conectar"))
        self.groupBox_5.setTitle(_translate("Form", "Monitor Serial"))
        self.pushButton_4.setText(_translate("Form", "Giro/Acel"))
        self.pushButton.setText(_translate("Form", "Magn"))
        self.pushButton_8.setText(_translate("Form", "Giro/Acel/Magn"))
        self.pushButton_6.setText(_translate("Form", "Giro/Acel"))
        self.pushButton_7.setText(_translate("Form", "Adquirir YZ (X)"))
        self.pushButton_5.setText(_translate("Form", "Magn Y (Z)"))
        self.pushButton_9.setText(_translate("Form", "Magn Z (Y)"))
        self.pushButton_10.setText(_translate("Form", "Calibraciones"))
        self.label_15.setText(_translate("Form", "N. Muestras"))
        self.label_16.setText(_translate("Form", "T. Max (s)"))
        self.label_17.setText(_translate("Form", "Calibrar: "))
        self.label_18.setText(_translate("Form", "Adquirir: "))



class VentanaBase(QtWidgets.QTabWidget):
    """Separa la interfaz conservada del espacio para el futuro visor URDF."""

    def __init__(self):
        super().__init__()
        self.setWindowTitle("MPU · STM · URDF — Base de interfaz")
        self.resize(1540, 900)
        self.form = QtWidgets.QWidget()
        self.ui = Ui_Form()
        self.ui.setupUi(self.form)
        self.form.setMinimumSize(1500, 712)
        scroll = QtWidgets.QScrollArea()
        scroll.setWidget(self.form)
        scroll.setWidgetResizable(True)
        self.addTab(scroll, "Interfaz MPU9250 / STM32")

        self.pagina_captura = PanelSensoresCorporales()
        self.tabla_sensores = self.pagina_captura.tabla
        self.addTab(self.pagina_captura, "Sensores corporales")
        self.setCurrentWidget(self.pagina_captura)

        self.pagina_urdf = QtWidgets.QWidget()
        layout = QtWidgets.QVBoxLayout(self.pagina_urdf)
        title = QtWidgets.QLabel("Brazo robótico — visor URDF")
        title.setStyleSheet("font-size: 20px; font-weight: bold;")
        layout.addWidget(title)
        row = QtWidgets.QHBoxLayout()
        self.ruta_urdf = QtWidgets.QLineEdit()
        self.ruta_urdf.setReadOnly(True)
        self.ruta_urdf.setPlaceholderText("Opcional: selecciona el URDF cuando esté disponible")
        self.seleccionar_urdf = QtWidgets.QPushButton("Seleccionar URDF…")
        self.seleccionar_urdf.clicked.connect(self.elegir_urdf)
        row.addWidget(self.ruta_urdf)
        row.addWidget(self.seleccionar_urdf)
        layout.addLayout(row)
        self.visor_urdf = VisorURDF()
        self.estado_urdf = self.visor_urdf.status
        layout.addWidget(self.visor_urdf, 1)
        self.addTab(self.pagina_urdf, "URDF")
        base_modelos = Path(__file__).resolve().parent / "modelo_usuario"
        modelo = base_modelos / "urdfv4" / "urdf" / "urdfv4.urdf"
        if not modelo.is_file():
            modelo = base_modelos / "URDF" / "urdf" / "URDF.urdf"
        if modelo.is_file() and self.visor_urdf.load_model(modelo):
            self.ruta_urdf.setText(str(modelo))
            self.setCurrentWidget(self.pagina_urdf)
        self.panel_nodos = PanelNodos(self)
        self.addTab(self.panel_nodos, "Nodos y Matplotlib")
        self.ui.groupBox_4.hide()
        self.ui.groupBox_5.hide()
        self.panel_adquisicion = PanelAdquisicion(self.panel_nodos,
            lambda: self.setCurrentWidget(self.panel_nodos), self.form,
            lambda: self.setCurrentWidget(self.pagina_captura))
        self.panel_adquisicion.setGeometry(QtCore.QRect(1070, 40, 420, 525))
        self.panel_adquisicion.show()
        self.panel_nodos.sesion_actualizada.connect(
            lambda session: self.pagina_captura.actualizar(session, self.panel_adquisicion.perfiles))
        self.setCurrentIndex(0)
        self.cierre_pendiente = False
        self.curvas_imu = {}
        for attr, data_attr, unit in [
            ('GiroscopioNoCalibrado', 'giro_raw', 'RAW'),
            ('AcelerometroNoCalibrado', 'acel_raw', 'RAW'),
            ('GiroscopioNoCalibrado_2', 'giro_dps', '°/s'),
            ('AcelerometroNoCalibrado_2', 'acel_g', 'g'),
            ('GiroscopioNoCalibrado_4', 'roll', '°'),
            ('AcelerometroNoCalibrado_4', 'pitch', '°')]:
            plot = getattr(self.ui, attr)
            plot.addLegend()
            plot.setLabel('left', unit)
            plot.setLabel('bottom', 'Tiempo configurado', units='s')
            names = ('Acel', 'Giro', 'Filtro') if data_attr in ('roll', 'pitch') else ('X', 'Y', 'Z')
            curves = [plot.plot(pen=pg.mkPen(color, width=1.5), name=name)
                      for color, name in zip(('b', 'g', 'r'), names)]
            self.curvas_imu[data_attr] = curves
        self.ui.groupBox_2.setTitle("Datos con offsets y escala")
        self.ui.label_21.setText("Yaw · magnetómetro compensado")
        self.ui.label_9.setText("Campo horizontal Bfx / Bfy (sin calibración magnética)")
        for widget, attr, names in [
            (self.ui.MagnetometroNoCalibrado, 'mag_raw', ('X', 'Y', 'Z')),
            (self.ui.MagnetometroNoCalibrado_2, 'mag_horizontal', ('Bfx', 'Bfy')),
            (self.ui.MagnetometroNoCalibrado_4, 'yaw', ('Yaw',))]:
            widget.addLegend()
            widget.setLabel('left', '°' if attr == 'yaw' else 'Unidades de entrada')
            widget.setLabel('bottom', 'Tiempo configurado', units='s')
            self.curvas_imu[attr] = [widget.plot(pen=pg.mkPen(color, width=1.5), name=name)
                                   for color, name in zip(('b', 'g', 'r'), names)]
        self.panel_nodos.datos_actualizados.connect(self.actualizar_graficas_imu)

    def actualizar_graficas_imu(self, samples):
        times = [sample.tiempo for sample in samples]
        for attr, curves in self.curvas_imu.items():
            for index, curve in enumerate(curves):
                values = []
                for sample in samples:
                    value = getattr(sample, attr)
                    values.append(float('nan') if value is None else value if attr == 'yaw' else value[index])
                curve.setData(times, values)
        self.ui.plainTextEdit.setPlainText(
            "Etapas locales de los ejemplos (sin ROS).\n"
            + self.panel_nodos.status.text() + "\n"
            + f"Última muestra: {samples[-1].indice:g}\n"
            + f"Roll filtrado: {samples[-1].roll[2]:.2f}°\n"
            + f"Pitch filtrado: {samples[-1].pitch[2]:.2f}°\n"
            + (f"Yaw magnético: {samples[-1].yaw:.2f}°\n" if samples[-1].yaw is not None
               else "Yaw sin datos: se requieren mx, my, mz y un campo horizontal válido.\n")
            + "Calibración magnética y vinculación corporal al URDF: pendientes."
        )

    def closeEvent(self, event):
        if self.panel_nodos.running():
            event.ignore()
            if not self.cierre_pendiente:
                self.cierre_pendiente = True
                self.panel_nodos.worker.finished.connect(self.close)
            self.panel_nodos.stop()
            return
        event.accept()

    def agregar_sensor(self):
        self.pagina_captura.agregar_sensor()

    def elegir_urdf(self):
        path, _ = QtWidgets.QFileDialog.getOpenFileName(
            self, "Seleccionar modelo URDF", "", "Modelo URDF (*.urdf)"
        )
        if path and self.visor_urdf.load_model(path):
            self.ruta_urdf.setText(str(Path(path)))


if __name__ == "__main__":
    app = QtWidgets.QApplication(sys.argv)
    window = VentanaBase()
    screen = app.primaryScreen()
    if screen is not None:
        available = screen.availableGeometry()
        window.resize(min(window.width(), available.width()),
                      min(window.height(), available.height()))
        window.move(available.center() - window.rect().center())
    window.show()
    window.raise_()
    window.activateWindow()
    print("Interfaz abierta. La terminal espera mientras la ventana esté abierta.", flush=True)
    sys.exit(app.exec_())