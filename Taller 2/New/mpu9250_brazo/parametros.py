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

STM32F767_MUESTRAS_ESPERADAS = 300

TOTAL_IMUS = 2
ENCABEZADO_TAMANO = 2
DATOS_POR_IMU = 9
DATOS_ESPERADOS_IMUS = TOTAL_IMUS * DATOS_POR_IMU + ENCABEZADO_TAMANO

GRAVITY = 9.80665
