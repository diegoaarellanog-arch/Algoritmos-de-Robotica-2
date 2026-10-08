"""Cálculos independientes de ROS para poder probar el protocolo y la orientación."""
import math
from .parametros import GRAVITY, TOTAL_IMUS, DATOS_ESPERADOS_IMUS, ENCABEZADO_TAMANO, DATOS_POR_IMU


# Limpia una línea de texto recibido por puerto serie, la convierte a números flotantes y valida que el tamaño de trama y valores sean finitos.
def parse_dual_line(text):
    values = tuple(float(value) for value in text.replace(',', ' ').split())
    if len(values) != DATOS_ESPERADOS_IMUS or not all(math.isfinite(value) for value in values):
        raise ValueError(f'La trama debe contener {DATOS_ESPERADOS_IMUS} números finitos.')
    return values


# Separa el arreglo de datos en subconjuntos por cada IMU, adjuntando el encabezado común a los 9 datos correspondientes de cada sensor.
def split_imus(values):
    if len(values) != DATOS_ESPERADOS_IMUS:
        raise ValueError(f'Se requiere una trama de {DATOS_ESPERADOS_IMUS} datos para {TOTAL_IMUS} IMU(s).')
    
    header = values[:ENCABEZADO_TAMANO]
    payload = values[ENCABEZADO_TAMANO:]
    
    imus = []
    for i in range(TOTAL_IMUS):
        inicio = i * DATOS_POR_IMU
        fin = inicio + DATOS_POR_IMU
        # Concatena el encabezado común + los 9 datos de la IMU correspondiente
        imus.append(header + payload[inicio:fin])
        
    return imus


# Convierte los valores crudos de los sensores a unidades del Sistema Internacional (m/s² para acelerómetro, rad/s para giroscopio y Teslas para magnetómetro).
def raw_to_si(values, accel_range_g=2.0, gyro_range_dps=250.0):
    accel = tuple(value * accel_range_g / 32768.0 * GRAVITY for value in values[2:5])
    gyro = tuple(math.radians(value * gyro_range_dps / 32768.0) for value in values[5:8])
    magnetic = tuple(value * 1e-6 for value in values[8:11])  # firmware entrega uT
    return accel, gyro, magnetic


# Acota un ángulo en radianes al rango continuo de [-π, π].
def wrap_angle(angle):
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


# Calcula la orientación de azimut (yaw) proyectando el campo magnético al plano horizontal usando los ángulos de inclinación (roll y pitch).
def tilt_compensated_yaw(magnetic, roll, pitch):
    mx, my, mz = magnetic
    bfy = mz * math.sin(roll) - my * math.cos(roll)
    bfx = (mx * math.cos(pitch) + my * math.sin(pitch) * math.sin(roll)
           + mz * math.sin(pitch) * math.cos(roll))
    if math.hypot(bfx, bfy) < 1e-15:
        return None
    return math.atan2(bfy, bfx)


# Genera un vector tridimensional con la magnitud dada orientada hacia la dirección espacial indicada (+X, -X, +Y, -Y, +Z, -Z).
def axis_vector(name, magnitude=GRAVITY):
    if name not in ('+X', '-X', '+Y', '-Y', '+Z', '-Z'):
        raise ValueError('El eje debe ser +X, -X, +Y, -Y, +Z o -Z.')
    vector = [0.0, 0.0, 0.0]
    vector['XYZ'.index(name[1])] = magnitude if name[0] == '+' else -magnitude
    return tuple(vector)