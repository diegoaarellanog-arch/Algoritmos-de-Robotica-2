#include <Wire.h>

// Endereço I2C de 7 bits (En Arduino Wire se usa 0x68 en lugar de 0xD0 que incluye el bit R/W)
#define MPU6500_ADDRESS       0x68 

// Escalas do girôscopio
#define GYRO_FULL_SCALE_250_DPS   0x00
#define GYRO_FULL_SCALE_500_DPS   0x08
#define GYRO_FULL_SCALE_1000_DPS  0x10
#define GYRO_FULL_SCALE_2000_DPS  0x18

// Escalas do acelerômetro
#define ACC_FULL_SCALE_2_G        0x00
#define ACC_FULL_SCALE_4_G        0x08
#define ACC_FULL_SCALE_8_G        0x08
#define ACC_FULL_SCALE_16_G       0x18

// Escalas de conversao
#define SENSITIVITY_ACCEL     (2.0 / 32768.0)
#define SENSITIVITY_GYRO      (250.0 / 32768.0)
#define SENSITIVITY_TEMP      333.87
#define TEMP_OFFSET           21.0
#define SENSITIVITY_MAGN      ((10.0 * 4800.0) / 32768.0)

// Valores "RAW" de tipo inteiro
int16_t raw_accelx, raw_accely, raw_accelz;
int16_t raw_gyrox, raw_gyroy, raw_gyroz;
int16_t raw_temp;

// Saídas calibradas
float accelx, accely, accelz;
float gyrox, gyroy, gyroz;
float temp;

// Buffer para la lectura en ráfaga de 14 bytes
uint8_t GirAcel[14];

unsigned long startTime;
float timer_sec = 0;

// Función auxiliar para escribir en un registro I2C
void writeRegister(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(MPU6500_ADDRESS);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission();
}

// Función auxiliar para leer 1 byte de un registro
uint8_t readRegister(uint8_t reg) {
  Wire.beginTransmission(MPU6500_ADDRESS);
  Wire.write(reg);
  Wire.endTransmission(false); // Restart
  Wire.requestFrom((uint8_t)MPU6500_ADDRESS, (uint8_t)1);
  if (Wire.available()) {
    return Wire.read();
  }
  return 0x00;
}

void setup() {
  Serial.begin(115200);
  while (!Serial);

  Wire.begin();
  Wire.setClock(400000); // Modo I2C Fast (400 kHz)

  // Desativa modo de hibernação do MPU (PWR_MGMT_1 = 0x6B)
  writeRegister(0x6B, 0x00);

  Serial.println("TESTE DE CONEXAO PARA O GIROSCOPIO E O ACELEROMETRO");
  Serial.println("1. Teste de conexao da MPU6050...");

  // Quem sou eu (WHO_AM_I = 0x75)
  uint8_t who_am_i = readRegister(0x75);

  if (who_am_i != 0x68 && who_am_i != 0x71 && who_am_i != 0x70) {
    Serial.println("Erro de conexao com a MPU6050");
    Serial.print("Opaaa. Eu nao sou a MPU6050, Quem sou eu? :S. I am: 0x");
    Serial.println(who_am_i, HEX);
    while (1); // Bucle infinito en caso de error
  } else {
    Serial.println("Conexao bem sucedida com a MPU6050");
    Serial.println("Oi, tudo joia?... Eu sou a MPU6050 XD\n");
  }

  delay(100);

  // Configura o Girôscopio (Full Scale Gyro Range = 250 deg/s -> GYRO_CONFIG 0x1B)
  writeRegister(0x1B, GYRO_FULL_SCALE_250_DPS);

  // Configura o Acelerômetro (Full Scale Accelerometer Range = 2g -> ACCEL_CONFIG 0x1C)
  writeRegister(0x1C, ACC_FULL_SCALE_2_G);

  delay(10);
}

void loop() {
  if (Serial.available() > 0) {
    char inChar = Serial.read();

    if (inChar == 'H') {
      for (int i = 0; i < 300; i++) {
        startTime = micros();

        // Solicitar los 14 bytes a partir del registro ACCEL_XOUT_H (0x3B)
        Wire.beginTransmission(MPU6500_ADDRESS);
        Wire.write(0x3B);
        Wire.endTransmission(false);
        Wire.requestFrom((uint8_t)MPU6500_ADDRESS, (uint8_t)14);

        int idx = 0;
        while (Wire.available() && idx < 14) {
          GirAcel[idx++] = Wire.read();
        }

        // Dados crus
        raw_accelx = (int16_t)(GirAcel[0] << 8 | GirAcel[1]);
        raw_accely = (int16_t)(GirAcel[2] << 8 | GirAcel[3]);
        raw_accelz = (int16_t)(GirAcel[4] << 8 | GirAcel[5]);
        raw_temp   = (int16_t)(GirAcel[6] << 8 | GirAcel[7]);
        raw_gyrox  = (int16_t)(GirAcel[8] << 8 | GirAcel[9]);
        raw_gyroy  = (int16_t)(GirAcel[10] << 8 | GirAcel[11]);
        raw_gyroz  = (int16_t)(GirAcel[12] << 8 | GirAcel[13]);

        // Retardo preciso
        delayMicroseconds(8380);

        timer_sec = (micros() - startTime) / 1000000.0;

        // Impresión directa sin snprintf para evitar el bug de %f en AVR
        Serial.print(i);
        Serial.print(" ");
        Serial.print(timer_sec, 4);
        Serial.print(" ");
        Serial.print(raw_accelx);
        Serial.print(" ");
        Serial.print(raw_accely);
        Serial.print(" ");
        Serial.print(raw_temp);
        Serial.print(" ");
        Serial.print(raw_accelz);
        Serial.print(" ");
        Serial.print(raw_gyrox);
        Serial.print(" ");
        Serial.print(raw_gyroy);
        Serial.print(" ");
        Serial.println(raw_gyroz);
      }
    }
  }
}
