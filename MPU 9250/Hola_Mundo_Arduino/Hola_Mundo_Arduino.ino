#include <Wire.h>

// Definición de pines para Arduino Uno / Nano
const uint8_t SCL_PIN = A5;
const uint8_t SDA_PIN = A4;

void unstickI2CBus() {
  // 1. Asegurar que las líneas estén liberadas como entradas
  pinMode(SDA_PIN, INPUT_PULLUP);
  pinMode(SCL_PIN, INPUT_PULLUP);
  delay(10);

  // 2. Si SDA está en LOW, el MPU quedó atrapado enviando datos
  if (digitalRead(SDA_PIN) == LOW) {
    Serial.println(F("[I2C RECOVERY] SDA atascado en LOW. Generando pulsos SCL..."));
    
    pinMode(SCL_PIN, OUTPUT);
    
    // Generar 16 pulsos de reloj manuales para desatascar la FSM del MPU
    for (int i = 0; i < 16; i++) {
      digitalWrite(SCL_PIN, LOW);
      delayMicroseconds(10);
      digitalWrite(SCL_PIN, HIGH);
      delayMicroseconds(10);
    }
    
    // Generar condición de STOP manual (SDA pasa de LOW a HIGH mientras SCL está en HIGH)
    pinMode(SDA_PIN, OUTPUT);
    digitalWrite(SDA_PIN, LOW);
    delayMicroseconds(10);
    digitalWrite(SCL_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(SDA_PIN, HIGH);
    delayMicroseconds(10);
    
    // Restaurar a modo entrada
    pinMode(SDA_PIN, INPUT_PULLUP);
    pinMode(SCL_PIN, INPUT_PULLUP);
    
    if (digitalRead(SDA_PIN) == HIGH) {
      Serial.println(F("[I2C RECOVERY] Bus I2C liberado con éxito."));
    } else {
      Serial.println(F("[I2C RECOVERY] Error: SDA sigue retenido en LOW (verificar hardware/CS)."));
    }
  } else {
    Serial.println(F("[I2C RECOVERY] Bus I2C libre. Iniciando..."));
  }
}

void setup() {
  Serial.begin(115200);
  while (!Serial); // Esperar puerto serial en boards con USB nativo

  Serial.println(F("\n--- INICIANDO DIAGNÓSTICO MPU9250 ---"));

  // Rutina de destrabe
  unstickI2CBus();

  // Inicializar periférico I2C por hardware
  Wire.begin();
  Wire.setClock(100000); // Standard Mode 100 kHz para máxima estabilidad
}

void loop() {
  byte error, address;
  int nDevices = 0;

  Serial.println(F("\nEscaneando bus I2C..."));

  for (address = 1; address < 127; address++) {
    Wire.beginTransmission(address);
    error = Wire.endTransmission();

    if (error == 0) {
      Serial.print(F(" -> Dispositivo encontrado en dirección 0x"));
      if (address < 16) Serial.print("0");
      Serial.print(address, HEX);

      // Verificación específica de MPU9250/6050
      if (address == 0x68 || address == 0x69) {
        Serial.print(F(" [MPU9250 / MPU6050 Detectado]"));
      } else if (address == 0x0C) {
        Serial.print(F(" [Magnetómetro AK8963 interno del MPU9250]"));
      }
      Serial.println();

      nDevices++;
    } else if (error == 4) {
      Serial.print(F(" -> Error desconocido en dirección 0x"));
      if (address < 16) Serial.print("0");
      Serial.println(address, HEX);
    }
  }

  if (nDevices == 0) {
    Serial.println(F("No se encontraron dispositivos I2C. Revisa conexiones, CS y Pull-Ups.\n"));
  } else {
    Serial.println(F("Escaneo finalizado.\n"));
  }

  delay(3000); // Escanear cada 3 segundos
}
