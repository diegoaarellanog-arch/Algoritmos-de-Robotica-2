#include "stm32f767xx.h"
#include <stdio.h>

#define MPU_ADDR        0x68    // Dirección I2C de 7 bits (AD0 = GND)
#define REG_WHO_AM_I    0x75    // Registro ID del dispositivo
#define REG_PWR_MGMT_1  0x6B    // Registro de gestión de energía
#define REG_ACCEL_XOUT_H 0x3B   // Inicio de lecturas de acelerómetro

// ----------------------------------------------------------------------------
// Redirección de printf a USART3 (PD8=TX, PD9=RX @ 115200 Baud)
// ----------------------------------------------------------------------------
int _write(int file, char *ptr, int len) {
    for (int i = 0; i < len; i++) {
        while (!(USART3->ISR & USART_ISR_TXE));
        USART3->TDR = ptr[i];
    }
    return len;
}

void UART3_Init(void) {
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN;
    RCC->APB1ENR |= RCC_APB1ENR_USART3EN;

    GPIOD->MODER &= ~((3 << (8 * 2)) | (3 << (9 * 2)));
    GPIOD->MODER |=  ((2 << (8 * 2)) | (2 << (9 * 2))); // AF
    GPIOD->AFR[1] &= ~((0xF << 0) | (0xF << 4));
    GPIOD->AFR[1] |=  ((7 << 0)   | (7 << 4));          // AF7

    USART3->BRR = 139; // 115200 con HSI @ 16MHz
    USART3->CR1 = USART_CR1_TE | USART_CR1_RE | USART_CR1_UE;
}

// ----------------------------------------------------------------------------
// Inicialización I2C1 (PB8=SCL, PB9=SDA)
// ----------------------------------------------------------------------------
void I2C1_Init(void) {
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
    RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;

    GPIOB->MODER   &= ~((3 << (8 * 2)) | (3 << (9 * 2)));
    GPIOB->MODER   |=  ((2 << (8 * 2)) | (2 << (9 * 2))); // AF
    GPIOB->OTYPER  |=  (1 << 8) | (1 << 9);               // Open-Drain
    GPIOB->PUPDR   &= ~((3 << (8 * 2)) | (3 << (9 * 2)));
    GPIOB->PUPDR   |=  ((1 << (8 * 2)) | (1 << (9 * 2))); // Pull-Up
    GPIOB->AFR[1]  &= ~((0xF << 0) | (0xF << 4));
    GPIOB->AFR[1]  |=  ((4 << 0)   | (4 << 4));          // AF4

    I2C1->CR1 &= ~I2C_CR1_PE;
    I2C1->TIMINGR = 0x00C0EAFF; // Standard Mode 100 kHz @ 16MHz
    I2C1->CR1 |= I2C_CR1_PE;
}

// ----------------------------------------------------------------------------
// Funciones de Lectura y Escritura I2C Bare-Metal (STM32F7)
// ----------------------------------------------------------------------------
void I2C1_WriteRegister(uint8_t devAddr, uint8_t regAddr, uint8_t data) {
    I2C1->ICR = I2C_ICR_NACKCF | I2C_ICR_STOPCF;
    
    // Transmisión de 2 bytes: Dirección de registro + Datos
    I2C1->CR2 = (devAddr << 1) | (2 << I2C_CR2_NBYTES_Pos) | I2C_CR2_START | I2C_CR2_AUTOEND;

    // Enviar registro objetivo
    while (!(I2C1->ISR & I2C_ISR_TXIS));
    I2C1->TXDR = regAddr;

    // Enviar valor
    while (!(I2C1->ISR & I2C_ISR_TXIS));
    I2C1->TXDR = data;

    while (!(I2C1->ISR & I2C_ISR_STOPF));
    I2C1->ICR = I2C_ICR_STOPCF;
}

uint8_t I2C1_ReadRegister(uint8_t devAddr, uint8_t regAddr) {
    uint8_t data = 0;
    I2C1->ICR = I2C_ICR_NACKCF | I2C_ICR_STOPCF;

    // 1. Escribir la dirección del registro a consultar (sin AUTOEND)
    I2C1->CR2 = (devAddr << 1) | (1 << I2C_CR2_NBYTES_Pos) | I2C_CR2_START;
    while (!(I2C1->ISR & I2C_ISR_TXIS));
    I2C1->TXDR = regAddr;
    while (!(I2C1->ISR & I2C_ISR_TC)); // Esperar fin de escritura

    // 2. Restart y leer 1 byte con AUTOEND
    I2C1->CR2 = (devAddr << 1) | I2C_CR2_RD_WRN | (1 << I2C_CR2_NBYTES_Pos) | I2C_CR2_START | I2C_CR2_AUTOEND;
    while (!(I2C1->ISR & I2C_ISR_RXNE));
    data = (uint8_t)I2C1->RXDR;

    while (!(I2C1->ISR & I2C_ISR_STOPF));
    I2C1->ICR = I2C_ICR_STOPCF;

    return data;
}

void delay_ms(uint32_t ms) {
    for (volatile uint32_t i = 0; i < ms * 4000; i++);
}

// ----------------------------------------------------------------------------
// Main
// ----------------------------------------------------------------------------
int main(void) {
    UART3_Init();
    I2C1_Init();

    printf("\r\n====================================\r\n");
    printf("   HOLA MUNDO MPU - STM32F767ZI\r\n");
    printf("====================================\r\n");

    // 1. Probar lectura del ID
    uint8_t whoami = I2C1_ReadRegister(MPU_ADDR, REG_WHO_AM_I);
    printf("Registro WHO_AM_I leido: 0x%02X\r\n", whoami);

    if (whoami == 0x68 || whoami == 0x71 || whoami == 0x73) {
        printf(" -> ¡Conexion Exitosa con el MPU!\r\n");
    } else {
        printf(" -> ERROR: Dispositivo no reconocido o lectura erronea.\r\n");
    }

    // 2. Despertar el MPU (Escribir 0 en PWR_MGMT_1)
    I2C1_WriteRegister(MPU_ADDR, REG_PWR_MGMT_1, 0x00);
    printf("MPU despierto y configurado.\r\n\r\n");

    while (1) {
        // Leer los bytes Alto y Bajo de la aceleración en X
        uint8_t accel_x_h = I2C1_ReadRegister(MPU_ADDR, REG_ACCEL_XOUT_H);
        uint8_t accel_x_l = I2C1_ReadRegister(MPU_ADDR, REG_ACCEL_XOUT_H + 1);
        
        int16_t accel_x_raw = (int16_t)((accel_x_h << 8) | accel_x_l);

        printf("Aceleracion X (Raw): %d\r\n", accel_x_raw);
        delay_ms(500);
    }
}}