#include "stm32f7xx.h"
#include <stdio.h>

// Funciones de prototipo
void UART3_Init(void);
void UART3_WriteChar(char c);
void UART3_Print(const char *str);
void Delay_ms(uint32_t ms);

int main(void) {
    // 1. Inicializar el puerto serie USART3
    UART3_Init();

    // Mensaje de bienvenida
    UART3_Print("\r\n=========================================\r\n");
    UART3_Print("  Prueba UART desde Keil uVision\r\n");
    UART3_Print("=========================================\r\n");

    int contador = 0;
    char buffer[50];

    // Bucle principal
    while (1) {
        // Formatear mensaje con el número de iteración
        sprintf(buffer, "[Keil STM32] Mensaje #%d enviado con exito\r\n", contador++);
        
        // Enviar por el puerto serie
        UART3_Print(buffer);

        // Retardo de aproximadamente 1 segundo
        Delay_ms(1000);
    }
}

// ----------------------------------------------------------------------------
// Configuración del periférico USART3 a nivel de registros
// ----------------------------------------------------------------------------
void UART3_Init(void) {
    // 1. Habilitar reloj para GPIOD y USART3
    RCC->AHB1ENR |= (1 << 3);   // Habilita GPIODEN (Bit 3)
    RCC->APB1ENR |= (1 << 18);  // Habilita USART3EN (Bit 18)

    // 2. Configurar pines PD8 (TX) y PD9 (RX) en Función Alternativa (AF7)
    GPIOD->MODER &= ~((0b11 << 16) | (0b11 << 18)); // Limpiar bits de PD8 y PD9
    GPIOD->MODER |=  ((0b10 << 16) | (0b10 << 18)); // Configurar como Función Alternativa (10)

    // Mapear PD8 y PD9 a AF7 (USART3) en el registro AFRH (AFR[1])
    GPIOD->AFR[1] &= ~((0xF << 0) | (0xF << 4)); // Limpiar AF para PD8 y PD9
    GPIOD->AFR[1] |=  ((7 << 0)   | (7 << 4));   // AF7 = 0111

    // 3. Configurar Baud Rate (9600 Baudios @ reloj base HSI de 16 MHz)
    // BRR = 16,000,000 / 9600 = 1666.66 => Hexadecimal 0x683
    USART3->BRR = 0x683;

    // 4. Habilitar Transmisor (TE), Receptor (RE) y el propio periférico USART (UE)
    USART3->CR1 = (1 << 3) | (1 << 2) | (1 << 0); // TE=1, RE=1, UE=1
}

// ----------------------------------------------------------------------------
// Funciones de Transmisión
// ----------------------------------------------------------------------------
void UART3_WriteChar(char c) {
    // Esperar hasta que el registro de transmisión esté vacío (TXE, Bit 7)
    while (!(USART3->ISR & (1 << 7)));
    USART3->TDR = c;
}

void UART3_Print(const char *str) {
    while (*str) {
        UART3_WriteChar(*str++);
    }
}

// Retardo simple por software usando el SysTick
void Delay_ms(uint32_t ms) {
    SysTick->LOAD = 16000 - 1; // 1 ms @ 16 MHz
    SysTick->VAL = 0;
    SysTick->CTRL = 5;         // Habilitar SysTick con reloj del sistema

    for (uint32_t i = 0; i < ms; i++) {
        while (!(SysTick->CTRL & (1 << 16))); // Esperar flag COUNTFLAG
    }
    SysTick->CTRL = 0; // Deshabilitar SysTick
}