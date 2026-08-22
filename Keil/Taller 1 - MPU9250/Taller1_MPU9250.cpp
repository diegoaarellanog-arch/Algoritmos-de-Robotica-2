// Taller 1 MPU9250
// Universidad ECCI
// STM32F767ZIT6U

#include <stdio.h>
#include "stm32f7xx.h"
#include <string.h>

//MPU6050
#define MPU6500_address 0x68 // Dirección de la MPU6500 (giroscopio y acelerómetro)

// Escalas del giroscopio
#define     GYRO_FULL_SCALE_250_DPS    0x00 // ESCALA_250 (°/s) = 0 (0x00 = 000|00|000)
#define     GYRO_FULL_SCALE_500_DPS    0x08 // ESCALA_500 (°/s) = 1 (0x08 = 000|01|000)
#define     GYRO_FULL_SCALE_1000_DPS   0x10 // ESCALA_1000 (°/s) = 2 (0x10 = 000|10|000)
#define     GYRO_FULL_SCALE_2000_DPS   0x18 // ESCALA_2000 (°/s) = 3 (0x18 = 000|11|000)

// Escalas del acelerómetro
#define     ACC_FULL_SCALE_2_G         0x00 // ESCALA_2_G (g) = 0 (0x00 = 000|00|000)
#define     ACC_FULL_SCALE_4_G         0x08 // ESCALA_4_G (g) = 1 (0x08 = 000|01|000)
#define     ACC_FULL_SCALE_8_G         0x10 // ESCALA_8_G (g) = 2 (0x10 = 000|10|000)
#define     ACC_FULL_SCALE_16_G        0x18 // ESCALA_16_G (g) = 3 (0x18 = 000|11|000)

// Escalas de conversión (Las tasas de conversión se especifican en la documentación)
#define SENSITIVITY_ACCEL     2.0/32768.0             // Valor de conversión del Acelerómetro (g/LSB) para 2g y 16 bits de longitud de palabra
#define SENSITIVITY_GYRO      250.0/32768.0           // Valor de conversión del Giroscopio ((°/s)/LSB) para 250 °/s y 16 bits de longitud de palabra
#define SENSITIVITY_TEMP      333.87                  // Valor de sensibilidad del Termómetro (Datasheet: MPU-9250 Product Specification, pág. 12)
#define TEMP_OFFSET           21                      // Valor de offset del Termómetro (Datasheet: MPU-6050 Product Specification, pág. 12)

// Offsets de calibración (AQUÍ DEBEN IR LOS VALORES DETERMINADOS EN LA CALIBRACIÓN PREVIA CON EL CÓDIGO "calibracao.ino")
//double offset_accelx = 334.0, offset_accely = -948.0, offset_accelz = 16252.0;
//double offset_gyrox = 111.0, offset_gyroy = 25.0, offset_gyroz = -49.0;

// Valores "RAW" de tipo entero
int16_t raw_accelx, raw_accely, raw_accelz;
int16_t raw_gyrox, raw_gyroy, raw_gyroz;
int16_t raw_temp;

// Salidas calibradas
float accelx, accely, accelz;
float gyrox, gyroy, gyroz;
float temp;

uint8_t data[1];
uint8_t GirAcel[14];

uint8_t flag = 0, j, cont = 0;
int i;
unsigned char d;
char text[100], text1[60]={"PRUEBA DE CONEXION PARA EL GIROSCOPIO Y EL ACELEROMETRO \n\r"}; 
char text2[36]={"Error de conexion con la MPU6050 \n\r"};
char text3[55]={"Ups... No soy la MPU6050, Quien soy? :S. Yo soy:"};
char text4[40]={"Conexion exitosa con la MPU6050 \n\r"};
char text5[45]={"Hola, todo bien?... Yo soy la MPU6050 XD \n\r"};
char text8[27]={"Comunicacion Serial: OK \n\r"}; 
char text9[70]={"-------------------------------------------------------------------\n\r"};
unsigned char cmd[1];

float timer = 0.0, t_fin = 1.0, cont_timer = 0.0;
char text6[40];
char text7[5]={"A\n"};

//I2C
uint8_t ReadI2C1(uint8_t Address, uint8_t Register, uint8_t *Data, uint8_t bytes);
uint8_t WriteI2C1(uint8_t Address, uint8_t Register, uint8_t *Data, uint8_t bytes);
uint8_t I2C1_WaitFlag(uint32_t flag);

void Print(char *data, int n);
void delay(void);
void I2C1_Bus_Reset(void);

void SysTick_Wait(uint32_t n){
    SysTick->LOAD = n - 1; //15999
    SysTick->VAL = 0; // Limpiar el valor del contador Systick
    while (((SysTick->CTRL & 0x00010000) >> 16) == 0); // Verificar la bandera de conteo hasta que sea 1 
}

void SysTick_ms(uint32_t x){
    for (uint32_t i = 0; i < x; i++){ // x ms
        SysTick_Wait(16000); // 1ms
    }
}

extern "C"{
    void EXTI15_10_IRQHandler(void){
        EXTI->PR |= 1; // Bajar bandera
        if(((GPIOC->IDR & (1<<13)) >> 13) == 1){
            flag = 1;
        }
    }

    void USART3_IRQHandler(void){ // Interrupción de recepción
        if(((USART3->ISR & 0x20) >> 5) == 1){ // Los datos recibidos están listos para ser leídos (bandera RXNE = 1)
            d = USART3->RDR; // Leer los datos recibidos por USART 
            if(d == 'H'){
                flag = 1;
            }
        }
    }
}

int main(){
    //----------------------------------------------------------------------------
    //                                  GPIOs
    //----------------------------------------------------------------------------
    RCC->AHB1ENR |= ((1<<1)|(1<<2)); 

    GPIOB->MODER &= ~((0b11<<0)|(0b11<<14));
    GPIOB->MODER |= ((1<<0)|(1<<14)); 
    GPIOC->MODER &= ~(0b11<<26);

    GPIOB->OTYPER &= ~((1<<0)|(1<<7));
    GPIOB->OSPEEDR |= (((1<<1)|(1<<0)|(1<<15)|(1<<14)));
    GPIOC->OSPEEDR |= ((1<<27)|(1<<26));
    GPIOB->PUPDR &= ~((0b11<<0)|(0b11<<14));
    GPIOC->PUPDR &= ~(0b11<<26);
    GPIOC->PUPDR |= (1<<27);
	
		GPIOB->ODR &= ~(1<<0); // Reset the Pin PB0

    //----------------------------------------------------------------------------
    //                                 Systick
    //----------------------------------------------------------------------------
    SysTick->LOAD = 0x00FFFFFF; 
    SysTick->CTRL |= (0b101);

    //----------------------------------------------------------------------------
    //                                Interrupción
    //----------------------------------------------------------------------------
    RCC->APB2ENR |= (1<<14); 
    SYSCFG->EXTICR[3] &= ~(0b1111<<4); 
    SYSCFG->EXTICR[3] |= (1<<5); 
    EXTI->IMR |= (1<<13); 
    EXTI->RTSR |= (1<<13);
    NVIC_EnableIRQ(EXTI15_10_IRQn); 
        
    //----------------------------------------------------------------------------
    //                                  UART
    //----------------------------------------------------------------------------
    RCC->AHB1ENR |= (1<<3); 
    GPIOD->MODER |= (1<<19)|(1<<17); 
    GPIOD->AFR[1] |= (0b111<<4)|(0b111<<0); 
    RCC->APB1ENR |= (1<<18); 
    USART3->BRR = 0x683; 
    USART3->CR1 |= ((1<<5)|(0b11<<2)); 
    NVIC_EnableIRQ(USART3_IRQn);
    

    //----------------------------------------------------------------------------
    //                                  I2C
    //----------------------------------------------------------------------------
    RCC->AHB1ENR |= (1<<1); // Habilitar reloj de GPIOB (PB9=I2C1_SDA y PB8=I2C1_SCL)
    GPIOB->MODER |= (1<<19)|(1<<17); // Configurar (10) pines PB9 (bits 19:18) y PB8 (bits 17:16) como función alternativa
    GPIOB->OTYPER |= (1<<9)|(1<<8); // Configurar (1) pin PB9 (bit 9) y pin PB8 (bit 8) como salida dreno abierto (ALTO o BAJO)
    GPIOB->OSPEEDR |= (0b11<<18)|(0b11<<16); // Configurar (11) pin PB9 (bits 19:18) y pin PB8 (bits 17:16) como Velocidad Muy Alta
    GPIOB->PUPDR|= (1<<18)|(1<<16); // Configurar (01) pin PB9 (bits 19:18) y pin PB8 (bits 17:16) como pull up
    GPIOB->AFR[1] |= (1<<6)|(1<<2); // Configurar la función alternativa I2C1 (AF4) para los pines PB9=I2C1_SDA (bits 7:4) y PB8=I2C1_SCL (bits 3:0)
    RCC->APB1ENR |= (1<<21); // Habilitar reloj de I2C1
    RCC->DCKCFGR2 |= (1<<17); // Configurar (10) bits 17:16 para seleccionar el reloj HSI como fuente para I2C1
    I2C1->CR1 &= ~(1<<0); // Limpiar la habilitación de I2C1
    I2C1->TIMINGR |= 0x30420F13; // Tabla 207 del manual de referencia
    I2C1->CR1 |= (1<<0); // Habilitar I2C1
        
    //----------------------------------------------------------------------------
    //                                 TEMPORIZADOR
    //----------------------------------------------------------------------------
    RCC->APB1ENR |= (1<<1); // Habilitar el reloj del TEMPORIZADOR3 
    TIM3->PSC = 24; // Factor de preescalado 25 para 100ms de tiempo
    TIM3->ARR = 63999; // Valor máximo de conteo
        
    RCC->APB1ENR |= (1<<3); // Habilitar el reloj del TEMPORIZADOR5 
    TIM5->PSC = 24; // Factor de preescalado 25 para 100ms de tiempo
    TIM5->ARR = 10000000; // Valor máximo de conteo
    

    USART3->CR1 |= (1<<0);
    
    SysTick_ms(1000);
    
    Print(text9, strlen(text9));
    Print(text1, strlen(text1));
    Print(text9, strlen(text9));
    Print(text8, strlen(text8));

    //----------------------------------------------------------------------------
    //                                  MPU6050
    //----------------------------------------------------------------------------
		I2C1_Bus_Reset();
    cmd[0] = 0x00; 
		GPIOB->ODR |= 1<<0; // Set the Pin PB0
    WriteI2C1(MPU6500_address, 0x6B, cmd, 1); // Desactiva modo de hibernación de la MPU6050
    GPIOB->ODR |= 1<<0; // Set the Pin PB0
    //.....................................................................
    //        Quien soy yo para la MPU6050 (giroscopio y acelerómetro)
    //.....................................................................
    ReadI2C1(MPU6500_address, 0x75, data, 14);
    if (data[0] != 0x71 & data[0] != 0x68) { // DEFAULT_REGISTER_WHO_AM_I_MPU6050 0x68
    Print(text2, strlen(text2));
    sprintf(text3, "Ups... No soy la MPU6050, Quien soy? :S. Yo soy: %#x \n\r", data[0]);
    Print(text3, strlen(text3));
    while (1);
    }else{
        Print(text4, strlen(text4));
        Print(text5, strlen(text5));
    }
    SysTick_ms(100);
    //.....................................................................
    //        Configuración de los sensores giroscopio y acelerómetro
    //.....................................................................
    cmd[0] = 0x00;
    WriteI2C1(MPU6500_address, 0x1B, cmd, 1);   
    WriteI2C1(MPU6500_address, 0x1C, cmd, 1);   
    SysTick_ms(10);
    
    while(1){
        if(flag == 1){
            flag = 0;
            i = 1;
            while(1){
                TIM5->CNT = 0;
                TIM5->CR1 |= (1<<0); // Habilitar conteo                                      
                ReadI2C1(MPU6500_address, 0x3B, GirAcel, 14);
                raw_accelx = GirAcel[0]<<8 | GirAcel[1];    
                raw_accely = GirAcel[2]<<8 | GirAcel[3];
                raw_accelz = GirAcel[4]<<8 | GirAcel[5];
                raw_temp = GirAcel[6]<<8 | GirAcel[7];
                raw_gyrox = GirAcel[8]<<8 | GirAcel[9];
                raw_gyroy = GirAcel[10]<<8 | GirAcel[11];
                raw_gyroz = GirAcel[12]<<8 | GirAcel[13];
                //SysTick_ms(1);    
                delay();
                // Datos escalados
                //accelx = raw_accelx*SENSITIVITY_ACCEL;
                //accely = raw_accely*SENSITIVITY_ACCEL;
                //accelz = raw_accelz*SENSITIVITY_ACCEL;
                //gyrox = raw_gyrox*SENSITIVITY_GYRO;
                //gyroy = raw_gyroy*SENSITIVITY_GYRO;
                //gyroz = raw_gyroz*SENSITIVITY_GYRO;
                //temp = (raw_temp/SENSITIVITY_TEMP)+21;
                TIM5->CR1 &= ~(1<<0); // Deshabilitar conteo           
                timer = TIM5->CNT*0.0000000625;
                cont_timer += timer;
                //sprintf(text6,"El tiempo es %f segundos \n", timer);
                //Print(text6, strlen(text6));
                sprintf(text,"%d \t %.4f \t %.4f \t %d \t %d \t %d \t %d \t %d \t %d \t %d \n",i++, timer, cont_timer, raw_accelx, raw_accely, raw_accelz, raw_gyrox, raw_gyroy, raw_gyroz, raw_temp);
                //sprintf(text,"%d \t %.2f \t %.2f \t %.2f \t %.2f \t %.2f \t %.2f \t %.2f \n\r",i+1,accelx, accely, accelz, gyrox, gyroy, gyroz, temp);
                Print(text, strlen(text));
                if(cont_timer >= t_fin){
                    cont_timer = 0;
                    Print(text7, strlen(text7));
                    break;
                }
                                
            }
        }
    }
}

#define I2C_TIMEOUT 20000 // Valor de iteraciones máximas para el tiempo de espera

// Función auxiliar para esperar banderas I2C de forma segura
uint8_t I2C1_WaitFlag(uint32_t flag) {
    uint32_t timeout = I2C_TIMEOUT;
    
    while ((I2C1->ISR & flag) == 0) {
        // 1. Detectar si el esclavo no respondió (NACK)
        if (I2C1->ISR & I2C_ISR_NACKF) {
            I2C1->ICR |= I2C_ICR_NACKCF; // Limpiar la bandera NACK
            I2C1->CR2 |= I2C_CR2_STOP;   // Forzar condición de STOP
            return 1;                    // Error por NACK
        }
        
        // 2. Control por Timeout si el hardware no responde
        if (--timeout == 0) {
            I2C1->CR2 |= I2C_CR2_STOP;   // Intentar liberar el bus
            I2C1->CR1 &= ~I2C_CR1_PE;    // Reiniciar periférico I2C1 (Deshabilitar)
            I2C1->CR1 |= I2C_CR1_PE;     // Habilitar I2C1
            return 1;                    // Error por Timeout
        }
    }
    return 0; // Operación exitosa
}

uint8_t WriteI2C1(uint8_t Address, uint8_t Register, uint8_t *Data, uint8_t bytes) {
    uint8_t n;
    
    I2C1->CR2 &= ~(0x3FF << 0);
    I2C1->CR2 |= (Address << 1);

    I2C1->CR2 &= ~(1 << 10);              // Modo Escritura
    I2C1->CR2 &= ~(0xFF << 16);
    I2C1->CR2 |= ((bytes + 1) << 16);     // Bytes a transmitir (Registro + Datos)
    I2C1->CR2 |= (1 << 25);               // AUTOEND habilitado

    I2C1->CR2 |= (1 << 13);               // Generar START

    // Esperar TXIS para enviar la dirección del registro
    if (I2C1_WaitFlag(I2C_ISR_TXIS)) return 1;
    I2C1->TXDR = Register;

    n = bytes;
    while (n > 0) {
        // Esperar TXIS para enviar cada byte de datos
        if (I2C1_WaitFlag(I2C_ISR_TXIS)) return 1;
        I2C1->TXDR = *Data;
        Data++;
        n--;
    }
    
    // Esperar bandera de detección STOPF (generada por AUTOEND)
    if (I2C1_WaitFlag(I2C_ISR_STOPF)) return 1;
    I2C1->ICR |= I2C_ICR_STOPCF;          // Limpiar bandera STOPF
    
    return 0; // Exito
}

uint8_t ReadI2C1(uint8_t Address, uint8_t Register, uint8_t *Data, uint8_t bytes) {
    uint8_t n;

    // --- FASE 1: Escritura de la dirección del registro ---
    I2C1->CR2 &= ~(0x3FF << 0);
    I2C1->CR2 |= (Address << 1);

    I2C1->CR2 &= ~(1 << 10);              // Modo Escritura
    I2C1->CR2 &= ~(0xFF << 16);
    I2C1->CR2 |= (1 << 16);               // 1 byte (Registro)
    I2C1->CR2 &= ~(1 << 25);              // AUTOEND deshabilitado (para hacer RESTART)

    I2C1->CR2 |= (1 << 13);               // Generar START
    
    if (I2C1_WaitFlag(I2C_ISR_TXIS)) return 1;
    I2C1->TXDR = Register;

    if (I2C1_WaitFlag(I2C_ISR_TC)) return 1; // Esperar Transfer Complete (TC)

    // --- FASE 2: Lectura de datos desde el dispositivo ---
    I2C1->CR2 |= (1 << 10);               // Modo Lectura
    I2C1->CR2 &= ~(0xFF << 16);
    I2C1->CR2 |= (bytes << 16);           // Bytes a recibir
    I2C1->CR2 &= ~(1 << 25);              // AUTOEND deshabilitado

    I2C1->CR2 |= (1 << 13);               // Generar RE-START

    n = bytes;
    while (n > 0) {
        if (I2C1_WaitFlag(I2C_ISR_RXNE)) return 1; // Esperar RXNE (Dato recibido)
        *Data = I2C1->RXDR;
        Data++;
        n--;
    }

    I2C1->CR2 |= (1 << 14);               // Generar STOP por software

    if (I2C1_WaitFlag(I2C_ISR_STOPF)) return 1; // Esperar STOPF
    I2C1->ICR |= I2C_ICR_STOPCF;          // Limpiar bandera STOPF

    return 0; // Exito
}

void Print(char *data, int n){
    for(j=0; j<n; j++){
        USART3->TDR = *data; 
        data++;
        while(((USART3->ISR & 0x80) >> 7) == 0){} 
    }
    //USART3->TDR = 0x0A; 
    //while((USART3->ISR & 0x80)==0){};
    USART3->TDR = 0x0D; 
    while(((USART3->ISR & 0x80) >> 7) == 0){}
}

void delay(void){
    TIM3->CNT = 0;
    TIM3->CR1 |= (1<<0); // Habilitar conteo
    //while(TIM5->CNT < 16000); //1ms  
    while(TIM3->CNT < 8000); //0.5ms
    //while(TIM5->CNT < 128000); //8.51ms=8150us
    TIM3->CR1 &= ~(1<<0); // Deshabilitar conteo    
    
    for(j=0; j<=7; j++){
        TIM3->CNT = 0;
        TIM3->CR1 |= (1<<0); // Habilitar conteo
        while(TIM3->CNT < 16000); //1ms 
        TIM3->CR1 &= ~(1<<0); // Deshabilitar conteo 
    }
}

void I2C1_Bus_Reset(void) {
    // 1. Configurar PB8 (SCL) y PB9 (SDA) como Salida Push-Pull temporalmente
    GPIOB->MODER &= ~((0b11 << 16) | (0b11 << 18));
    GPIOB->MODER |=  ((0b01 << 16) | (0b01 << 18));
    
    // 2. Generar 9 pulsos en SCL para liberar la línea SDA retenida por el esclavo
    for (int k = 0; k < 9; k++) {
        GPIOB->BSRR = (1 << 8);  // SCL ALTO
        SysTick_ms(1);
        GPIOB->BSRR = (1 << 24); // SCL BAJO
        SysTick_ms(1);
    }
    
    // 3. Generar condición de STOP manual
    GPIOB->BSRR = (1 << 9);  // SDA ALTO
    SysTick_ms(1);
    
    // 4. Restaurar pines PB8 y PB9 a Función Alternativa AF4 (I2C1)
    GPIOB->MODER &= ~((0b11 << 16) | (0b11 << 18));
    GPIOB->MODER |=  ((0b10 << 16) | (0b10 << 18));
}