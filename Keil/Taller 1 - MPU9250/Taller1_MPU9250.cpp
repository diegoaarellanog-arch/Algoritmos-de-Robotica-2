//Ejemplo I2C
//INTEGRANDO UN MAGNETOMETRO EN BASE DEL CODIGO DEL PROFE FABIAN
//Fabián Barrera Prieto
//Universidad ECCI
//STM32F767ZIT6U
//operation 'or' (|) for set bit and operation 'and' (&) for clear bit

#include <stdio.h>
#include "stm32f7xx.h"
#include <string.h>

char text_mag_err[] = "Error: AK8963 no encontrado (Bucle I2C podria colgarse)\n\r";
char text_mag_ok[]  = "Magnetometro AK8963 OK! \n\r";

//MPU6050
#define MPU6500_address 0x68 // Endereço da MPU6500 (giroscópio e acelerômetro)

#define MPU_CONFIG_REG       0x1A
#define MPU_ACCEL_CONFIG2    0x1D

// Opciones de ancho de banda para el DLPF del Giroscopio/Temperatura (Registro 0x1A)
#define DLPF_BW_250HZ        0x00
#define DLPF_BW_184HZ        0x01
#define DLPF_BW_92HZ         0x02
#define DLPF_BW_41HZ         0x03
#define DLPF_BW_20HZ         0x04
#define DLPF_BW_10HZ         0x05
#define DLPF_BW_5HZ          0x06

// Escalas do girôscopio
#define    GYRO_FULL_SCALE_250_DPS    0x00 
#define    GYRO_FULL_SCALE_500_DPS    0x08 
#define    GYRO_FULL_SCALE_1000_DPS   0x10 
#define    GYRO_FULL_SCALE_2000_DPS   0x18 

// Escalas do acelerômetro
#define    ACC_FULL_SCALE_2_G         0x00 
#define    ACC_FULL_SCALE_4_G         0x08 
#define    ACC_FULL_SCALE_8_G         0x10 
#define    ACC_FULL_SCALE_16_G        0x18 

//----------------------------------------------------------------------------
//                       MAGNETOMETRO (AK8963)
//----------------------------------------------------------------------------
#define AK8963_address        0x0C  // Dirección I2C del magnetómetro
#define AK8963_WIA            0x00  // Who Am I (Debe ser 0x48)
#define AK8963_ST1            0x02  // Status 1 (bit0 = DRDY, dato listo)
#define AK8963_HXL            0x03  // Inicio de datos (X,Y,Z low/high)
#define AK8963_ST2            0x09  // Status 2 (bit3 = HOFL) - OBLIGATORIO LEERLO
#define AK8963_CNTL1          0x0A  // Control 1 (Modo de operación)
#define AK8963_ASAX           0x10  // Ajustes de sensibilidad de fábrica

// Registros de la MPU para activar el puente (Bypass)
#define MPU_INT_PIN_CFG       0x37  
#define MPU_USER_CTRL         0x6A

#define MAG_SENSITIVITY       4912.0/32760.0 // Factor de conversión uT

// Variables para el Magnetómetro
int16_t raw_magx, raw_magy, raw_magz;
float magx, magy, magz;
float mag_asa_x, mag_asa_y, mag_asa_z;
uint8_t MagData[7]; 
uint8_t asa_raw[3];

// Escalas de conversao (As taxas de conversão são especificadas na documentação)
#define SENSITIVITY_ACCEL     2.0/32768.0             
#define SENSITIVITY_GYRO      250.0/32768.0           
#define SENSITIVITY_TEMP      333.87                  
#define TEMP_OFFSET           21                      

// Valores "RAW" de tipo inteiro
int16_t raw_accelx, raw_accely, raw_accelz;
int16_t raw_gyrox, raw_gyroy, raw_gyroz;
int16_t raw_temp;

// Saídas calibradas
float accelx, accely, accelz;
float gyrox, gyroy, gyroz;
float temp;

uint8_t data[1];
uint8_t GirAcel[14];

// IMPORTANTE: el volatile para que no se quede ciego el condicional
volatile uint8_t flag = 0, j, cont = 0;
int i;
unsigned char d;
char text[120], text1[60]={"TESTE DE CONEXAO PARA O GIROSCOPIO E O ACELEROMETRO \n\r"}; 
char text2[35]={"Erro de conexao com a MPU6050 \n\r"};
char text3[55]={"Opaaa. Eu nao sou a MPU6050, Quem sou eu? :S. I am:"};
char text4[40]={"Conexao bem sucedida com a MPU6050 \n\r"};
char text5[45]={"Oi, tudo joia?... Eu sou a MPU6050 XD \n\r"};
unsigned char cmd[1];

float timer = 0.0, cont_timer = 0.0;
char text6[40];
char text7[5]={"A\n"};

//I2C Prototypes
void ReadI2C1(uint8_t Address, uint8_t Register, uint8_t *Data, uint8_t bytes);
void WriteI2C1(uint8_t Address, uint8_t Register, uint8_t *Data, uint8_t bytes);

void Print(char *data, int n);
void delay(void);

//magnetometro I2C Prototypes
void MPU_HabilitarBypass(void);
void AK8963_Init(void);
void AK8963_Leer(void);

void SysTick_Wait(uint32_t n){
    SysTick->LOAD = n - 1; 
    SysTick->VAL = 0; 
    while (((SysTick->CTRL & 0x00010000) >> 16) == 0); 
}

void SysTick_ms(uint32_t x){
    for (uint32_t i = 0; i < x; i++){
        SysTick_Wait(16000); 
    }
}

extern "C"{
    void EXTI15_10_IRQHandler(void){
        // 1. Limpiamos la bandera del PIN 13 para que no se congele
        EXTI->PR |= (1<<13); 

        // 2. ¡EL BRILLO! Hacemos "Toggle" (alternar) al LED Verde en PB0
        GPIOB->ODR ^= (1<<0); 

        // 3. Activamos la bandera para el main
        if(((GPIOC->IDR & (1<<13)) >> 13) == 1){
            flag = 1;
        }
    }

    void USART3_IRQHandler(void){ 
        if(((USART3->ISR & 0x20) >> 5) == 1){
            d = USART3->RDR;
            if(d == 'H'){
                GPIOB->ODR ^= (1<<0); 
                flag = 1;
            }
        }
    }
}
    
int main(){
    //----------------------------------------------------------------------------
    //                       GPIOs
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

    //----------------------------------------------------------------------------
    //                       Systick
    //----------------------------------------------------------------------------
    SysTick->LOAD = 0x00FFFFFF; 
    SysTick->CTRL |= (0b101);

    //----------------------------------------------------------------------------
    //                       Interrupt
    //----------------------------------------------------------------------------
    RCC->APB2ENR |= (1<<14); 
    SYSCFG->EXTICR[3] &= ~(0b1111<<4); 
    SYSCFG->EXTICR[3] |= (1<<5); 
    EXTI->IMR |= (1<<13); 
    EXTI->RTSR |= (1<<13);
    NVIC_EnableIRQ(EXTI15_10_IRQn); 
        
    //----------------------------------------------------------------------------
    //                       UART
    //----------------------------------------------------------------------------
    RCC->AHB1ENR |= (1<<3); 
    GPIOD->MODER |= (1<<19)|(1<<17); 
    GPIOD->AFR[1] |= (0b111<<4)|(0b111<<0); 
    RCC->APB1ENR |= (1<<18); 
    USART3->BRR = 0x683; 
    USART3->CR1 |= ((1<<5)|(0b11<<2)); 
    NVIC_EnableIRQ(USART3_IRQn);

    //----------------------------------------------------------------------------
    //                       I2C
    //----------------------------------------------------------------------------
    RCC->AHB1ENR |= (1<<1); 
    GPIOB->MODER |= (1<<19)|(1<<17); 
    GPIOB->OTYPER |= (1<<9)|(1<<8); 
    GPIOB->OSPEEDR |= (0b11<<18)|(0b11<<16); 
    GPIOB->PUPDR|= (1<<18)|(1<<16); 
    GPIOB->AFR[1] |= (1<<6)|(1<<2); 
    RCC->APB1ENR |= (1<<21); 
    RCC->DCKCFGR2 |= (1<<17); 
    I2C1->CR1 &= ~(1<<0);
    I2C1->TIMINGR |= 0x30420F13;
    I2C1->CR1 |= (1<<0);
        
    //----------------------------------------------------------------------------
    //                       TIMER
    //----------------------------------------------------------------------------
    RCC->APB1ENR |= (1<<1);  
    TIM3->PSC = 24; 
    TIM3->ARR = 63999; 
        
    RCC->APB1ENR |= (1<<3); 
    TIM5->PSC = 24; 
    TIM5->ARR = 10000000; 
    
    USART3->CR1 |= (1<<0);
    SysTick_ms(1000);

    //----------------------------------------------------------------------------
    //                       MPU6050
    //----------------------------------------------------------------------------
    cmd[0] = 0x00;  
    WriteI2C1(MPU6500_address, 0x6B, cmd, 1); 
    Print(text1, strlen(text1));
        
    ReadI2C1(MPU6500_address, 0x75, data, 14);
    if (data[0] != 0x71) { 
        Print(text2, strlen(text2));
        sprintf(text3,"%s %#x \n\r",data[0]);
        Print(text3, strlen(text3));
        while (1);
    } else {
        Print(text4, strlen(text4));
        Print(text5, strlen(text5));
    }
    
    SysTick_ms(100);

    // Configuracao dos sensores giroscópio e acelerômetro
    cmd[0] = 0x00;
    WriteI2C1(MPU6500_address, 0x1B, cmd, 1);   
    WriteI2C1(MPU6500_address, 0x1C, cmd, 1);   
    SysTick_ms(10);
		
    // =========================================================================
    //                    ACTIVAR FILTRO PASABAJAS (DLPF)
    // =========================================================================
    // Configura el DLPF para Giroscopio y Temperatura (Registro 0x1A)
    // Se recomienda 41Hz o 20Hz para un buen balance entre atenuación de ruido y latencia.
    cmd[0] = DLPF_BW_41HZ; 
    WriteI2C1(MPU6500_address, MPU_CONFIG_REG, cmd, 1);
    SysTick_ms(10);

    // Configura el DLPF para el Acelerómetro (Registro 0x1D)
    // 0x03 activa el DLPF del acelerómetro a ~41Hz
    cmd[0] = 0x03; 
    WriteI2C1(MPU6500_address, MPU_ACCEL_CONFIG2, cmd, 1);
    SysTick_ms(10);
		
    // ======== INICIALIZAR MAGNETOMETRO ========
    MPU_HabilitarBypass();
    SysTick_ms(10);
    AK8963_Init();
    SysTick_ms(10);
        
    while(1){
        if(flag == 1){
            flag = 0;
            i = 1;
                    
            char msg_test[] = "Boton detectado, entrando a leer...\n\r";
            Print(msg_test, strlen(msg_test));
                    
            while(1){
                TIM5->CNT = 0;
                TIM5->CR1 |= (1<<0);                                      
                ReadI2C1(MPU6500_address, 0x3B, GirAcel, 14);
                raw_accelx = GirAcel[0]<<8 | GirAcel[1];    
                raw_accely = GirAcel[2]<<8 | GirAcel[3];
                raw_accelz = GirAcel[4]<<8 | GirAcel[5];
                raw_temp = GirAcel[6]<<8 | GirAcel[7];
                raw_gyrox = GirAcel[8]<<8 | GirAcel[9];
                raw_gyroy = GirAcel[10]<<8 | GirAcel[11];
                raw_gyroz = GirAcel[12]<<8 | GirAcel[13];
                
                delay();
                
                TIM5->CR1 &= ~(1<<0);           
                timer = TIM5->CNT*0.0000000625;
                cont_timer += timer;
                                
                // ======== LEER MAGNETOMETRO ========
                AK8963_Leer();
                // ===================================
                                
                // ========  SPRINTF CON MAG ========
                sprintf(text,"%d %.4f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f \n\r", 
                        i++, timer, 
                        (float)raw_accelx, (float)raw_accely, (float)raw_accelz, 
                        (float)raw_gyrox, (float)raw_gyroy, (float)raw_gyroz,
                        magx, magy, magz);
                Print(text, strlen(text));
                                                
                // ====================
                // SALIR CUANDO LLEGUE A 300 DATOS EXACTOS
                if(i > 300){
                    cont_timer = 0;
                    break;
                }
            }
        }
    }
}

// ----------------------------------------------------------------------------
//                FUNCIONES I2C CON ESCUDO ANTI-BLOQUEO
// ----------------------------------------------------------------------------
void WriteI2C1(uint8_t Address, uint8_t Register, uint8_t *Data, uint8_t bytes){
    uint8_t n; 
    uint32_t timeout; // <--- ESCUDO ANTI-BLOQUEO
    
    I2C1->CR2 &= ~(0x3FF<<0);
    I2C1->CR2 |= (Address<<1);
    I2C1->CR2 &= ~(1<<10);
    I2C1->CR2 &= ~(0xFF<<16);
    I2C1->CR2 |= ((bytes+1)<<16);
    I2C1->CR2 |= (1<<25);
    I2C1->CR2 |= (1<<13);

    timeout = 100000;
    while (((I2C1->ISR) & (1<<1)) != (0b10)){
        if(--timeout == 0) return; // Si se traba, aborta y salva el programa
    }

    I2C1->TXDR = Register;

    n = bytes;
    while(n>0){
        timeout = 100000;
        while (((I2C1->ISR) & (1<<1)) != (0b10)){
            if(--timeout == 0) return; 
        }
        I2C1->TXDR = *Data;
        Data++;
        n--;
    }
    
    timeout = 100000;
    while (((I2C1->ISR) & (1<<5)) != (0b100000)){
        if(--timeout == 0) return;
    }
}

void ReadI2C1(uint8_t Address, uint8_t Register, uint8_t *Data, uint8_t bytes ) {
    uint8_t n;
    uint32_t timeout; // <--- ESCUDO ANTI-BLOQUEO

    I2C1->CR2 &= ~(0x3FF<<0);
    I2C1->CR2 |= (Address<<1);
    I2C1->CR2 &= ~(1<<10);
    I2C1->CR2 &= ~(0xFF<<16);
    I2C1->CR2 |= (1<<16);
    I2C1->CR2 &= ~(1<<25);
    I2C1->CR2 |= (1<<13);
    
    timeout = 100000;
    while (((I2C1->ISR) & (1<<1)) != (0b10)){
        if(--timeout == 0) return; 
    }

    I2C1->TXDR = Register;

    timeout = 100000;
    while (((I2C1->ISR) & (1<<6)) != (0b1000000)){
        if(--timeout == 0) return;
    }

    I2C1->CR2 |= (1<<10);
    I2C1->CR2 &= ~(0xFF<<16);
    I2C1->CR2 |= (bytes<<16);
    I2C1->CR2 &= ~(1<<25);
    I2C1->CR2 |= (1<<13);

    n = bytes;
    while (n>0){
        timeout = 100000;
        while (((I2C1->ISR) & (1<<2)) != (0b100)){
            if(--timeout == 0) return;
        }
        *Data = I2C1->RXDR;
        Data++;
        n--;
    }

    I2C1->CR2 |= (1<<14);

    timeout = 100000;
    while (((I2C1->ISR) & (1<<5)) != (0b100000)){
        if(--timeout == 0) return;
    }
}

void Print(char *data, int n){
    for(j=0; j<n; j++){
        USART3->TDR = *data; 
        data++;
        while(((USART3->ISR & 0x80) >> 7) == 0){} 
    }
    USART3->TDR = 0x0D; 
    while(((USART3->ISR & 0x80) >> 7) == 0){}
}

void delay(void){
    TIM3->CNT = 0;
    TIM3->CR1 |= (1<<0); 
    while(TIM3->CNT < 7344); 
    TIM3->CR1 &= ~(1<<0); 
    
    for(j=0; j<=7; j++){
        TIM3->CNT = 0;
        TIM3->CR1 |= (1<<0); 
        while(TIM3->CNT < 16000); 
        TIM3->CR1 &= ~(1<<0); 
    }
}

//----------------------------------------------------------------------------
//                      FUNCIONES DEL MAGNETOMETRO AK8963
//----------------------------------------------------------------------------
void MPU_HabilitarBypass(void){
    cmd[0] = 0x00;
    WriteI2C1(MPU6500_address, MPU_USER_CTRL, cmd, 1);
    SysTick_ms(10);

    cmd[0] = 0x02;
    WriteI2C1(MPU6500_address, MPU_INT_PIN_CFG, cmd, 1);
    SysTick_ms(10);
}

void AK8963_Init(void){
    ReadI2C1(AK8963_address, 0x00, data, 1);
    if (data[0] != 0x48){
        Print(text_mag_err, strlen(text_mag_err));
    } else {
        Print(text_mag_ok, strlen(text_mag_ok));
    }

    cmd[0] = 0x00;
    WriteI2C1(AK8963_address, 0x0A, cmd, 1);
    SysTick_ms(10);

    cmd[0] = 0x0F;
    WriteI2C1(AK8963_address, 0x0A, cmd, 1);
    SysTick_ms(10);
    
    ReadI2C1(AK8963_address, 0x10, asa_raw, 3);
    mag_asa_x = ((asa_raw[0] - 128) * 0.5 / 128.0) + 1.0;
    mag_asa_y = ((asa_raw[1] - 128) * 0.5 / 128.0) + 1.0;
    mag_asa_z = ((asa_raw[2] - 128) * 0.5 / 128.0) + 1.0;

    cmd[0] = 0x00;
    WriteI2C1(AK8963_address, 0x0A, cmd, 1);
    SysTick_ms(10);

    cmd[0] = 0x16;
    WriteI2C1(AK8963_address, 0x0A, cmd, 1);
    SysTick_ms(10);
}

void AK8963_Leer(void){
    ReadI2C1(AK8963_address, 0x02, data, 1);
    
    if ((data[0] & 0x01) == 0x01){ 
        ReadI2C1(AK8963_address, 0x03, MagData, 7);

        if ((MagData[6] & 0x08) == 0){ 
            raw_magx = (int16_t)(MagData[1]<<8 | MagData[0]);
            raw_magy = (int16_t)(MagData[3]<<8 | MagData[2]);
            raw_magz = (int16_t)(MagData[5]<<8 | MagData[4]);

            magx = raw_magx * MAG_SENSITIVITY * mag_asa_x;
            magy = raw_magy * MAG_SENSITIVITY * mag_asa_y;
            magz = raw_magz * MAG_SENSITIVITY * mag_asa_z;
        }
    }
}