// Dos IMU MPU9250 (MPU + AK8963) en el MISMO bus I2C1 usando el pin AD0
// Base: codigo del profe Fabian Barrera Prieto - Universidad ECCI
// STM32F767ZIT6U
//
// CONEXION:
//   IMU 1: AD0 -> GND  => direccion 0x68
//   IMU 2: AD0 -> 3.3V => direccion 0x69
//   SDA -> PB9, SCL -> PB8 (compartidos por las dos IMU), con pull-ups externos
//   (o los internos que ya activa el codigo). VCC 3.3V y GND comunes.
//
// PROBLEMA IMPORTANTE:
//   El magnetometro AK8963 de cada MPU9250 tiene SIEMPRE la direccion 0x0C
//   (no se puede cambiar con AD0). Si los dos puentes (bypass) estan activos a la vez,
//   los dos AK8963 colisionan en el bus.
//   SOLUCION: solo una IMU tiene el bypass activo a la vez. Antes de hablar con un
//   magnetometro, se activa el bypass de esa IMU y se desactiva el de la otra.
//
// operation 'or' (|) for set bit and operation 'and' (&) for clear bit

#include <stdio.h>
#include "stm32f7xx.h"
#include <string.h>

//----------------------------------------------------------------------------
//                       DEFINICIONES
//----------------------------------------------------------------------------
#define N_IMU                 2
#define MPU_ADDR_1            0x68   // AD0 = GND
#define MPU_ADDR_2            0x69   // AD0 = 3.3V

#define MPU_REG_WHO_AM_I      0x75
#define MPU_REG_PWR_MGMT_1    0x6B
#define MPU_REG_GYRO_CONFIG   0x1B
#define MPU_REG_ACCEL_CONFIG  0x1C
#define MPU_REG_ACCEL_XOUT_H  0x3B
#define MPU_INT_PIN_CFG       0x37
#define MPU_USER_CTRL         0x6A

#define BYPASS_ON             0x02
#define BYPASS_OFF            0x00

#define AK8963_address        0x0C
#define AK8963_WIA            0x00   // Debe ser 0x48
#define AK8963_ST1            0x02
#define AK8963_HXL            0x03
#define AK8963_CNTL1          0x0A
#define AK8963_ASAX           0x10

#define MAG_SENSITIVITY       (4912.0f/32760.0f)   // uT por LSB

#define N_MUESTRAS            300

//----------------------------------------------------------------------------
//                       ESTRUCTURA DE CADA IMU
//----------------------------------------------------------------------------
typedef struct {
    uint8_t  addr;                 // direccion I2C del MPU
    uint8_t  mpu_ok;               // 1 si respondio el WHO_AM_I
    uint8_t  mag_ok;               // 1 si respondio el AK8963
    int16_t  raw_ax, raw_ay, raw_az;
    int16_t  raw_gx, raw_gy, raw_gz;
    int16_t  raw_temp;
    int16_t  raw_mx, raw_my, raw_mz;
    float    mx, my, mz;           // uT
    float    asa_x, asa_y, asa_z;  // ajuste de sensibilidad de fabrica
} IMU_t;

IMU_t imu[N_IMU];

//----------------------------------------------------------------------------
//                       VARIABLES GLOBALES
//----------------------------------------------------------------------------
uint8_t data[1];
uint8_t GirAcel[14];
uint8_t MagData[7];
uint8_t asa_raw[3];
unsigned char cmd[1];

// volatile para que el condicional del main vea los cambios hechos en la ISR
volatile uint8_t flag = 0, j;
int i;
unsigned char d;

char text[300];
char text1[] = "TESTE DE CONEXAO PARA 2 IMU (MPU9250 + AK8963) \n\r";
char text_hdr[] = "n t | IMU1: ax ay az gx gy gz mx my mz | IMU2: ax ay az gx gy gz mx my mz \n\r";

float timer = 0.0f, cont_timer = 0.0f;

//----------------------------------------------------------------------------
//                       PROTOTIPOS
//----------------------------------------------------------------------------
void ReadI2C1(uint8_t Address, uint8_t Register, uint8_t *Data, uint8_t bytes);
void WriteI2C1(uint8_t Address, uint8_t Register, uint8_t *Data, uint8_t bytes);
void Print(char *data, int n);
void delay(void);

void IMU_Init(uint8_t k);
void IMU_LeerInercial(uint8_t k);
void Mag_Seleccionar(uint8_t k);
void Mag_Init(uint8_t k);
void Mag_Leer(uint8_t k);

//----------------------------------------------------------------------------
//                       SYSTICK
//----------------------------------------------------------------------------
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

//----------------------------------------------------------------------------
//                       INTERRUPCIONES
//----------------------------------------------------------------------------
extern "C"{
    void EXTI15_10_IRQHandler(void){
        EXTI->PR |= (1<<13);               // limpia bandera del PC13
        GPIOB->ODR ^= (1<<0);              // toggle LED verde PB0
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

//----------------------------------------------------------------------------
//                       MAIN
//----------------------------------------------------------------------------
int main(){
    //------------------------------ GPIOs ------------------------------------
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

    //------------------------------ SysTick ----------------------------------
    SysTick->LOAD = 0x00FFFFFF;
    SysTick->CTRL |= (0b101);

    //------------------------------ Interrupcion boton -----------------------
    RCC->APB2ENR |= (1<<14);
    SYSCFG->EXTICR[3] &= ~(0b1111<<4);
    SYSCFG->EXTICR[3] |= (1<<5);
    EXTI->IMR |= (1<<13);
    EXTI->RTSR |= (1<<13);
    NVIC_EnableIRQ(EXTI15_10_IRQn);

    //------------------------------ UART3 ------------------------------------
    RCC->AHB1ENR |= (1<<3);
    GPIOD->MODER |= (1<<19)|(1<<17);
    GPIOD->AFR[1] |= (0b111<<4)|(0b111<<0);
    RCC->APB1ENR |= (1<<18);
    USART3->BRR = 0x683;
    USART3->CR1 |= ((1<<5)|(0b11<<2));
    NVIC_EnableIRQ(USART3_IRQn);

    //------------------------------ I2C1 (PB8 SCL, PB9 SDA) ------------------
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

    //------------------------------ Timers -----------------------------------
    RCC->APB1ENR |= (1<<1);
    TIM3->PSC = 24;
    TIM3->ARR = 63999;

    RCC->APB1ENR |= (1<<3);
    TIM5->PSC = 24;
    TIM5->ARR = 10000000;

    USART3->CR1 |= (1<<0);
    SysTick_ms(1000);

    //------------------------------ Inicializar las 2 IMU --------------------
    Print(text1, strlen(text1));

    imu[0].addr = MPU_ADDR_1;
    imu[1].addr = MPU_ADDR_2;

    for(uint8_t k = 0; k < N_IMU; k++){
        IMU_Init(k);
    }

    // Deja las IMU en estado seguro: ninguna con bypass activo
    // (Mag_Seleccionar lo activa solo en la que se necesita)
    for(uint8_t k = 0; k < N_IMU; k++){
        Mag_Init(k);
    }

    Print(text_hdr, strlen(text_hdr));

    //------------------------------ Bucle principal --------------------------
    while(1){
        if(flag == 1){
            flag = 0;
            i = 1;

            char msg_test[] = "Boton detectado, entrando a leer...\n\r";
            Print(msg_test, strlen(msg_test));

            while(1){
                TIM5->CNT = 0;
                TIM5->CR1 |= (1<<0);

                // Acelerometro + giroscopio de ambas IMU
                for(uint8_t k = 0; k < N_IMU; k++){
                    IMU_LeerInercial(k);
                }

                delay();

                TIM5->CR1 &= ~(1<<0);
                timer = TIM5->CNT*0.0000000625f;
                cont_timer += timer;

                // Magnetometro de ambas IMU (una a la vez)
                for(uint8_t k = 0; k < N_IMU; k++){
                    Mag_Leer(k);
                }

                sprintf(text,
                    "%d %.4f "
                    "%.0f %.0f %.0f %.0f %.0f %.0f %.2f %.2f %.2f "
                    "%.0f %.0f %.0f %.0f %.0f %.0f %.2f %.2f %.2f \n\r",
                    i++, timer,
                    (float)imu[0].raw_ax, (float)imu[0].raw_ay, (float)imu[0].raw_az,
                    (float)imu[0].raw_gx, (float)imu[0].raw_gy, (float)imu[0].raw_gz,
                    imu[0].mx, imu[0].my, imu[0].mz,
                    (float)imu[1].raw_ax, (float)imu[1].raw_ay, (float)imu[1].raw_az,
                    (float)imu[1].raw_gx, (float)imu[1].raw_gy, (float)imu[1].raw_gz,
                    imu[1].mx, imu[1].my, imu[1].mz);
                Print(text, strlen(text));

                if(i > N_MUESTRAS){
                    cont_timer = 0;
                    break;
                }
            }
        }
    }
}

//----------------------------------------------------------------------------
//                  FUNCIONES DE ALTO NIVEL DE LA IMU
//----------------------------------------------------------------------------
void IMU_Init(uint8_t k){
    char msg[80];

    // Despertar el MPU (sale de sleep)
    cmd[0] = 0x00;
    WriteI2C1(imu[k].addr, MPU_REG_PWR_MGMT_1, cmd, 1);
    SysTick_ms(100);

    // Verificar WHO_AM_I (0x71 = MPU9250, 0x73 = MPU9255, 0x70 = MPU6500)
    data[0] = 0;
    ReadI2C1(imu[k].addr, MPU_REG_WHO_AM_I, data, 1);

    if(data[0] == 0x71 || data[0] == 0x73 || data[0] == 0x70){
        imu[k].mpu_ok = 1;
        sprintf(msg, "IMU %d (0x%02X) OK, WHO_AM_I = 0x%02X \n\r", k+1, imu[k].addr, data[0]);
        Print(msg, strlen(msg));
    } else {
        imu[k].mpu_ok = 0;
        sprintf(msg, "Error: IMU %d (0x%02X) no responde, WHO_AM_I = 0x%02X \n\r", k+1, imu[k].addr, data[0]);
        Print(msg, strlen(msg));
        return;
    }

    // Escalas: giroscopio +-250 dps, acelerometro +-2 g
    cmd[0] = 0x00;
    WriteI2C1(imu[k].addr, MPU_REG_GYRO_CONFIG, cmd, 1);
    WriteI2C1(imu[k].addr, MPU_REG_ACCEL_CONFIG, cmd, 1);

    // Bypass desactivado por defecto
    WriteI2C1(imu[k].addr, MPU_USER_CTRL, cmd, 1);
    cmd[0] = BYPASS_OFF;
    WriteI2C1(imu[k].addr, MPU_INT_PIN_CFG, cmd, 1);
    SysTick_ms(10);
}

void IMU_LeerInercial(uint8_t k){
    if(!imu[k].mpu_ok) return;

    ReadI2C1(imu[k].addr, MPU_REG_ACCEL_XOUT_H, GirAcel, 14);
    imu[k].raw_ax   = (int16_t)(GirAcel[0]<<8  | GirAcel[1]);
    imu[k].raw_ay   = (int16_t)(GirAcel[2]<<8  | GirAcel[3]);
    imu[k].raw_az   = (int16_t)(GirAcel[4]<<8  | GirAcel[5]);
    imu[k].raw_temp = (int16_t)(GirAcel[6]<<8  | GirAcel[7]);
    imu[k].raw_gx   = (int16_t)(GirAcel[8]<<8  | GirAcel[9]);
    imu[k].raw_gy   = (int16_t)(GirAcel[10]<<8 | GirAcel[11]);
    imu[k].raw_gz   = (int16_t)(GirAcel[12]<<8 | GirAcel[13]);
}

//----------------------------------------------------------------------------
//          MAGNETOMETRO AK8963 (compartido en 0x0C -> hay que multiplexar)
//----------------------------------------------------------------------------
// Activa el bypass solo en la IMU 'k' y lo desactiva en las demas.
// Asi solo UN AK8963 es visible en el bus.
void Mag_Seleccionar(uint8_t k){
    for(uint8_t n = 0; n < N_IMU; n++){
        if(!imu[n].mpu_ok) continue;
        cmd[0] = (n == k) ? BYPASS_ON : BYPASS_OFF;
        WriteI2C1(imu[n].addr, MPU_INT_PIN_CFG, cmd, 1);
    }
}

void Mag_Init(uint8_t k){
    char msg[80];

    imu[k].mag_ok = 0;
    imu[k].mx = imu[k].my = imu[k].mz = 0.0f;
    imu[k].asa_x = imu[k].asa_y = imu[k].asa_z = 1.0f;

    if(!imu[k].mpu_ok) return;

    Mag_Seleccionar(k);
    SysTick_ms(10);

    // WHO_AM_I del AK8963
    data[0] = 0;
    ReadI2C1(AK8963_address, AK8963_WIA, data, 1);
    if(data[0] != 0x48){
        sprintf(msg, "Error: AK8963 de IMU %d no encontrado (0x%02X) \n\r", k+1, data[0]);
        Print(msg, strlen(msg));
        return;
    }
    imu[k].mag_ok = 1;
    sprintf(msg, "Magnetometro AK8963 de IMU %d OK! \n\r", k+1);
    Print(msg, strlen(msg));

    // Power down
    cmd[0] = 0x00;
    WriteI2C1(AK8963_address, AK8963_CNTL1, cmd, 1);
    SysTick_ms(10);

    // Fuse ROM access para leer ASA
    cmd[0] = 0x0F;
    WriteI2C1(AK8963_address, AK8963_CNTL1, cmd, 1);
    SysTick_ms(10);

    ReadI2C1(AK8963_address, AK8963_ASAX, asa_raw, 3);
    imu[k].asa_x = ((asa_raw[0] - 128) * 0.5f / 128.0f) + 1.0f;
    imu[k].asa_y = ((asa_raw[1] - 128) * 0.5f / 128.0f) + 1.0f;
    imu[k].asa_z = ((asa_raw[2] - 128) * 0.5f / 128.0f) + 1.0f;

    // Power down
    cmd[0] = 0x00;
    WriteI2C1(AK8963_address, AK8963_CNTL1, cmd, 1);
    SysTick_ms(10);

    // Medicion continua 2 (100 Hz), 16 bits
    cmd[0] = 0x16;
    WriteI2C1(AK8963_address, AK8963_CNTL1, cmd, 1);
    SysTick_ms(10);
}

void Mag_Leer(uint8_t k){
    if(!imu[k].mag_ok) return;

    Mag_Seleccionar(k);

    ReadI2C1(AK8963_address, AK8963_ST1, data, 1);

    if ((data[0] & 0x01) == 0x01){            // DRDY: dato listo
        ReadI2C1(AK8963_address, AK8963_HXL, MagData, 7);  // 6 datos + ST2 (obligatorio)

        if ((MagData[6] & 0x08) == 0){        // sin overflow
            imu[k].raw_mx = (int16_t)(MagData[1]<<8 | MagData[0]);
            imu[k].raw_my = (int16_t)(MagData[3]<<8 | MagData[2]);
            imu[k].raw_mz = (int16_t)(MagData[5]<<8 | MagData[4]);

            imu[k].mx = imu[k].raw_mx * MAG_SENSITIVITY * imu[k].asa_x;
            imu[k].my = imu[k].raw_my * MAG_SENSITIVITY * imu[k].asa_y;
            imu[k].mz = imu[k].raw_mz * MAG_SENSITIVITY * imu[k].asa_z;
        }
    }
}

// ----------------------------------------------------------------------------
//                FUNCIONES I2C CON ESCUDO ANTI-BLOQUEO
// ----------------------------------------------------------------------------
void WriteI2C1(uint8_t Address, uint8_t Register, uint8_t *Data, uint8_t bytes){
    uint8_t n;
    uint32_t timeout;

    I2C1->CR2 &= ~(0x3FF<<0);
    I2C1->CR2 |= (Address<<1);
    I2C1->CR2 &= ~(1<<10);
    I2C1->CR2 &= ~(0xFF<<16);
    I2C1->CR2 |= ((bytes+1)<<16);
    I2C1->CR2 |= (1<<25);                  // AUTOEND
    I2C1->CR2 |= (1<<13);                  // START

    timeout = 100000;
    while (((I2C1->ISR) & (1<<1)) != (0b10)){
        if(--timeout == 0){ I2C1->ICR |= (1<<4)|(1<<5); return; }   // NACKCF, STOPCF
    }

    I2C1->TXDR = Register;

    n = bytes;
    while(n>0){
        timeout = 100000;
        while (((I2C1->ISR) & (1<<1)) != (0b10)){
            if(--timeout == 0){ I2C1->ICR |= (1<<4)|(1<<5); return; }
        }
        I2C1->TXDR = *Data;
        Data++;
        n--;
    }

    timeout = 100000;
    while (((I2C1->ISR) & (1<<5)) != (0b100000)){
        if(--timeout == 0) return;
    }
    I2C1->ICR |= (1<<5);                   // limpia STOPF para la siguiente transaccion
}

void ReadI2C1(uint8_t Address, uint8_t Register, uint8_t *Data, uint8_t bytes ) {
    uint8_t n;
    uint32_t timeout;

    I2C1->CR2 &= ~(0x3FF<<0);
    I2C1->CR2 |= (Address<<1);
    I2C1->CR2 &= ~(1<<10);
    I2C1->CR2 &= ~(0xFF<<16);
    I2C1->CR2 |= (1<<16);
    I2C1->CR2 &= ~(1<<25);
    I2C1->CR2 |= (1<<13);

    timeout = 100000;
    while (((I2C1->ISR) & (1<<1)) != (0b10)){
        if(--timeout == 0){ I2C1->ICR |= (1<<4)|(1<<5); return; }
    }

    I2C1->TXDR = Register;

    timeout = 100000;
    while (((I2C1->ISR) & (1<<6)) != (0b1000000)){
        if(--timeout == 0){ I2C1->ICR |= (1<<4)|(1<<5); return; }
    }

    I2C1->CR2 |= (1<<10);                  // lectura
    I2C1->CR2 &= ~(0xFF<<16);
    I2C1->CR2 |= (bytes<<16);
    I2C1->CR2 &= ~(1<<25);
    I2C1->CR2 |= (1<<13);                  // RESTART

    n = bytes;
    while (n>0){
        timeout = 100000;
        while (((I2C1->ISR) & (1<<2)) != (0b100)){
            if(--timeout == 0){ I2C1->ICR |= (1<<4)|(1<<5); return; }
        }
        *Data = I2C1->RXDR;
        Data++;
        n--;
    }

    I2C1->CR2 |= (1<<14);                  // STOP

    timeout = 100000;
    while (((I2C1->ISR) & (1<<5)) != (0b100000)){
        if(--timeout == 0) return;
    }
    I2C1->ICR |= (1<<5);                   // limpia STOPF
}

//----------------------------------------------------------------------------
//                       UART / DELAY
//----------------------------------------------------------------------------
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