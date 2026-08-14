#include "setup.h"

//MPU6050
#define MPU6050_address 0x68 // Endereço da MPU6500 (giroscópio e acelerômetro)
#define AK8963_address 0x0C
//----------------------------------------------------------------------------
//                        					GLOSARIO
//----------------------------------------------------------------------------
/*
AHB1ENR: RCC AHB1 peripheral clock register
MODER: GPIO port mode register
OTYPER: GPIO port output type register 
OSPEEDR: GPIO port output speed register
PUPDR: GPIO port  rpull-up/pull-down register 
APB2ENR: RCC APB2 peripheral clock enable register 
EXTICR[3]: SYSCFG external interrupt configuration register 4
IMR: Interrupt mask register
RTSR: Rising trigger selection register
AFR[1]: GPIO alternate function high register
APB1ENR: RCC APB1 peripheral clock enable register
BRR: USART baud rate register
*/
void setup(void){
	//----------------------------------------------------------------------------
	// GPIOs
	//----------------------------------------------------------------------------
	RCC->AHB1ENR |= ((1<<1)|(1<<2)); //Habilita puertos B y C 
	GPIOB->MODER &= ~((0b11<<0)|(0b11<<14)); //00: Input mode (reset state) pines PB0 y PB7
	GPIOB->MODER |= ((1<<0)|(1<<14)); // 01: General purpose output mode pines PB0 y PB7
	GPIOC->MODER &= ~(0b11<<26); //00: Input mode (reset state) pines PC13
	GPIOB->OTYPER &= ~((1<<0)|(1<<7)); //0: Output push-pull (reset state) pines PB0 y PB7
	GPIOB->OSPEEDR |= (((1<<1)|(1<<0)|(1<<15)|(1<<14))); //11: Very high speed pines PB0 y PB7
	GPIOC->OSPEEDR |= ((1<<27)|(1<<26)); //11: Very high speed pines PC13
	GPIOB->PUPDR &= ~((0b11<<0)|(0b11<<14)); //00: No pull-up, pull-down pines PB0 y PB7
	GPIOC->PUPDR &= ~(0b11<<26); //00: No pull-up, pull-down pin PC13
	GPIOC->PUPDR |= (1<<27); //10: Pull-down pin PC13
	
	//----------------------------------------------------------------------------
	// Interrupt
	//----------------------------------------------------------------------------
	RCC->APB2ENR |= (1<<14); //1: System configuration controller clock enabled
	SYSCFG->EXTICR[3] &= ~(0b1111<<4); //0000: Limpia bits
	SYSCFG->EXTICR[3] |= (1<<5); //0010: PC13 pin as source input for the EXTIx external interrupt.
	EXTI->IMR |= (1<<13); //1: Interrupt request from line 13 is not masked
	EXTI->RTSR |= (1<<13); //1: Rising trigger enabled (for Event and Interrupt) for input line 13
	NVIC_EnableIRQ(EXTI15_10_IRQn); 
			
	//----------------------------------------------------------------------------
	//                        					UART
	//----------------------------------------------------------------------------
	RCC->AHB1ENR |= (1<<3); //1: IO port D clock enabled
	GPIOD->MODER |= (1<<19)|(1<<17); //10: Alternate function mode pines PD8 y PD9
	GPIOD->AFR[1] |= (0b111<<4)|(0b111<<0); //0111: AF7 pines PD8 y PD9
	RCC->APB1ENR |= (1<<18); //1: USART3 clock enabled
	USART3->BRR = 0x683; // USARTDIV
	USART3->CR1 |= ((1<<5)|(0b11<<2)); 
	NVIC_EnableIRQ(USART3_IRQn);

	//----------------------------------------------------------------------------
	//                        					I2C
	//----------------------------------------------------------------------------
	RCC->AHB1ENR |= (1<<1); //Enable GPIOB clock (PB9=I2C1_SDA and PB8=I2C1_SCL)
	GPIOB->MODER |= (1<<19)|(1<<17); //Set (10) pins PB9 (bits 19:18) and PB8 (bits 17:16) as alternant function
	GPIOB->OTYPER |= (1<<9)|(1<<8); //Set (1) pin PB9 (bit 9) and pin PB8 (bit 8) as output open drain (HIGH or LOW)
	GPIOB->OSPEEDR |= (0b11<<18)|(0b11<<16); //Set (11) pin PB9 (bits 19:18) and pin PB8 (bits 17:16) as Very High Speed
	GPIOB->PUPDR|= (1<<18)|(1<<16); //Set (01) pin PB9 (bits 19:18) and pin PB8 (bits 17:16) as pull up
	GPIOB->AFR[1] |= (1<<6)|(1<<2); //Set the I2C1 (AF4) alternant function for pins PB9=I2C1_SDA (bits 7:4) and PB8=I2C1_SCL (bits 3:0)
	RCC->APB1ENR |= (1<<21); //Enable I2C1 clock
	RCC->DCKCFGR2 |= (1<<17); //Set (10) bits 17:16 as HSI clock is selected as source I2C1 clock
	I2C1->CR1 &= ~(1<<0);// Clear the enable I2C1
	I2C1->TIMINGR |= 0x30420F13;// Table 207 of reference manual
	I2C1->CR1 |= (1<<0);// Enable I2C1
	
	//----------------------------------------------------------------------------
	//                        					TIMER
	//----------------------------------------------------------------------------
	//TIMER
	RCC->APB1ENR |= (1<<1); //Enable the TIMER3 clock 
	TIM3->PSC = 24; // Prescale factor 25 for 100ms of time
	TIM3->ARR = 63999; // Maximum count value
	
RCC->APB1ENR |= (1<<3); //Enable the TIMER5 clock 
	TIM5->PSC = 24; // Prescale factor 25 for 100ms of time
	TIM5->ARR = 10000000; // Maximum count value
	

	USART3->CR1 |= (1<<0);
	
//	SysTick_ms(1000);


}