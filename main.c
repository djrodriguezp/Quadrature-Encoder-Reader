#include "MK64F12.h"
#include<stdio.h>
#include<stdlib.h>
#include<string.h>

volatile uint32_t ticks = 0;
volatile uint32_t oldticks = 0;
volatile float linearVelocity = 0;
volatile float rpm =0;

const float wheelRadius = 0.021f; //metros
const float encoderResolution = 456.0f; //cantidad de ticks generados en 1 vuelta de la llanta
const float interruptPeriod = 10.0f; //interrupciones periodicas generadas cada 10ms

uint8_t uart_getchar ();
void put(char *ptr_str);
void uart_putchar (char ch);


void PORTC_IRQHandler(void)
{
	PORTC_ISFR |= (6<<1);
	if (FTM0_C1V < 200)
	{
		FTM0_C1V += 50;
	}
	else
	{
		FTM0_C1V = 0;
	}
}

void PORTA_IRQHandler(void)
{
	PORTA_ISFR |= (2<<1);
	ticks++;
}

void SysTick_Handler (void) {
	rpm = ((ticks - oldticks) * (60000.f / interruptPeriod)) / encoderResolution; //60000 milisegundos en 1 minuto para calcular rpm
	linearVelocity =  0.10472 * wheelRadius * rpm ; //(2PI/60) * radio * rpm
	oldticks = ticks;
}

int main()
{
	//inicializacion de relojes de peurtos utilizados
	SIM_SCGC4  |= SIM_SCGC4_UART0_MASK;   	
	SIM_SCGC6  |= SIM_SCGC6_FTM0_MASK;	
	SIM->SCGC5 |= SIM_SCGC5_PORTA_MASK;
	SIM_SCGC5  |= SIM_SCGC5_PORTB_MASK;	
	SIM->SCGC5 |= SIM_SCGC5_PORTC_MASK; 
	SIM_SCGC5  |= SIM_SCGC5_PORTD_MASK;
	
	//pin utilizado para interrupciones del encoder
	PORTA->PCR[2] = 0xB0100;
	
	//pin para PWM que controla la velocidad del motor
	PORTC->PCR[2]   = PORT_PCR_MUX(4);
	
  //pines para controlar direccion del motor	
	PORTC->PCR[3]   = 0x100;
	PORTC->PCR[4]   = 0x100;
	//pin para el SW2
	PORTC->PCR[6]   =  0x90100;
	
	//pines utilizados por UART
	PORTB->PCR[16] |= PORT_PCR_MUX(3);
	PORTB->PCR[17] |= PORT_PCR_MUX(3);

	PTA->PDDR |= (0 << 2);
	PTC->PDDR |= (1 << 3);
	PTC->PDDR |= (1 << 4);
	PTC->PDDR |= (0 << 6);
	
	//habilitamos interrupciones para encoder y para el switch que controla PWM
	PORTA_ISFR |= (2<<1);
	NVIC_EnableIRQ(PORTA_IRQn);
	
	PORTC_ISFR |= (6<<1);
	NVIC_EnableIRQ(PORTC_IRQn);
	
	SystemCoreClockUpdate ();  
	SysTick_Config(SystemCoreClock/100); //interrupcion periodica cada 10ms
	
	//configuracion para PWM en puerto c pin 2
	FTM0_SC |= 0x004F;			
	FTM0_MOD = 255;			
	FTM0_C1SC |= 0x0028;			
	FTM0_C1V = 0;					

	//pines para dirección de motor
	PTC->PSOR = (1UL << 3);
	PTC->PCOR = (1UL << 4);
	
	uint16_t ubd;					/*Variable to save the baud rate*/
	uint8_t temp;




	//configuracion de UART para comunicacion serial
	UART0_C2 &= ~(UART_C2_TE_MASK | UART_C2_RE_MASK ); 
	UART0_C1 = 0; 		
	ubd = (uint16_t)((21000*1000)/(9600 * 16));  
	temp = UART0_BDH & ~(UART_BDH_SBR(0x1F)); 
	UART0_BDH = temp | (((ubd & 0x1F00) >> 8));
	UART0_BDL = (uint8_t)(ubd & UART_BDL_SBR_MASK);
	UART0_C2 |= (UART_C2_TE_MASK | UART_C2_RE_MASK );  

	uint8_t ch;
	

	
	while(1){
		char buf[50];
		for(int i; i < 1000000;i++);
		snprintf(buf, 50, "RPM:%.4f, velocidad lineal: %.4f m/s\r\n\n", rpm, linearVelocity);
		put(buf);
	}
}


void uart_putchar (char ch)
{
while(!(UART0_S1 & UART_S1_TDRE_MASK));
UART0_D = (uint8_t)ch;
}

void put(char *ptr_str)
{
	while(*ptr_str)
		uart_putchar(*ptr_str++);
}
