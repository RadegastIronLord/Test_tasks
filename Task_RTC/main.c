#include "stm32f411xe.h"
#include <stdlib.h> 
#include <stdbool.h>
#include <math.h>

void Clock_init(void);
void Dummy_delay(uint16_t);
void UART2_init(void);
void USART2_IRQHandler(void);
void UART_TX_byte(uint8_t);
void UART_TX_str(uint8_t *, size_t);
void CMD_Handler(void);


static uint8_t buf[32];
volatile static size_t cnt = 0;

void Clock_init(void) {
	RCC->PLLCFGR &= ~RCC_PLLCFGR_PLLSRC; // set PLLsrc as HSI 16 MHz
	RCC->PLLCFGR |= 16 << RCC_PLLCFGR_PLLM_Pos; // set M as /16
	RCC->PLLCFGR |= 192 << RCC_PLLCFGR_PLLN_Pos; // set N as *192
	RCC->PLLCFGR |= 1 << RCC_PLLCFGR_PLLP_Pos; // set P as /4
	//PLL freq is set at 48 MHz
	RCC->CFGR |= 0 << RCC_CFGR_PPRE1_Pos; // set APB1 mult as /2
	// APB1 freq = 24 MHz
	RCC->CFGR |= 0 << RCC_CFGR_PPRE2_Pos; // set APB2 mult as /1
	// APB2 freq = 48 MHz
	RCC->CFGR |= 8 << RCC_CFGR_HPRE_Pos; // set AHB mult as /1
	// AHB freq = 48 MHz
	
	RCC->CR |= RCC_CR_PLLON; // turn on PLL
	while(!(RCC->CR & RCC_CR_PLLRDY)); //wait for PLL enebled
	
	RCC->CFGR |= 2 << RCC_CFGR_SW_Pos; // set PLL as SysClk src
	while((RCC->CFGR & RCC_CFGR_SWS_Msk) != (2 << RCC_CFGR_SWS_Pos)); //wait for PLL as SysClk src
}

void UART2_init(void) {
	// RX - PA3
	// TX - PA2
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
	GPIOA->MODER |= 0x2 << GPIO_MODER_MODER2_Pos // set AF output
								| 0x2 << GPIO_MODER_MODER3_Pos; // set input
	//GPIOA->PUPDR |= 0x1 << GPIO_PUPDR_PUPD3_Pos; // RX input pull-up
	GPIOA->AFR[0] |= 0x7 << GPIO_AFRL_AFSEL2_Pos
								 | 0x7 << GPIO_AFRL_AFSEL3_Pos; // set AF7 (USART2)
	
	RCC->APB1ENR |= RCC_APB1ENR_USART2EN;
	USART2->BRR |= 0x4EF; //  19200
	USART2->CR1 |= 0x0 << USART_CR1_M_Pos // 1 start bit, 8 data bits n stop bits 
							| USART_CR1_RE // RX en
							| USART_CR1_TE
							| USART_CR1_RXNEIE; // Recieved data ready to be read interrupt en
	USART2->CR2 |= 0x0 << USART_CR2_STOP_Pos; // 1 stop bit
	USART2->CR1 |= USART_CR1_UE; // enable USART
	NVIC_EnableIRQ(USART2_IRQn);
}


volatile static bool transfer_completed = false;
void USART2_IRQHandler() {
	uint8_t c;
	if(USART2->SR & USART_SR_RXNE_Msk){
		USART2->SR &= ~USART_SR_RXNE;
		c = USART2->DR;
		if(c != 0x0A && c != 0x0D) {
			buf[cnt] = c;
			cnt++;
		}
		if(c == 0xA){
			transfer_completed = true;
		}
	}
	USART2->SR &= 0;
}

void UART_TX_byte(uint8_t byte) {
	while(!(USART2->SR & USART_SR_TXE_Msk)){}
	USART2->DR = byte;
	while(!(USART2->SR & USART_SR_TC_Msk)){}
}

void UART_TX_str(uint8_t *data, size_t num) {
	for(size_t i = 0; i < num; ++i) {
		UART_TX_byte(data[i]);
	}
}

void Dummy_delay(uint16_t t) {
	for(size_t i = 0; i < t; ++i)
		__ASM("NOP");
}


void CMD_Handler() {
	UART_TX_str(buf, cnt);
	
	cnt = 0;
}


int main() {
	__enable_irq ();
	Clock_init();
	UART2_init();
	GPIOA->MODER |= 0x1 << GPIO_MODER_MODE5_Pos;
	while(1){
		if(transfer_completed) {
			transfer_completed = false;
			CMD_Handler();
		}
	}
}