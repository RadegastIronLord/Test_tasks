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
void RTC_init(void);
void RTC_set_time(uint8_t ht, uint8_t hu, uint8_t mt, uint8_t mu, uint8_t st, uint8_t su);
void RTC_set_alarmA(uint8_t ht, uint8_t hu, uint8_t mt, uint8_t mu, uint8_t st, uint8_t su);
void RTC_set_alarmB(uint8_t ht, uint8_t hu, uint8_t mt, uint8_t mu, uint8_t st, uint8_t su);
void RTC_get_time(void);

void RTC_WKUP_IRQHandler(void);
void RTC_Alarm_IRQHandler(void);

static uint8_t buf[32];
static uint8_t time[8];
volatile static size_t cnt = 0;
volatile static bool transfer_completed = false;

static const char* message_help = "Available commands\nstHHMMSS - set time HH:MM:SS\nsaHHMMSS - set time LEDON HH:MM:SS\nsbHHMMSS - set time LEDOFF HH:MM:SS\ngt - get current time\n";
static const char* message_cmd = "Command ";
static const char* message_ACK = " received\n";
static const char* message_err = "Command not recognized!\n";
static const char* message_time = "Current time:\n";

int main() {
	__enable_irq ();
	Clock_init();
	UART2_init();
	RTC_init();
	UART_TX_str(message_help, 0xff);
	GPIOA->MODER |= 0x1 << GPIO_MODER_MODE5_Pos;
	while(1){
		if(transfer_completed) {
			transfer_completed = false;
			CMD_Handler();
		}
	}
}

void RTC_init(){
	RCC->APB1ENR |= RCC_APB1ENR_PWREN;
	PWR->CR |= PWR_CR_DBP;
	RCC->BDCR |= RCC_BDCR_LSEON;  
	while (!(RCC->BDCR & RCC_BDCR_LSEON)){} 
	RCC->BDCR |= 0x1 << RCC_BDCR_RTCSEL_Pos;
	RCC->BDCR |= RCC_BDCR_RTCEN;
	RTC_set_time(1,2,0,0,0,0);
	RTC_get_time();
	
	RTC->WPR = 0xCA;
	RTC->WPR = 0x53;
	RTC->ISR |= RTC_ISR_INIT;
	while( !(RTC->ISR & RTC_ISR_INITF) ){} 		
		
	// configure exti22 for wkup
	EXTI->IMR |= EXTI_IMR_IM22;
	EXTI->RTSR |= EXTI_RTSR_TR22;
	// configure exti17 for alarm
	EXTI->IMR |= EXTI_IMR_IM17;
	EXTI->RTSR |= EXTI_RTSR_TR17;	
	// NVIC 
	NVIC_EnableIRQ(RTC_WKUP_IRQn);
	NVIC_EnableIRQ(RTC_Alarm_IRQn);
		
	// Setup WKUP
	RTC->CR &= ~ RTC_CR_WUTE;
	while(!(RTC->ISR & RTC_ISR_WUTWF));
	RTC->CR |= 0x4 << RTC_CR_WUCKSEL_Pos;
	RTC->WUTR = 0;
	RTC->CR |= RTC_CR_WUTE;         
  RTC->CR |= RTC_CR_WUTIE;
		
	// Setup alarm
	RTC->CR &= ~ RTC_CR_ALRAE;
	while(!(RTC->ISR & RTC_ISR_ALRAWF));
	RTC->CR |= RTC_CR_ALRAIE;	
	RTC->CR |= RTC_CR_ALRAE;
	RTC->CR &= ~ RTC_CR_ALRBE;
	while(!(RTC->ISR & RTC_ISR_ALRBWF));
	RTC->CR |= RTC_CR_ALRBIE;	
	RTC->CR |= RTC_CR_ALRBE;
	
	RTC->ISR &= ~(RTC_ISR_INIT);
	RTC->WPR = 0xFF;	
}

void RTC_WKUP_IRQHandler() {
	RTC->ISR &= ~RTC_ISR_WUTF;
	EXTI->PR |= 0x1 << EXTI_PR_PR22_Pos;
	RTC_get_time();
	UART_TX_str(message_time, 0xff);
	UART_TX_str(time, 8);
	UART_TX_byte('\n');
}

void RTC_Alarm_IRQHandler() {
	if(RTC->ISR & RTC_ISR_ALRAF) {
		RTC->ISR &= ~RTC_ISR_ALRAF;
		GPIOA->BSRR |= 0x20;
		EXTI->PR |= 0x1 << EXTI_PR_PR17_Pos;
	}
	else{
		RTC->ISR &= ~RTC_ISR_ALRBF;
		GPIOA->BSRR |= 0x20 << 16;
		EXTI->PR |= 0x1 << EXTI_PR_PR17_Pos;
	}
}

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
	for(size_t i = 0; i < num && data[i] != '\0'; ++i) {
		UART_TX_byte(data[i]);
	}
}

void Dummy_delay(uint16_t t) {
	for(size_t i = 0; i < t; ++i)
		__ASM("NOP");
}


void CMD_Handler() {
	/*UART_TX_str(message_cmd, 0xff);
	UART_TX_str(buf, cnt);
	UART_TX_str(message_ACK, 0xff);*/
	if(cnt != 8) {
		if(buf[0] == 'h' && buf[1] == 'e' && buf[2] == 'l' &&buf[3] == 'p') 
			UART_TX_str(message_help, 0xff);
		else if (buf[0] == 'g' && buf[1] == 't'){
			RTC_get_time();
			UART_TX_str(message_time, 0xff);
			UART_TX_str(time, 8);
			UART_TX_byte('\n');
		}
		else
			UART_TX_str(message_err, 0xff);
	}
	else if(buf[0] != 's'){
		UART_TX_str(message_err, 0xff);
	}
	else {
		switch(buf[1]){
			case 't': //st - set time
				RTC_set_time(buf[2], buf[3], buf[4], buf[5], buf[6], buf[7]);
				break;
			case 'a': //sa - set alarm a: turn on
				RTC_set_alarmA(buf[2], buf[3], buf[4], buf[5], buf[6], buf[7]);
				break;
			case 'b': //sb - set alarm b: turn off
				RTC_set_alarmB(buf[2], buf[3], buf[4], buf[5], buf[6], buf[7]);
				break;
			default:
				UART_TX_str(message_err, 0xff);
		}
	}
	cnt = 0;
}

void RTC_set_time(uint8_t ht, uint8_t hu, uint8_t mt, uint8_t mu, uint8_t st, uint8_t su){
	RTC->WPR = 0xCA;
	RTC->WPR = 0x53;
	RTC->ISR |= RTC_ISR_INIT;
	while( !(RTC->ISR & RTC_ISR_INITF) ){} 
		
	RTC->TR = (ht & 0xF) << RTC_TR_HT_Pos
					| (hu & 0xF) << RTC_TR_HU_Pos
					| (mt & 0xF) << RTC_TR_MNT_Pos
					| (mu & 0xF) << RTC_TR_MNU_Pos
					| (st & 0xF) << RTC_TR_ST_Pos
					| (su & 0xF) << RTC_TR_SU_Pos;

	RTC->ISR &= ~(RTC_ISR_INIT);
	RTC->WPR = 0xFF;
}

void RTC_set_alarmA(uint8_t ht, uint8_t hu, uint8_t mt, uint8_t mu, uint8_t st, uint8_t su){
	RTC->WPR = 0xCA;
	RTC->WPR = 0x53;
	RTC->ISR |= RTC_ISR_INIT;
	while( !(RTC->ISR & RTC_ISR_INITF) ){} 
	RTC->CR &= ~RTC_CR_ALRAE;
	while( !(RTC->ISR & RTC_ISR_ALRAWF) ){}
		
	RTC->ALRMAR = RTC_ALRMAR_MSK4
							| (ht & 0xF) << RTC_ALRMAR_HT_Pos
							| (hu & 0xF) << RTC_ALRMAR_HU_Pos
							| (mt & 0xF) << RTC_ALRMAR_MNT_Pos
							| (mu & 0xF) << RTC_ALRMAR_MNU_Pos
							| (st & 0xF) << RTC_ALRMAR_ST_Pos
							| (su & 0xF) << RTC_ALRMAR_SU_Pos;
	RTC->CR |= RTC_CR_ALRAE;
	RTC->ISR &= ~(RTC_ISR_INIT);
	RTC->WPR = 0xFF;
}
	
void RTC_set_alarmB(uint8_t ht, uint8_t hu, uint8_t mt, uint8_t mu, uint8_t st, uint8_t su){
	RTC->WPR = 0xCA;
	RTC->WPR = 0x53;
	RTC->ISR |= RTC_ISR_INIT;
	while( !(RTC->ISR & RTC_ISR_INITF) ){}
	RTC->CR &= ~RTC_CR_ALRBE;
	while( !(RTC->ISR & RTC_ISR_ALRBWF) ){}		
		
	RTC->ALRMBR = RTC_ALRMBR_MSK4
							| (ht & 0xF) << RTC_ALRMBR_HT_Pos
							| (hu & 0xF) << RTC_ALRMBR_HU_Pos
							| (mt & 0xF) << RTC_ALRMBR_MNT_Pos
							| (mu & 0xF) << RTC_ALRMBR_MNU_Pos
							| (st & 0xF) << RTC_ALRMBR_ST_Pos
							| (su & 0xF) << RTC_ALRMBR_SU_Pos;
	RTC->CR |= RTC_CR_ALRBE;
	RTC->ISR &= ~(RTC_ISR_INIT);
	RTC->WPR = 0xFF;
}

void RTC_get_time(void){
	time[0] = 0x30 | ((RTC->TR & RTC_TR_HT_Msk) >> RTC_TR_HT_Pos);
	time[1] = 0x30 | ((RTC->TR & RTC_TR_HU_Msk) >> RTC_TR_HU_Pos);
	time[3] = 0x30 | ((RTC->TR & RTC_TR_MNT_Msk) >> RTC_TR_MNT_Pos);
	time[4] = 0x30 | ((RTC->TR & RTC_TR_MNU_Msk) >> RTC_TR_MNU_Pos);
	time[6] = 0x30 | ((RTC->TR & RTC_TR_ST_Msk) >> RTC_TR_ST_Pos);
	time[7] = 0x30 | ((RTC->TR & RTC_TR_SU_Msk) >> RTC_TR_SU_Pos);
	time[2] = ':';
	time[5] = ':';
}

