#include "stm32f10x.h"
#include <math.h>

unsigned char symbol_USART2 = 0;
unsigned char symbol_USART3 = 0;

#define BUFFER_SIZE 10240
unsigned char symbol_buffer[ BUFFER_SIZE ];
uint16_t      buffer_addr_USART2;
uint16_t      buffer_addr_USART3;

void USART2_IRQHandler(void);
void USART3_IRQHandler(void);

#define BAUD_RATE_USART2 115200
#define BAUD_RATE_USART3 55555


volatile uint32_t msTicks;                                 // counts 1ms timeTicks
/*----------------------------------------------------------------------------
 * SysTick_Handler:
 *----------------------------------------------------------------------------*/
void SysTick_Handler( void )
{
  msTicks++;
}


void Delay ( uint32_t dlyTicks )
{
  uint32_t curTicks;

  curTicks = msTicks;
  while ( ( msTicks - curTicks ) < dlyTicks ) { __NOP(); }
}

int16_t double_to_int( double divider )
{
		return ( ( uint16_t )( ceil( divider ) ) << 4 ) | ( uint16_t )( round( ( divider - ceil( divider ) ) * 16 ) );
}

void SystemCoreClockConfigure( void )
{
		RCC->CR |= ( ( uint32_t )RCC_CR_HSEON );                    
		while ( ( RCC->CR & RCC_CR_HSERDY ) == 0 );                  

		RCC->CFGR = RCC_CFGR_SW_HSE;                             
		while ( ( RCC->CFGR & RCC_CFGR_SWS ) != RCC_CFGR_SWS_HSE );  
	
	  RCC->CFGR  = RCC_CFGR_HPRE_DIV1;                         // HCLK = SYSCLK
		RCC->CFGR |= RCC_CFGR_PPRE1_DIV1;                        // APB1 = HCLK/1
		RCC->CFGR |= RCC_CFGR_PPRE2_DIV1;                        // APB2 = HCLK/1

		RCC->CR &= ~RCC_CR_PLLON;                                

		//  PLL configuration:  = HSE * 9 = 72 MHz
		RCC->CFGR &= ~( RCC_CFGR_PLLSRC     | RCC_CFGR_PLLMULL );
		RCC->CFGR |=  ( RCC_CFGR_PLLSRC_HSE | RCC_CFGR_PLLMULL9 );

		RCC->CR |= RCC_CR_PLLON;                                
		while( ( RCC->CR & RCC_CR_PLLRDY ) == 0 ) __NOP();           

		RCC->CFGR &= ~RCC_CFGR_SW;                               
		RCC->CFGR |=  RCC_CFGR_SW_PLL;
		while ( ( RCC->CFGR & RCC_CFGR_SWS ) != RCC_CFGR_SWS_PLL );  
}

void USART_Init ()
{
	// USART2
	RCC->APB1ENR |= RCC_APB1ENR_USART2EN;
	RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;
	
	// CNF_A2 = 10, MODE_A2 = 01
	GPIOA->CRL &= ~( GPIO_CRL_CNF2   | GPIO_CRL_MODE2 );
	GPIOA->CRL |=  ( GPIO_CRL_CNF2_1 | GPIO_CRL_MODE2 );

	// CNF_A3 = 10, MODE_A3 = 00, ODR_A3 = 1
	GPIOA->CRL  &= ~( GPIO_CRL_CNF3 | GPIO_CRL_MODE3 );
	GPIOA->CRL  |=    GPIO_CRL_CNF3_1;
	GPIOB->BSRR |=    GPIO_BSRR_BR3;
	
	
	// USART3
	RCC->APB1ENR |= RCC_APB1ENR_USART3EN;
	RCC->APB2ENR |= RCC_APB2ENR_IOPBEN;
	
	// CNF_B10 = 10, MODE_B10	 = 01
	GPIOB->CRH &= ~( GPIO_CRH_CNF10   | GPIO_CRH_MODE10 );
	GPIOB->CRH |=  ( GPIO_CRH_CNF10_1 | GPIO_CRH_MODE10 );

	// CNF_B11 = 10, MODE_B11 = 00, ODR_B11 = 1
	GPIOA->CRH  &= ~( GPIO_CRH_CNF11 | GPIO_CRH_MODE11 );
	GPIOA->CRH  |=    GPIO_CRH_CNF11_1;
	GPIOB->BSRR |=    GPIO_BSRR_BR11;
	

	// USARTDIV = SystemCoreClock / (16 * BAUD) = 72000000 / (16 * 9600) = 468,75

	double divider_USART2 = SystemCoreClock / ( 16 * BAUD_RATE_USART2 );

	USART2->BRR  = double_to_int( divider_USART2 );

	USART2->CR1 |= USART_CR1_TE | USART_CR1_RE | USART_CR1_UE | USART_CR1_RXNEIE;
	NVIC_EnableIRQ ( USART2_IRQn );
	USART2->CR2 = 0;
	USART2->CR3 = 0;
	
	
	double divider_USART3 = SystemCoreClock / ( 16 * BAUD_RATE_USART3 );

	USART3->BRR  = double_to_int( divider_USART3 );
	
	USART3->CR1 |= USART_CR1_TE | USART_CR1_RE | USART_CR1_UE | USART_CR1_RXNEIE;
	NVIC_EnableIRQ ( USART3_IRQn );
	USART3->CR2 = 0;
	USART3->CR3 = 0;
}

void USART3_Send ( unsigned char data )
{
		while ( ( USART3->SR & USART_SR_TXE ) == 0 ) {}
		USART3->DR = data;
}

void USART2_IRQHandler( void )
{
	if ( buffer_addr_USART2 >= BUFFER_SIZE )
		buffer_addr_USART2 = 0;
		
	symbol_buffer[ buffer_addr_USART2++ ] = ( unsigned char )( USART2->DR );
}

void USART3_IRQHandler( void )
{
	symbol_USART2 = ( unsigned char )( USART3->DR );
	USART2->DR = symbol_USART2;
}

int main()
{
	SystemCoreClockConfigure();
	SystemCoreClockUpdate();
	SysTick_Config( SystemCoreClock / 1000000 );                  // SysTick 1 msec interrupts
	
	buffer_addr_USART2 = 0;
	
	USART_Init();
	
	while ( 1 )
	{
		while ( buffer_addr_USART2 != buffer_addr_USART3 )
		{
			if ( buffer_addr_USART3 >= BUFFER_SIZE )
				buffer_addr_USART3 = 0;
			
			USART3_Send( symbol_buffer[ buffer_addr_USART3++ ] );
		}
	}
	
	return 0;
}