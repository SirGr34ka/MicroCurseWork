#include <math.h>
#include <stdio.h>

#include "stm32f10x.h"
#include "ds18b20.h"


unsigned char symbol_USART3      = 0;

uint8_t       start_command_flag = 0; 
unsigned char command            = 0;
uint8_t       var_size           = 0;
uint8_t       var_cnt            = 0;
unsigned char var_in_char[ 3 ];
uint8_t       end_command_flag   = 0;

void USART3_IRQHandler(void);

#define BAUD_RATE_USART3 55555


#define MAX_SENSORS 2

uint8_t       devCount = 0;
Sensor sensors[ MAX_SENSORS ];									// array of structures to store sensors data
uint8_t resolution = RESOLUTION_12BIT;

unsigned char symbol = 0;
uint8_t command_var = 0;
uint8_t num_pow = 0;
int8_t tempCheck = 0;


//--------------------------------------------------------------------------------------------------------------------------------

void Init_Sensors ()
{													// reset all values in sensors
	for ( uint8_t i = 0 ; i < MAX_SENSORS ; ++i )
	{
		sensors[ i ].raw_temp        = 0x0;
		sensors[ i ].temp            = 0.0;
		sensors[ i ].crc8_rom        = 0x0;
		sensors[ i ].crc8_data       = 0x0;
		sensors[ i ].crc8_rom_error  = 0x0;
		sensors[ i ].crc8_data_error = 0x0;
	
		for ( uint8_t j = 0 ; j < 8 ; ++j )
			sensors[ i ].ROM_code[ j ] = 0x00;
	
		for ( uint8_t j = 0 ; j < 9 ; ++j )
			sensors[ i ].scratchpad_data[ j ] = 0x00;
	}
}

int checkDelay ( uint8_t resolution )
{
	switch ( resolution )
	{
		case RESOLUTION_9BIT  : return  93750;
		case RESOLUTION_10BIT : return 187500;
		case RESOLUTION_11BIT : return 375000;
		case RESOLUTION_12BIT : return 750000;
		default               : return 750000;
	}
}

void waitWhileRightCRC ( Sensor* sensor )
{
	do
	{
		ds18b20_ReadStratchpad( 1 , sensor->scratchpad_data , sensor->ROM_code );
		
		sensor->crc8_data       = Compute_CRC8( sensor->scratchpad_data , 8 );																	// 	Compute CRC for data
		sensor->crc8_data_error = Compute_CRC8( sensor->scratchpad_data , 9 ) == 0 ? 0 : 1;														// 	Get CRC Error Signal
	}
	while ( sensor->crc8_data_error );
	
	return;
}

void writeRes ( uint8_t dec_resolution )
{
	switch ( dec_resolution )
	{
		case  9 : resolution = RESOLUTION_9BIT; break;
		case 10 : resolution = RESOLUTION_10BIT; break;
		case 11 : resolution = RESOLUTION_11BIT; break;
		case 12 : resolution = RESOLUTION_12BIT; break;
		default : return;
	}
	
	for ( uint8_t i = 0 ; i < devCount ; ++i )
	{
		ds18b20_MatchRom( sensors[ i ].ROM_code );
		
		ds18b20_WriteByte( WRITE_SCRATCHPAD );
		ds18b20_WriteByte( sensors[ i ].scratchpad_data[ 2 ] );
		ds18b20_WriteByte( sensors[ i ].scratchpad_data[ 3 ] );
		ds18b20_WriteByte( resolution );
		
		waitWhileRightCRC( &( sensors[ i ] ) );
	}
	
	return;
}

void writeTempHigh ( uint8_t Th )
{
	tempCheck = ~( Th & ( 0 << 7 ) ) + 1;
	
	if ( ( tempCheck > 125 ) | ( tempCheck < -55 ) )
		return;
	
	for ( uint8_t i = 0 ; i < devCount ; ++i )
	{
		ds18b20_MatchRom( sensors[ i ].ROM_code );
		
		ds18b20_WriteByte( WRITE_SCRATCHPAD );
		ds18b20_WriteByte( Th );
		ds18b20_WriteByte( sensors[ i ].scratchpad_data[ 3 ] );
		ds18b20_WriteByte( sensors[ i ].scratchpad_data[ 4 ] );
		
		waitWhileRightCRC( &( sensors[ i ] ) );
	}
	
	return;
}

void writeTempLow ( uint8_t Tl )
{
	tempCheck = ~( Tl & 0x7F ) + 1;
	
	if ( ( tempCheck > 125 ) | ( tempCheck < -55 ) )
		return;
	
	for ( uint8_t i = 0 ; i < devCount ; ++i )
	{
		ds18b20_MatchRom( sensors[ i ].ROM_code );
		
		ds18b20_WriteByte( WRITE_SCRATCHPAD );
		ds18b20_WriteByte( sensors[ i ].scratchpad_data[ 2 ] );
		ds18b20_WriteByte( Tl );
		ds18b20_WriteByte( sensors[ i ].scratchpad_data[ 4 ] );
		
		waitWhileRightCRC( &( sensors[ i ] ) );
	}
	
	return;
}


//--------------------------------------------------------------------------------------------------------------------------------

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


	double divider_USART3 = SystemCoreClock / ( 16 * BAUD_RATE_USART3 );

	USART3->BRR  = double_to_int( divider_USART3 );

	USART3->CR1 |= USART_CR1_TE | USART_CR1_RE | USART_CR1_UE | USART_CR1_RXNEIE;
//	USART3->CR1 |= USART_CR1_TE | USART_CR1_UE ;
	
	NVIC_EnableIRQ ( USART3_IRQn );
	USART3->CR2 = 0;
	USART3->CR3 = 0;
}

void USART3_Send ( unsigned char symbol )
{
		while ( ( USART3->SR & USART_SR_TXE ) == 0 ) {}
		USART3->DR = symbol;
}

void USART3_IRQHandler( void )
{
	symbol_USART3 = ( unsigned char )( USART3->DR );
	
	if ( symbol_USART3 == ':' )
	{
		command            = 0;
		var_size           = 0;
		var_cnt            = 0;
		start_command_flag = 1;
		end_command_flag   = 0;
	}
	else if ( start_command_flag )
	{
		start_command_flag = 0;
		
		switch ( symbol_USART3 )
		{
			case 'R' : var_size = 2; command = 'R'; break;
			case 'l' : var_size = 3; command = 'l'; break;
			case 'h' : var_size = 3; command = 'h'; break;
			default  : var_size = 0; command = 0;
		}
	}
	else if ( ( symbol_USART3 == ';' ) && ( command == 0 ) )
	{
		command  = 0;
		var_size = 0;
	}
	else if ( ( symbol_USART3 == ';' ) && ( var_cnt > 0 ) )
	{
		end_command_flag = 1;
	}
	else if ( var_cnt < var_size )
	{
		var_in_char[ var_cnt++ ] = symbol_USART3;
	}
}

//--------------------------------------------------------------------------------------------------------------------------------

int main () {
	SystemCoreClockConfigure();
	SystemCoreClockUpdate();
	SysTick_Config( SystemCoreClock / 1000000 );                  // SysTick 1 usec interrupts

	USART_Init();
																																																					
	ds18b20_PortInit();                                        //	Init PortB.12 for data transfering 
	while ( ds18b20_Reset() ) ;                                // 	If DS18B20 exists then continue

	devCount = Search_ROM( ROM_SEARCH , &sensors );            // 	Init Search ROM routine

	for ( size_t i = 0 ; i < devCount ; ++i )
	{
		ds18b20_Init( 1 , &( sensors[ i ] ) );
	}
	
	unsigned char charTemp[ 10ul ];                            // 7 chars is max for RES_BIT12
	uint8_t ret = 0;

	while ( 1 )
	{
		if ( end_command_flag )
		{
			command_var = 0;
			
			for ( size_t i = 0 ; i < var_cnt ; ++i )
			{
				symbol = var_in_char[ i ];
				
				if ( ( symbol >= '0' ) && ( symbol <= '9' ) )
				{
					num_pow = var_cnt - i - 1;
					command_var += num_pow ? ( symbol - '0' ) * 10 * num_pow : symbol - '0';
				}
				else if ( ( i == 0 ) && ( symbol >= '-' ) )
					command_var = 1 << 7;
			}
			
			switch ( command )
			{
				case 'R' : writeRes( command_var ); break;
				case 'h' : writeTempHigh( command_var ); break;
				case 'l' : writeTempLow( command_var ); break;
				default  : break;
			}
			
			end_command_flag = 0;
		}
		
		for ( uint8_t i = 0 ; i < devCount ; ++i )
		{
			if ( !sensors[ i ].crc8_data_error )
				ds18b20_ConvertTemp( 1 , sensors[ i ].ROM_code );
			
			DelayMicro( checkDelay( resolution ) );

			waitWhileRightCRC( &( sensors[ i ] ) );

			if ( !sensors[ i ].crc8_data_error )
			{                                              // 	Check if data correct
				sensors[ i ].raw_temp = ( ( uint16_t )sensors[ i ].scratchpad_data[ 1 ] << 8 ) | sensors[ i ].scratchpad_data[ 0 ]; // 	Get 16 bit temperature with sign
				sensors[ i ].temp     = sensors[ i ].raw_temp * 0.0625;    // 	Convert to float
			}
			
			ret = snprintf( charTemp , sizeof charTemp , "%.4f" , sensors[ i ].temp );
			
			for ( uint8_t j = 0 ; j < ret ; ++j )
			{
				USART3_Send( charTemp[ j ] );
			}
			
			USART3_Send( ' ' );
		}
		
		USART3_Send( '\r' );
	}

	return 0;
}