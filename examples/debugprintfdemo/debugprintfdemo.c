/* Small example showing how to use the SWIO programming pin to 
   do printf through the debug interface */

#include "ch32v003fun.h"
#include <stdio.h>

// uint32_t count;

// int last = 0;
// void handle_debug_input( int numbytes, uint8_t * data )
// {
// 	last = data[0];
// 	count += numbytes;
// }

// int main()
// {
// 	SystemInit();

// 	// Enable GPIOs
// 	RCC->APB2PCENR |= RCC_APB2Periph_GPIOD | RCC_APB2Periph_GPIOC;

// 	// GPIO D0 Push-Pull
// 	GPIOD->CFGLR &= ~(0xf<<(4*0));
// 	GPIOD->CFGLR |= (GPIO_Speed_10MHz | GPIO_CNF_OUT_PP)<<(4*0);

// 	// GPIO C0 Push-Pull
// 	GPIOC->CFGLR &= ~(0xf<<(4*0));
// 	GPIOC->CFGLR |= (GPIO_Speed_10MHz | GPIO_CNF_OUT_PP)<<(4*0);

// 	while(1)
// 	{
// 		GPIOD->BSHR = 1;	 // Turn on GPIOs
// 		GPIOC->BSHR = 1;
// 		printf( "+%lu\n", count++ );
// 		Delay_Ms(100);
// 		int i;
// 		for( i = 0; i < 10000; i++ )
// 			poll_input();
// 		GPIOD->BSHR = (1<<16); // Turn off GPIODs
// 		GPIOC->BSHR = (1<<16);
// 		printf( "-%lu[%c]\n", count++, last );
// 		Delay_Ms(100);
// 	}
// }

#include <string.h>

#define CMD_BUFFER_SIZE			(64)
#define CMD_POLL_PERIOD_MS		(100)

char cmd_buffer[CMD_BUFFER_SIZE];
uint8_t cmd_buffer_index = 0;

/* Callback from poll_input() and putchar() - not an ISR but can't block here too */
void handle_debug_input(int size, uint8_t *buffer)
{
	uint8_t cmd_ready = 0;

	while (size--) {
		/* Treate a new-line character as the end of the command  */
		if ((*buffer == '\r') || (*buffer == '\n')) {
			cmd_ready = cmd_buffer_index;
		}
		else {
			cmd_buffer[cmd_buffer_index++] = *buffer++;
			/* Treate a full buffer as the end of the command  */
			if (cmd_buffer_index >= CMD_BUFFER_SIZE - 1)
				cmd_ready = 1;
		}
		/* Signal main function a new command is readable from the buffer */
		if (cmd_ready) {
			cmd_buffer[cmd_buffer_index] = '\0';
			cmd_buffer_index = CMD_BUFFER_SIZE;
			/* Ignore the rest of the input - main will now handle the command buffer */
			return;
		}
	}
}

int main()
{
	SystemInit();

	SysTick->CNT = 0;
	uint32_t cur_time = SysTick->CNT;
	uint32_t next_putchar_time = 0;

	printf("Enter command and press ENTER:\n");
	while(1)
	{
		cur_time = SysTick->CNT;

		/* Poll debug input every CMD_POLL_PERIOD_MS */
		if (((int32_t)(cur_time - next_putchar_time)) > 0) {
			next_putchar_time = cur_time + Ticks_from_Ms(CMD_POLL_PERIOD_MS);
			/* poll_input() not working, so I find this hack */
			/* Proccess debug line DMA buffer and calls handle_debug_input() when needed */
			putchar('\0');
		}

		/* Did we got a full command? (debug input that ends with a new-line) */
		if (cmd_buffer_index == CMD_BUFFER_SIZE) {
			printf("> [%02d] %s\n", strlen(cmd_buffer), cmd_buffer);
			*cmd_buffer = '\0';
			cmd_buffer_index = 0;
		}
	}
}