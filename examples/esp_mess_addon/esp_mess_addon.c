#include "ch32v003fun.h"
#include "ch32v_hal.inl"
#include <stdio.h>
#include "TestClass.h"
#include "i2c_slave.h"

//#define SSD1306_64X32
// #define SSD1306_128X32
#define SSD1306_128X64

#include "ssd1306_i2c.h"
#include "ssd1306.h"
#include "HardwareSerial.h"

/* some bit definitions for systick regs */
#define SYSTICK_SR_CNTIF (1<<0)
#define SYSTICK_CTLR_STE (1<<0)
#define SYSTICK_CTLR_STIE (1<<1)
#define SYSTICK_CTLR_STCLK (1<<2)
#define SYSTICK_CTLR_STRE (1<<3)
#define SYSTICK_CTLR_SWIE (1<<31)

volatile uint32_t systick_cnt;

// Start up the SysTick IRQ
void systick_init(void) {
	SysTick->CTLR = 0;                  // disable default SysTick behavior   
	NVIC_EnableIRQ(SysTicK_IRQn);       // enable the SysTick IRQ
	
	/* Set the tick interval to 1ms for normal op */
	SysTick->CMP = (FUNCONF_SYSTEM_CORE_CLOCK/1000)-1;
	
	/* Start at zero */
	SysTick->CNT = 0;
	systick_cnt = 0;
	
	/* Enable SysTick counter, IRQ, HCLK/1 */
	SysTick->CTLR = SYSTICK_CTLR_STE | SYSTICK_CTLR_STIE | SYSTICK_CTLR_STCLK;
}

// SysTick ISR just counts ticks
// note - the __attribute__((interrupt)) syntax is crucial!
void SysTick_Handler(void) __attribute__((interrupt));
void SysTick_Handler(void) {
	// move the compare further ahead in time. as a warning, if more than this length of time
	// passes before triggering, you may miss your interrupt.
	SysTick->CMP += (FUNCONF_SYSTEM_CORE_CLOCK/1000);
	SysTick->SR = 0;     // clear IRQ
	systick_cnt++;       // update counter
}

uint32_t delayTime = 500;

uint8_t bh17Out[2];
uint8_t sht31ReadCmd[2];

volatile uint8_t i2c_registers[32] = {0x00, 0x01, 0x02, 0x03, 0x04, 0x05};
volatile uint32_t systick_cnt;

void toggleLed() {
   digitalWrite(0xC0, 1);
   digitalWrite(0xA1, 1);
	digitalWrite(0xD2, 1);
   Delay_Ms(delayTime);

   digitalWrite(0xC0, 0);
   digitalWrite(0xA1, 0);
	digitalWrite(0xD2, 0);
   Delay_Ms(delayTime);
}

uint32_t count = 0;
uint8_t key;
uint8_t value = 0; 


int lastkey = 0;
void handle_debug_input(int numbytes, uint8_t * data) {
	if(numbytes > 0 )
		digitalWrite(0xC0, value);
		value = (value == 0) ? 1 : 0;
		delay(500);

		// printf("\nIM HERE 2222 = %d", numbytes);
		lastkey = data[numbytes-1];
}

char readings[100] = "";
int readIndex = 0;

int main() {	
	SystemInit();
	// Delay_Ms(100);

   pinMode(0xC0, OUTPUT);
	pinMode(0xC3, OUTPUT);
   pinMode(0xA1, OUTPUT);
	pinMode(0xD2, OUTPUT);
	digitalWrite(0xC3, 1);
   digitalWrite(0xA1, 1);
	digitalWrite(0xD2, 1);

   pinMode(0xD0, INPUT_PULLUP);
   pinMode(0xD2, INPUT_PULLUP);
   pinMode(0xD3, INPUT_PULLUP);

   SetupI2CSlave(0x78, i2c_registers, sizeof(i2c_registers));

	// init systick @ 1ms rate
	// systick_init();

   // I2CInit(0xC1, 0xC2, 100000);
   // uint8_t bh17Mode[1] = { 0x13 };
   // I2CWrite(0x23, bh17Mode, sizeof(bh17Mode));

	// ssd1306_i2c_init();
	// ssd1306_init();
	UARTInit(9600, 0);
	// hwSerial_begin(9600, 0);

	while(1) {	
		int check = 1;
      // int check = I2CTest(0x23);
      // I2CRead(0x23, bh17Out, 2);
      // printf("\nCheck = %d, Value = %02X %02X", check, bh17Out[0], bh17Out[1]);
		// printf("\nIM HERE 3");

		// char myStr[] = "\nHELLO WORLD 3";
		// UART_Write(myStr, sizeof(myStr));
      
      // uint8_t read1 = digitalRead(0xD0);
      // uint8_t read2 = digitalRead(0xD2);
      // uint8_t read3 = digitalRead(0xD3);
      // printf("\nRead1 = %u, Read2 = %u, Read3 = %u", read1, read2, read3);

      // if (i2c_registers[0] & 1) { // Turn on LED (PD0) if bit 1 of register 0 is set
      //    digitalWrite(0xC0, 1);
      //    printf("\n\n**********IM HEREEEEEEEEEEEE");
      //    Delay_Ms(1000);
      // } else {
      //    digitalWrite(0xC0, 0);
      // }

		// mini_pprintf 

		// while (hwSerial_available()) {
		// 	printStuff();
		// 	// delay(50);
		// }

		// uint8_t read = i2c_registers[0] & 1;
		// digitalWrite(0xD2, read);

      // toggleLed();
		// printf("Print #: %lu / Milliseconds: %lu / CNT: %lu\n", count++, systick_cnt, SysTick->CNT);
		// printStuff();
		// Delay_Ms(1000);
	}
}

void printStuff() {
	// clear buffer for next mode
	ssd1306_setbuf(0);
	ssd1306_drawstr_sz(0,32, "16x16", 1, fontsize_16x16);

	char strOut[16];
	count++;
	mini_snprintf(strOut, sizeof(strOut), "%lu", count);
	ssd1306_drawstr_sz(0,0, strOut, 1, fontsize_8x8);
	
	// int read = hwSerial_read();

	// while (read) {
	// 	read = hwSerial_read();
	// }
	// // for (int i=0; i<30; i++) {
	// // 	read = hwSerial_read();
	// // 	readings[i] = read;
	// // }

	char strOut2[16];
	mini_snprintf(strOut2, sizeof(strOut2), "%s\n", readings);
	ssd1306_drawstr_sz(0,16, strOut2, 1, fontsize_8x8);
	ssd1306_refresh();
}





// int main() {
// 	SystemInit();

// 	// This delay gives us some time to reprogram the device. 
// 	// Otherwise if the device enters standby mode we can't 
// 	// program it any more.
// 	Delay_Ms(5000);

// 	// Set all GPIOs to input pull up
// 	RCC->APB2PCENR |= RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOD;

// 	// GPIOA: Set to output
// 	// CFGLR = configuration "low" register (there is a high on other processors)
// 	// BSHR = bit set and clear register
// 	// each 4 bits of CFGLR defines the mode of a GPIO pin (8x4 = 32 bits)
// 	GPIOA->CFGLR = (GPIO_CNF_IN_PUPD<<(4*2)) | (GPIO_CNF_IN_PUPD<<(4*1));
// 	GPIOA->BSHR = GPIO_BSHR_BS2 | GPIO_BSHR_BR1;
// 	GPIOC->CFGLR = (GPIO_CNF_IN_PUPD<<(4*7)) | (GPIO_CNF_IN_PUPD<<(4*6)) |
// 						(GPIO_CNF_IN_PUPD<<(4*5)) | (GPIO_CNF_IN_PUPD<<(4*4)) |
// 						(GPIO_CNF_IN_PUPD<<(4*3)) | (GPIO_CNF_IN_PUPD<<(4*2)) |
// 						(GPIO_CNF_IN_PUPD<<(4*1)) | (GPIO_CNF_IN_PUPD<<(4*0));
// 	GPIOC->BSHR = GPIO_BSHR_BS7 | GPIO_BSHR_BS6 | GPIO_BSHR_BS5 | GPIO_BSHR_BS4 |
// 						GPIO_BSHR_BS3 | GPIO_BSHR_BS2 | GPIO_BSHR_BS1 | GPIO_BSHR_BS0;
// 	GPIOD->CFGLR = (GPIO_CNF_IN_PUPD<<(4*7)) | (GPIO_CNF_IN_PUPD<<(4*6)) |
// 						(GPIO_CNF_IN_PUPD<<(4*5)) | (GPIO_CNF_IN_PUPD<<(4*4)) |
// 						(GPIO_CNF_IN_PUPD<<(4*3)) | (GPIO_CNF_IN_PUPD<<(4*2)) |
// 						(GPIO_CNF_IN_PUPD<<(4*0));
// 	GPIOD->BSHR = GPIO_BSHR_BS7 | GPIO_BSHR_BS6 | GPIO_BSHR_BS5 |
// 						GPIO_BSHR_BS4 | GPIO_BSHR_BS3 | GPIO_BSHR_BS2 | GPIO_BSHR_BS0;

// 	// enable power interface module clock
// 	RCC->APB1PCENR |= RCC_APB1Periph_PWR;

// 	// enable low speed oscillator (LSI)
// 	RCC->RSTSCKR |= RCC_LSION;
// 	while ((RCC->RSTSCKR & RCC_LSIRDY) == 0) {}

// 	// enable AutoWakeUp event
// 	EXTI->EVENR |= EXTI_Line9;
// 	EXTI->FTENR |= EXTI_Line9;

// 	// configure AWU prescaler
// 	PWR->AWUPSC |= PWR_AWU_Prescaler_61440;

// 	// configure AWU window comparison value
// 	PWR->AWUWR &= ~0x3f;
// 	PWR->AWUWR |= 20;

// 	// enable AWU
// 	PWR->AWUCSR |= (1 << 1);

// 	// select standby on power-down
// 	PWR->CTLR |= PWR_CTLR_PDDS;

// 	// peripheral interrupt controller send to deep sleep
// 	PFIC->SCTLR |= (1 << 2);

// 	uint16_t counter = 0;
// 	printf("entering sleep loop\r\n");

//    //! `t = AWUWR*AWUPSC/128000  

// 	for (;;) {
// 		__WFE();
// 		// restore clock to full speed
// 		SystemInit();
// 		SetClock(24000000);
// 		printf("\r\nawake, %u\r\n", counter++);
// 	}
// }
