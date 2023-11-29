#include "ch32v003fun.h"
#include "ch32v_hal.inl"
#include "i2c_slave.h"
#include <stdio.h>

#define SSD1306_128X64
#include "ssd1306_i2c.h"
#include "ssd1306.h"

volatile uint8_t i2c_registers[32] = {0x00, 0x03, 0x04, 0x05, 0x06};
volatile uint32_t systick_cnt;
uint32_t count = 0;

int main() {
    SystemInit();


    pinMode(0xC0, OUTPUT);
    pinMode(0xC3, OUTPUT);
    pinMode(0xA1, OUTPUT);
    pinMode(0xD2, OUTPUT);

    digitalWrite(0xC3, 1);
    digitalWrite(0xA1, 1);
    digitalWrite(0xD2, 1);

	systick_init();     // init systick @ 1ms rate

    // SetupI2CSlave(0x78, i2c_registers, sizeof(i2c_registers));
    // I2CInit(0xC1, 0xC2, 100000);
    // BH17_Setup();
	// ssd1306_init();

    Delay_Ms(100);
    I2CInit(0xC1, 0xC2, 100000);
    BH17_Setup();

    while (1) {
        // printStuff();
		printf("\nPrint #: %lu / ms: %lu / CNT: %lu\n", count++, systick_cnt, SysTick->CNT);
        toggleLed();
        BH17_Read();
    }
}

#define BH17_CONT_HI1 0x10      // 1 lux resolution 120ms
#define BH17_CONT_HI2 0x11      // .5 lux resolution 120ms
#define BH17_CONT_LOW 0x13      // 4 lux resolution 16ms
#define BH17_ONCE_HI1 0x20      // 1 lux resolution 120ms
#define BH17_ONCE_HI2 0x21      // .5 lux resolution 120ms
#define BH17_ONCE_LOW 0x23      // 4 lux resolution 16ms

uint16_t hexToDecimal(const uint8_t* hexArray, size_t length) {
    uint16_t result = 0;
    
    for (size_t i = 0; i < length; i++) {
        result = (result << 8) | hexArray[i];
    }
    
    return result;
}

void BH17_Setup() {
    uint8_t BH17_ADDR = 0x23;
    uint8_t bh17Mode[1] = { BH17_CONT_LOW };
    I2CWrite(0x23, bh17Mode, sizeof(bh17Mode));
}

void BH17_Read() {
    uint8_t bh17Out[2];
    int check = I2CTest(0x23);
    I2CRead(0x23, bh17Out, 2);
    uint16_t value = hexToDecimal(bh17Out, sizeof(bh17Out));
    printf("\nCheck = %d, Value = %u", check, value);
}

uint32_t delayTime = 1000;

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
	// mini_snprintf(strOut2, sizeof(strOut2), "%s\n", readings);
	ssd1306_drawstr_sz(0,16, strOut2, 1, fontsize_8x8);
	ssd1306_refresh();
}


/* some bit definitions for systick regs */
#define SYSTICK_SR_CNTIF (1<<0)
#define SYSTICK_CTLR_STE (1<<0)
#define SYSTICK_CTLR_STIE (1<<1)
#define SYSTICK_CTLR_STCLK (1<<2)
#define SYSTICK_CTLR_STRE (1<<3)
#define SYSTICK_CTLR_SWIE (1<<31)

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
