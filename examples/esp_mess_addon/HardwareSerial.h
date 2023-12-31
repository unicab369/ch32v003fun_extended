#define OVER8DIV_X 4
#define RX_BUFFER_SIZE 100
uint8_t rxBuffer[RX_BUFFER_SIZE] = {0};
volatile uint8_t rxBufferHead = 0;
volatile uint8_t rxBufferTail = 0;

void hwSerial_begin(unsigned long baud, uint16_t config) {
    // Hardware Serial Pins D5 / D6
   RCC->APB2PCENR |= RCC_APB2Periph_GPIOD | RCC_APB2Periph_USART1; // Enable UART
   GPIOD->CFGLR &= ~(0xf<<(4*5));
   GPIOD->CFGLR |= (GPIO_Speed_10MHz | GPIO_CNF_OUT_PP_AF)<<(4*5);

   GPIOD->CFGLR &= ~(0xf<<(4*6));
   GPIOD->CFGLR |= (GPIO_CNF_IN_FLOATING)<<(4*6);

   USART1->CTLR1 = USART_WordLength_8b | USART_Parity_No | USART_Mode_Rx | USART_Mode_Tx;
   USART1->CTLR2 = USART_StopBits_1;
   USART1->CTLR3 = USART_HardwareFlowControl_None;

   // Set Baudrate
   uint32_t integerDivider = ((25 * APB_CLOCK)) / (OVER8DIV_X * baud);
   uint32_t fractionalDivider = integerDivider % 100;

   USART1->BRR = ((integerDivider / 100) << 4) | (((fractionalDivider * (OVER8DIV_X * 2) + 50) / 100) & 7);

   // Enable Interrupt
   USART1->CTLR1 |= USART_FLAG_RXNE;
   NVIC_EnableIRQ(USART1_IRQn);

   // Enable UART
   USART1->CTLR1 |= CTLR1_UE_Set;
}

// void hwSerial_begin(unsigned long baud) {
//    hwSerial_begin(baud, 0);
// }

void hwSerial_end() {
   USART1->CTLR1 &= CTLR1_UE_Reset;
}

int hwSerial_available() {
   return rxBufferTail - rxBufferHead;
}

int hwSerial_peek() {
   return rxBuffer[rxBufferHead];
}

int hwSerial_read() {
   if(rxBufferHead == rxBufferTail) return -1;

   uint8_t c = rxBuffer[rxBufferHead];

   rxBufferHead = (rxBufferHead + 1) % RX_BUFFER_SIZE;
   if(rxBufferHead != rxBufferTail) {
      USART1->CTLR1 |= USART_FLAG_RXNE;
   }

   return c;
}

void hwSerial_flush() {
}

int hwSerial_availableForWrite() {
   return USART1->CTLR1 & CTLR1_UE_Set;
}

// size_t hwSerial_write(uint8_t c) {
//    while( !(USART1->STATR & USART_FLAG_TC));
//    USART1->DATAR = c;
//    return 1;
// }

size_t hwSerial_write(const uint8_t *buffer, size_t size)
{
   size_t n = 0;
   while (size--) {
      if (write(*buffer++)) n++;
      else break;
   }
   return n;
}

// HardwareSerial::operator bool() {
//    return availableForWrite();
// }

// Removed IRQ handler for testing
void USART1_IRQHandler(void) __attribute__((interrupt));
void USART1_IRQHandler(void) {
   if(USART1->STATR & USART_FLAG_RXNE) {
      // Write into buffer
      rxBuffer[rxBufferTail] = USART1->DATAR & (uint16_t)0x01FF;

      rxBufferTail = (rxBufferTail + 1) % RX_BUFFER_SIZE;
      printf("*** IM HERE  4444");

      if(rxBufferTail == rxBufferHead) {
         // Buffer empty, disable USART interrupt
         USART1->CTLR1 &= ~USART_FLAG_RXNE;
         return;
      }
   }
}