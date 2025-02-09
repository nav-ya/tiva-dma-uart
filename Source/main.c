#include "TM4C123.h"

// UART0 Registers
#define UART0_DR        (*((volatile unsigned long *)0x4000C000)) 
#define UART0_FR        (*((volatile unsigned long *)0x4000C018)) 
#define UART0_CTL       (*((volatile unsigned long *)0x4000C030))
#define UART0_IBRD      (*((volatile unsigned long *)0x4000C024))
#define UART0_FBRD      (*((volatile unsigned long *)0x4000C028))
#define UART0_LCRH      (*((volatile unsigned long *)0x4000C02C))
#define UART0_CC        (*((volatile unsigned long *)0x4000CFC8))

// GPIOA Registers
#define GPIO_PORTA_AFSEL (*((volatile unsigned long *)0x40004420))
#define GPIO_PORTA_PCTL  (*((volatile unsigned long *)0x4000452C))
#define GPIO_PORTA_DEN   (*((volatile unsigned long *)0x4000451C))

// DMA Registers
#define UDMA_CFG         (*((volatile unsigned long *)0x400FF004))
#define UDMA_CTLBASE     (*((volatile unsigned long *)0x400FF008))
#define UDMA_ENASET      (*((volatile unsigned long *)0x400FF014))
#define UDMA_CHMAP0      (*((volatile unsigned long *)0x400FF510))

// DMA Control Table
#define DMA_CHANNEL_UART0_RX  8
#define DMA_CHANNEL_UART0_TX  9
#define UART0_RX_BUFFER_SIZE  16
#define UART0_TX_BUFFER_SIZE  16

volatile uint8_t txBuffer[UART0_TX_BUFFER_SIZE] = "Hello DMA UART!";
volatile uint8_t rxBuffer[UART0_RX_BUFFER_SIZE];

typedef struct {
    volatile uint32_t srcEndAddr;
    volatile uint32_t dstEndAddr;
    volatile uint32_t control;
    volatile uint32_t unused;
} DMAControlTable;

DMAControlTable dmaTable[64] __attribute__((aligned(1024)));

void UART0_Init(void) {
    // Enable UART0 and GPIOA Clocks
    SYSCTL->RCGCUART |= (1 << 0);
    SYSCTL->RCGCGPIO |= (1 << 0);

    // Configure PA0 as UART0 TX and PA1 as UART0 RX
    GPIO_PORTA_AFSEL |= (1 << 0) | (1 << 1);
    GPIO_PORTA_PCTL  |= (1 << 0) | (1 << 4);
    GPIO_PORTA_DEN   |= (1 << 0) | (1 << 1);

    // Disable UART0 before configuring
    UART0_CTL &= ~(1 << 0);

    // Set baud rate to 115200
    UART0_IBRD = 8;     // Integer part
    UART0_FBRD = 44;    // Fractional part
    UART0_LCRH = (3 << 5); // 8-bit, no parity, 1-stop bit

    // Enable UART0
    UART0_CTL |= (1 << 0) | (1 << 8) | (1 << 9);
}

void DMA_Init(void) {
    // Enable DMA Clock
    SYSCTL->RCGCDMA |= 1;
    
    // Enable DMA and configure base address
    UDMA_CFG = 1;
    UDMA_CTLBASE = (uint32_t)dmaTable;

    // Configure DMA for UART0 RX
    dmaTable[DMA_CHANNEL_UART0_RX].srcEndAddr = (uint32_t)&UART0_DR;
    dmaTable[DMA_CHANNEL_UART0_RX].dstEndAddr = (uint32_t)(rxBuffer + UART0_RX_BUFFER_SIZE - 1);
    dmaTable[DMA_CHANNEL_UART0_RX].control = (UART0_RX_BUFFER_SIZE << 4) | (1 << 30) | (1 << 28);
    
    // Configure DMA for UART0 TX
    dmaTable[DMA_CHANNEL_UART0_TX].srcEndAddr = (uint32_t)(txBuffer + UART0_TX_BUFFER_SIZE - 1);
    dmaTable[DMA_CHANNEL_UART0_TX].dstEndAddr = (uint32_t)&UART0_DR;
    dmaTable[DMA_CHANNEL_UART0_TX].control = (UART0_TX_BUFFER_SIZE << 4) | (1 << 30) | (1 << 28);
    
    // Enable DMA Channels for UART0 RX and TX
    UDMA_ENASET |= (1 << DMA_CHANNEL_UART0_RX) | (1 << DMA_CHANNEL_UART0_TX);
    
    // Map UART0 RX and TX to DMA Channel
    UDMA_CHMAP0 |= (DMA_CHANNEL_UART0_RX << 8) | (DMA_CHANNEL_UART0_TX << 16);
}

void Start_UART_DMA_Transfer(void) {
    // Start DMA UART TX Transfer
    UDMA_ENASET |= (1 << DMA_CHANNEL_UART0_TX);
}

int main(void) {
    UART0_Init();
    DMA_Init();
    Start_UART_DMA_Transfer();

    while (1);
}
