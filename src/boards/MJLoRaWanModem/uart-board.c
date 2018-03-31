/*!
 * \file      uart-board.c
 *
 * \brief     Target board UART driver implementation
 *
 * \copyright Revised BSD License, see section \ref LICENSE.
 *
 * \code
 *                ______                              _
 *               / _____)             _              | |
 *              ( (____  _____ ____ _| |_ _____  ____| |__
 *               \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 *               _____) ) ____| | | || |_| ____( (___| | | |
 *              (______/|_____)_|_|_| \__)_____)\____)_| |_|
 *              (C)2013-2017 Semtech
 *
 * \endcode
 *
 * \author    Miguel Luis ( Semtech )
 *
 * \author    Gregory Cristian ( Semtech )
 */
#include "lpc824board.h"
#include "utilities.h"
#include "board.h"
#include "uart-board.h"
#if defined( USE_USB_CDC )
#include "uart-usb-board.h"
#endif

#define USART             USART1
#define USART_TX_PORT     GPIOA
#define USART_TX_PIN      9
#define USART_RX_PORT     GPIOA
#define USART_RX_PIN      10
#define USART_AF          0x07
#define USART_RCCREG      APB2ENR
#define USART_RCCVAL      RCC_APB2ENR_USART1EN
#define USART_IRQN        USART0_IRQn
#define USART_IRQHANDLER  UART0_IRQHandler

/*****************************************************************************
 * Private types/enumerations/variables
 ****************************************************************************/

/* Transmit and receive ring buffers */
RINGBUFF_T txring, rxring;

/* Ring buffer size */
#define UART_RB_SIZE 1024

/* Set the default UART, IRQ number, and IRQ handler name */
#define LPC_USART       LPC_USART0
#define LPC_IRQNUM      UART0_IRQn
#define LPC_UARTHNDLR   UART0_IRQHandler

/* Default baudrate for testing */
#define UART_TEST_DEFAULT_BAUDRATE 115200
#define	UART_CLOCK_DIV	1

/* Transmit and receive buffers */
static uint8_t rxbuff[UART_RB_SIZE], txbuff[UART_RB_SIZE];

/*!
 * Number of times the UartPutBuffer will try to send the buffer before
 * returning ERROR
 */

void UartMcuInit( Uart_t *obj, UartId_t uartId, PinNames tx, PinNames rx )
{
    BoardDisableIrq( );
    /* Enable the clock to the Switch Matrix */
    Chip_Clock_EnablePeriphClock(SYSCTL_CLOCK_SWM);

    Chip_Clock_SetUARTClockDiv(UART_CLOCK_DIV);

    /* Connect the U0_TXD_O and U0_RXD_I signals to port pins(P0.4, P0.0) */
    Chip_SWM_DisableFixedPin(SWM_FIXED_ACMP_I1);
    Chip_SWM_MovablePinAssign(SWM_U0_TXD_O, 0);
    Chip_SWM_MovablePinAssign(SWM_U0_RXD_I, 4);
    Chip_Clock_DisablePeriphClock(SYSCTL_CLOCK_SWM);
    /* Setup UART */
    Chip_UART_Init(LPC_USART);
    Chip_UART_ConfigData(LPC_USART, UART_CFG_DATALEN_8 | UART_CFG_PARITY_NONE | UART_CFG_STOPLEN_1);
    Chip_Clock_SetUSARTNBaseClockRate((115200 * 16), true);
    Chip_UART_SetBaud(LPC_USART, UART_TEST_DEFAULT_BAUDRATE);
    Chip_UART_Enable(LPC_USART);
    Chip_UART_TXEnable(LPC_USART);

    /* Before using the ring buffers, initialize them using the ring
    buffer init function */
    RingBuffer_Init(&rxring, rxbuff, 1, UART_RB_SIZE);
    RingBuffer_Init(&txring, txbuff, 1, UART_RB_SIZE);

    /* Enable receive data and line status interrupt */
    Chip_UART_IntEnable(LPC_USART, UART_INTEN_RXRDY);
    Chip_UART_IntDisable(LPC_USART, UART_INTEN_TXRDY);	/* May not be needed */

    /* preemption = 1, sub-priority = 1 */
    NVIC_EnableIRQ(LPC_IRQNUM);
    BoardEnableIrq( );
}


