/*
 * @brief NXP LPCXpresso LPC824 board file
 *
 * @note
 * Copyright(C) NXP Semiconductors, 2013
 * All rights reserved.
 *
 * @par
 * Software that is described herein is for illustrative purposes only
 * which provides customers with programming information regarding the
 * LPC products.  This software is supplied "AS IS" without any warranties of
 * any kind, and NXP Semiconductors and its licensor disclaim any and
 * all warranties, express or implied, including all implied warranties of
 * merchantability, fitness for a particular purpose and non-infringement of
 * intellectual property rights.  NXP Semiconductors assumes no responsibility
 * or liability for the use of the software, conveys no license or rights under any
 * patent, copyright, mask work right, or any other intellectual property rights in
 * or to any products. NXP Semiconductors reserves the right to make changes
 * in the software without notification. NXP Semiconductors also makes no
 * representation or warranty that such application will be suitable for the
 * specified use without further testing or modification.
 *
 * @par
 * Permission to use, copy, modify, and distribute this software and its
 * documentation is hereby granted, under NXP Semiconductors' and its
 * licensor's relevant copyrights in the software, without fee, provided that it
 * is used in conjunction with NXP Semiconductors microcontrollers.  This
 * copyright, permission, and disclaimer notice must appear in all copies of
 * this code.
 */

#include "lpc824board.h"
#include "string.h"
#include "retarget.h"

/*****************************************************************************
 * Private types/enumerations/variables
 ****************************************************************************/
#define	UART_CLOCK_DIV	1
#define LEDSAVAIL 1
static const uint8_t ledBits[LEDSAVAIL] = {11};

/*****************************************************************************
 * Public types/enumerations/variables
 ****************************************************************************/

/* System oscillator rate and clock rate on the CLKIN pin */
const uint32_t OscRateIn = 12000000;
const uint32_t ExtRateIn = 0;
static uint8_t ledstatus = 0;
/*****************************************************************************
 * Private functions
 ****************************************************************************/

/* Initialize the LEDs on the NXP LPC824 LPCXpresso Board */
static void Board_LED_Init(void)
{
	int i;

	for (i = 0; i < LEDSAVAIL; i++) {
		Chip_GPIO_SetPinDIROutput(LPC_GPIO_PORT, 0, ledBits[i]);
		Chip_GPIO_SetPinState(LPC_GPIO_PORT, 0, ledBits[i], true);
	}
}

/* Board Debug UART Initialisation function */
STATIC void Board_UART_Init(void)
{
	/* Enable the clock to the Switch Matrix */
	Chip_Clock_EnablePeriphClock(SYSCTL_CLOCK_SWM);

	Chip_Clock_SetUARTClockDiv(UART_CLOCK_DIV);

#if (defined(BOARD_NXP_LPCXPRESSO_812) || defined(BOARD_LPC812MAX) || defined(BOARD_NXP_LPCXPRESSO_824))
	/* Connect the U0_TXD_O and U0_RXD_I signals to port pins(P0.4, P0.0) */
	Chip_SWM_DisableFixedPin(SWM_FIXED_ACMP_I1);
	Chip_SWM_MovablePinAssign(SWM_U0_TXD_O, 4);
	Chip_SWM_MovablePinAssign(SWM_U0_RXD_I, 0);
#else
	/* Configure your own UART pin muxing here if needed */
#warning "No UART pin muxing defined"
#endif

	/* Disable the clock to the Switch Matrix to save power */
	Chip_Clock_DisablePeriphClock(SYSCTL_CLOCK_SWM);
}

/*****************************************************************************
 * Public functions
 ****************************************************************************/

/* Set the LED to the state of "On" */
void Board_LED_Set(uint8_t LEDNumber, bool On)
{
	if (LEDNumber < LEDSAVAIL) {
		Chip_GPIO_SetPinState(LPC_GPIO_PORT, 0, ledBits[LEDNumber], (bool) !On);
                ledstatus = On;
	}
}

uint8_t Board_LED_Get(uint8_t LEDNumber)
{
    if (LEDNumber < LEDSAVAIL) {
        return ledstatus;
    }
    return ledstatus;
}

/* Return the state of LEDNumber */
bool Board_LED_Test(uint8_t LEDNumber)
{
	bool state = false;

	if (LEDNumber < LEDSAVAIL) {
		state = (bool) !Chip_GPIO_GetPinState(LPC_GPIO_PORT, 0, ledBits[LEDNumber]);
	}

	return state;
}

/* Toggles the current state of a board LED */
void Board_LED_Toggle(uint8_t LEDNumber)
{
	if (LEDNumber < LEDSAVAIL) {
		Chip_GPIO_SetPinToggle(LPC_GPIO_PORT, 0, ledBits[LEDNumber]);
	}
}

/*  Classic implementation of itoa -- integer to ASCII */
char *Board_itoa(int value, char *result, int base)
{
    char* ptr = result, *ptr1 = result, tmp_char;
    int tmp_value;

    if (base < 2 || base > 36) { *result = '\0'; return result; }
    do {
        tmp_value = value;
        value /= base;
        *ptr++ = "zyxwvutsrqponmlkjihgfedcba9876543210123456789abcdefghijklmnopqrstuvwxyz" [35 + (tmp_value - value * base)];
    } while ( value );

    if (tmp_value < 0) *ptr++ = '-';
    *ptr-- = '\0';
    while (ptr1 < ptr) {
        tmp_char = *ptr;
        *ptr--= *ptr1;
        *ptr1++ = tmp_char;
    }
    return result;
}

/* Sends a character on the UART */
void Board_UARTPutChar(char ch)
{
#if defined(DEBUG_UART)
	Chip_UART_SendBlocking(DEBUG_UART, &ch, 1);
#endif
}

/* Gets a character from the UART, returns EOF if no character is ready */
int Board_UARTGetChar(void)
{
#if defined(DEBUG_UART)
	uint8_t data;

	if (Chip_UART_Read(DEBUG_UART, &data, 1) == 1) {
		return (int) data;
	}
#endif
	return EOF;
}

/* Outputs a string on the debug UART */
void Board_UARTPutSTR(const char *str)
{
#if defined(DEBUG_UART)
	while (*str != '\0') {
		Board_UARTPutChar(*str++);
	}
#endif
}

/* Initialize debug output via UART for board */
void Board_Debug_Init(void)
{
#if defined(DEBUG_UART)
	Board_UART_Init();
	//Chip_UART_Init(DEBUG_UART);
	//Chip_UART_ConfigData(DEBUG_UART, UART_CFG_DATALEN_8 | UART_CFG_PARITY_NONE | UART_CFG_STOPLEN_1);
	//Chip_Clock_SetUSARTNBaseClockRate((115200 * 16), true);
	//Chip_UART_SetBaud(DEBUG_UART, 115200);
	//Chip_UART_Enable(DEBUG_UART);
	//Chip_UART_TXEnable(DEBUG_UART);
        /* Enable receive data and line status interrupt */
	//Chip_UART_IntEnable(DEBUG_UART, UART_INTEN_RXRDY);
	//Chip_UART_IntEnable(DEBUG_UART, UART_INTEN_TXRDY);	/* May not be needed */
        //NVIC_EnableIRQ(UART0_IRQn);
        //Chip_UART_ClearStatus(DEBUG_UART,UART_STAT_RXRDY | UART_INTEN_TXRDY | UART_INTEN_TXIDLE);

#endif
}

void Board_GPIO_Init(void)
{
    Chip_Clock_EnablePeriphClock(SYSCTL_CLOCK_SWM);        
    Chip_SWM_DisableFixedPin(SWM_FIXED_ACMP_I1);
    Chip_SWM_DisableFixedPin(SWM_FIXED_ACMP_I2);
    Chip_SWM_DisableFixedPin(SWM_FIXED_ACMP_I3);
    Chip_SWM_DisableFixedPin(SWM_FIXED_ACMP_I4);
    //Chip_SWM_DisableFixedPin(SWM_FIXED_SWCLK);
    //Chip_SWM_DisableFixedPin(SWM_FIXED_SWDIO);
    Chip_SWM_DisableFixedPin(SWM_FIXED_XTALIN);
    Chip_SWM_DisableFixedPin(SWM_FIXED_XTALOUT);
    //Chip_SWM_DisableFixedPin(SWM_FIXED_RST);
    Chip_SWM_DisableFixedPin(SWM_FIXED_CLKIN);
    Chip_SWM_DisableFixedPin(SWM_FIXED_VDDCMP);
    Chip_SWM_DisableFixedPin(SWM_FIXED_I2C0_SDA);
    Chip_SWM_DisableFixedPin(SWM_FIXED_I2C0_SCL);
    Chip_SWM_DisableFixedPin(SWM_FIXED_ADC0);
    Chip_SWM_DisableFixedPin(SWM_FIXED_ADC1);
    Chip_SWM_DisableFixedPin(SWM_FIXED_ADC2);
    Chip_SWM_DisableFixedPin(SWM_FIXED_ADC3);
    Chip_SWM_DisableFixedPin(SWM_FIXED_ADC4);
    Chip_SWM_DisableFixedPin(SWM_FIXED_ADC5);
    Chip_SWM_DisableFixedPin(SWM_FIXED_ADC6);
    Chip_SWM_DisableFixedPin(SWM_FIXED_ADC7);
    Chip_SWM_DisableFixedPin(SWM_FIXED_ADC8);
    Chip_SWM_DisableFixedPin(SWM_FIXED_ADC9);
    Chip_SWM_DisableFixedPin(SWM_FIXED_ADC10);
    Chip_SWM_DisableFixedPin(SWM_FIXED_ADC11);
    Chip_Clock_DisablePeriphClock(SYSCTL_CLOCK_SWM);
        
    /*Chip_GPIO_SetPortDIRInput(LPC_GPIO_PORT,0,0x1FFFFFFF);
    //Chip_GPIO_SetPortValue(LPC_GPIO_PORT,0,~0x1FFFF7DF);
    Chip_IOCON_PinEnableOpenDrainMode(LPC_IOCON,IOCON_PIO0);
    Chip_IOCON_PinEnableOpenDrainMode(LPC_IOCON,IOCON_PIO1);
    //Chip_IOCON_PinEnableOpenDrainMode(LPC_IOCON,IOCON_PIO2);
    //Chip_IOCON_PinEnableOpenDrainMode(LPC_IOCON,IOCON_PIO3);
    Chip_IOCON_PinEnableOpenDrainMode(LPC_IOCON,IOCON_PIO4);
    Chip_IOCON_PinEnableOpenDrainMode(LPC_IOCON,IOCON_PIO5);
    Chip_IOCON_PinEnableOpenDrainMode(LPC_IOCON,IOCON_PIO6);
    Chip_IOCON_PinEnableOpenDrainMode(LPC_IOCON,IOCON_PIO7);
    Chip_IOCON_PinEnableOpenDrainMode(LPC_IOCON,IOCON_PIO8);
    Chip_IOCON_PinEnableOpenDrainMode(LPC_IOCON,IOCON_PIO9);
    Chip_IOCON_PinEnableOpenDrainMode(LPC_IOCON,IOCON_PIO10);
    Chip_IOCON_PinEnableOpenDrainMode(LPC_IOCON,IOCON_PIO11);
    Chip_IOCON_PinEnableOpenDrainMode(LPC_IOCON,IOCON_PIO12);
    Chip_IOCON_PinEnableOpenDrainMode(LPC_IOCON,IOCON_PIO13);
    Chip_IOCON_PinEnableOpenDrainMode(LPC_IOCON,IOCON_PIO14);
    Chip_IOCON_PinEnableOpenDrainMode(LPC_IOCON,IOCON_PIO15);
    Chip_IOCON_PinEnableOpenDrainMode(LPC_IOCON,IOCON_PIO16);
    Chip_IOCON_PinEnableOpenDrainMode(LPC_IOCON,IOCON_PIO17);
    Chip_IOCON_PinEnableOpenDrainMode(LPC_IOCON,IOCON_PIO18);
    Chip_IOCON_PinEnableOpenDrainMode(LPC_IOCON,IOCON_PIO19);
    Chip_IOCON_PinEnableOpenDrainMode(LPC_IOCON,IOCON_PIO20);
    Chip_IOCON_PinEnableOpenDrainMode(LPC_IOCON,IOCON_PIO21);
    Chip_IOCON_PinEnableOpenDrainMode(LPC_IOCON,IOCON_PIO22);
    Chip_IOCON_PinEnableOpenDrainMode(LPC_IOCON,IOCON_PIO23);
    Chip_IOCON_PinEnableOpenDrainMode(LPC_IOCON,IOCON_PIO24);
    Chip_IOCON_PinEnableOpenDrainMode(LPC_IOCON,IOCON_PIO25);
    Chip_IOCON_PinEnableOpenDrainMode(LPC_IOCON,IOCON_PIO26);
    Chip_IOCON_PinEnableOpenDrainMode(LPC_IOCON,IOCON_PIO27);
    Chip_IOCON_PinEnableOpenDrainMode(LPC_IOCON,IOCON_PIO28);
    
    */
    Chip_IOCON_PinSetMode(LPC_IOCON,IOCON_PIO0,PIN_MODE_INACTIVE);
    Chip_IOCON_PinSetMode(LPC_IOCON,IOCON_PIO1,PIN_MODE_INACTIVE);
    //Chip_IOCON_PinSetMode(LPC_IOCON,IOCON_PIO2,PIN_MODE_INACTIVE);
    //Chip_IOCON_PinSetMode(LPC_IOCON,IOCON_PIO3,PIN_MODE_INACTIVE);
    Chip_IOCON_PinSetMode(LPC_IOCON,IOCON_PIO4,PIN_MODE_INACTIVE);
    Chip_IOCON_PinSetMode(LPC_IOCON,IOCON_PIO5,PIN_MODE_INACTIVE);
    Chip_IOCON_PinSetMode(LPC_IOCON,IOCON_PIO6,PIN_MODE_INACTIVE);
    Chip_IOCON_PinSetMode(LPC_IOCON,IOCON_PIO7,PIN_MODE_INACTIVE);
    Chip_IOCON_PinSetMode(LPC_IOCON,IOCON_PIO8,PIN_MODE_INACTIVE);
    Chip_IOCON_PinSetMode(LPC_IOCON,IOCON_PIO9,PIN_MODE_INACTIVE);
    Chip_IOCON_PinSetMode(LPC_IOCON,IOCON_PIO10,PIN_MODE_INACTIVE);
    Chip_IOCON_PinSetMode(LPC_IOCON,IOCON_PIO11,PIN_MODE_INACTIVE);
    Chip_IOCON_PinSetMode(LPC_IOCON,IOCON_PIO12,PIN_MODE_INACTIVE);
    Chip_IOCON_PinSetMode(LPC_IOCON,IOCON_PIO13,PIN_MODE_INACTIVE);
    Chip_IOCON_PinSetMode(LPC_IOCON,IOCON_PIO14,PIN_MODE_INACTIVE);
    Chip_IOCON_PinSetMode(LPC_IOCON,IOCON_PIO15,PIN_MODE_INACTIVE);
    Chip_IOCON_PinSetMode(LPC_IOCON,IOCON_PIO16,PIN_MODE_INACTIVE);
    Chip_IOCON_PinSetMode(LPC_IOCON,IOCON_PIO17,PIN_MODE_INACTIVE);
    Chip_IOCON_PinSetMode(LPC_IOCON,IOCON_PIO18,PIN_MODE_INACTIVE);
    Chip_IOCON_PinSetMode(LPC_IOCON,IOCON_PIO19,PIN_MODE_INACTIVE);
    Chip_IOCON_PinSetMode(LPC_IOCON,IOCON_PIO20,PIN_MODE_INACTIVE);
    Chip_IOCON_PinSetMode(LPC_IOCON,IOCON_PIO21,PIN_MODE_INACTIVE);
    Chip_IOCON_PinSetMode(LPC_IOCON,IOCON_PIO22,PIN_MODE_INACTIVE);
    Chip_IOCON_PinSetMode(LPC_IOCON,IOCON_PIO23,PIN_MODE_INACTIVE);
    Chip_IOCON_PinSetMode(LPC_IOCON,IOCON_PIO24,PIN_MODE_INACTIVE);
    Chip_IOCON_PinSetMode(LPC_IOCON,IOCON_PIO25,PIN_MODE_INACTIVE);
    Chip_IOCON_PinSetMode(LPC_IOCON,IOCON_PIO26,PIN_MODE_INACTIVE);
    Chip_IOCON_PinSetMode(LPC_IOCON,IOCON_PIO27,PIN_MODE_INACTIVE);
    Chip_IOCON_PinSetMode(LPC_IOCON,IOCON_PIO28,PIN_MODE_INACTIVE);
}

/* Set up and initialize all required blocks and functions related to the
   board hardware */
void Board_Init(void)
{
	/* Initialize GPIO */
	Chip_GPIO_Init(LPC_GPIO_PORT);
	Board_GPIO_Init();
	/* Initialize the LEDs */
	Board_LED_Init();
        /* Sets up DEBUG UART */
	//DEBUGINIT();

}
