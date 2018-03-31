/*!
 * \file      gpio-board.c
 *
 * \brief     Target board GPIO driver implementation
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
#include "board.h"
#include "utilities.h"
#include "board-config.h"
#include "rtc-board.h"
#include "gpio-board.h"
#if defined( BOARD_IOE_EXT )
#include "gpio-ioe.h"
#endif

static GpioIrqHandler *GpioIrq[16];

void GpioWrite( Gpio_t *obj, uint32_t value )
{
  value?Chip_GPIO_SetPinOutHigh(LPC_GPIO_PORT,0,14):Chip_GPIO_SetPinOutLow(LPC_GPIO_PORT,0,14);
}

void GpioInit( Gpio_t *obj, PinNames pin, PinModes mode, PinConfigs config, PinTypes type, uint32_t value )
{
}
void GpioMcuInit( Gpio_t *obj, PinNames pin, PinModes mode, PinConfigs config, PinTypes type, uint32_t value )
{

}

void GpioMcuSetInterrupt( Gpio_t *obj, IrqModes irqMode, IrqPriorities irqPriority, GpioIrqHandler *irqHandler )
{

}

void GpioMcuRemoveInterrupt( Gpio_t *obj )
{
}

void GpioMcuWrite( Gpio_t *obj, uint32_t value )
{

}

void GpioMcuToggle( Gpio_t *obj )
{

}


void EXTI0_IRQHandler( void )
{
}

void EXTI1_IRQHandler( void )
{
}

void EXTI2_IRQHandler( void )
{
}

void EXTI3_IRQHandler( void )
{
}

void EXTI4_IRQHandler( void )
{
}

void EXTI9_5_IRQHandler( void )
{
}

void EXTI15_10_IRQHandler( void )
{
}

void HAL_GPIO_EXTI_Callback( uint16_t gpioPin )
{

}
