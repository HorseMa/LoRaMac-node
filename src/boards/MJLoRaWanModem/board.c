/*!
 * \file      board.c
 *
 * \brief     Target board general functions implementation
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
#include "delay.h"
#include "gpio.h"
#include "board-config.h"
#include "rtc-board.h"
#include "sx1276-board.h"
#if defined( USE_USB_CDC )
#include "uart-usb-board.h"
#endif
#include "board.h"
#include "timer.h"
/*!
 * Unique Devices IDs register set ( STM32L1xxx )
 */
#define         ID1                                 ( 0x1FF80050 )
#define         ID2                                 ( 0x1FF80054 )
#define         ID3                                 ( 0x1FF80064 )

/*!
 * Nested interrupt counter.
 *
 * \remark Interrupt should only be fully disabled once the value is 0
 */
static uint8_t IrqNestLevel = 0;

void BoardDisableIrq( void )
{
    __disable_irq( );
    IrqNestLevel++;
}

void BoardEnableIrq( void )
{
    IrqNestLevel--;
    if( IrqNestLevel == 0 )
    {
        __enable_irq( );
    }
}

void BoardInitMcu( void )
{
    SystemCoreClockUpdate();
    Board_Init();
    UartMcuInit(NULL,0,PA_1,PA_1);
    Chip_Clock_EnablePeriphClock(SYSCTL_CLOCK_SYS);
    SysTick_Config(SystemCoreClock / 1000);
    SpiInit(NULL,SPI_1,PA_1,PA_1,PA_0,PA_0);
}

void BoardResetMcu( void )
{
    BoardDisableIrq( );

    //Restart system
    NVIC_SystemReset( );

}

uint32_t BoardGetRandomSeed( void )
{
    uint32_t unique_id[4];
    Chip_IAP_ReadUID(unique_id);
    return ( unique_id[0] ^ unique_id[1] ^ unique_id[2] ^ unique_id[3] );
}

void BoardGetUniqueId( uint8_t *id )
{
    uint32_t unique_id[4];
    Chip_IAP_ReadUID(unique_id);
    memcpy(id,unique_id,8);
}

/*!
 * Factory power supply
 */
#define FACTORY_POWER_SUPPLY                        3300 // mV

/*!
 * Battery thresholds
 */
#define BATTERY_MAX_LEVEL                           4150 // mV
#define BATTERY_MIN_LEVEL                           3200 // mV
#define BATTERY_SHUTDOWN_LEVEL                      3100 // mV

static uint16_t BatteryVoltage = BATTERY_MAX_LEVEL;
TimerTime_t systick = 0;
TimerTime_t aLarmTimerTime = 0xffffffff;         // 上一次闹钟设置的超时时间
TimerTime_t sTartAlarmTimerTime = 0;    // 上一次闹钟设置的当前时刻


void SysTick_Handler( void )
{
    systick ++;
    if(TimerGetElapsedTime(sTartAlarmTimerTime) >= aLarmTimerTime)
    {
        TimerIrqHandler();
    }
    //HAL_IncTick( );
    //HAL_SYSTICK_IRQHandler( );
}


#ifdef USE_FULL_ASSERT
/*
 * Function Name  : assert_failed
 * Description    : Reports the name of the source file and the source line number
 *                  where the assert_param error has occurred.
 * Input          : - file: pointer to the source file name
 *                  - line: assert_param error line source number
 * Output         : None
 * Return         : None
 */
void assert_failed( uint8_t* file, uint32_t line )
{
    /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %lu\r\n", file, line) */

    printf( "Wrong parameters value: file %s on line %lu\r\n", ( const char* )file, line );
    /* Infinite loop */
    while( 1 )
    {
    }
}

void GpioWrite( Gpio_t *obj, uint32_t value )
{
  value?Chip_GPIO_SetPinOutHigh(LPC_GPIO_PORT,0,14):Chip_GPIO_SetPinOutLow(LPC_GPIO_PORT,0,14);
}
#endif
