/*!
 * \file      sx1276-board.c
 *
 * \brief     Target board SX1276 driver implementation
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
#include <stdlib.h>
#include "utilities.h"
#include "board-config.h"
#include "board.h"
#include "delay.h"
#include "radio.h"
#include "sx1276-board.h"
#include "lpc824board.h"
#include "sx1276.h"

/*!
 * Flag used to set the RF switch control pins in low power mode when the radio is not active.
 */
static bool RadioIsActive = false;

/*!
 * Radio driver structure initialization
 */
const struct Radio_s Radio =
{
    SX1276Init,
    SX1276GetStatus,
    SX1276SetModem,
    SX1276SetChannel,
    SX1276IsChannelFree,
    SX1276Random,
    SX1276SetRxConfig,
    SX1276SetTxConfig,
    SX1276CheckRfFrequency,
    SX1276GetTimeOnAir,
    SX1276Send,
    SX1276SetSleep,
    SX1276SetStby,
    SX1276SetRx,
    SX1276StartCad,
    NULL,//SX1276SetTxContinuousWave,
    SX1276ReadRssi,
    SX1276Write,
    SX1276Read,
    SX1276WriteBuffer,
    SX1276ReadBuffer,
    SX1276SetMaxPayloadLength,
    SX1276SetPublicNetwork,
    SX1276GetWakeupTime
};

/*!
 * Antenna switch GPIO pins objects
 */
Gpio_t AntSwitchLf;
Gpio_t AntSwitchHf;

void SX1276IoInit( void )
{
    Chip_Clock_EnablePeriphClock(SYSCTL_CLOCK_SWM);

    /* Configure interrupt channel 0 for the GPIO pin in SysCon block */
    //Chip_SWM_MovablePinAssign(SWM_SCT_IN0_I, 19);
    Chip_SYSCTL_SetPinInterrupt(0, 19);   // DIO0

    /* Configure channel 0 as wake up interrupt in SysCon block */
    Chip_SYSCTL_EnablePINTWakeup(0);

    /* Configure GPIO pin as input pin */
    Chip_GPIO_SetPinDIRInput(LPC_GPIO_PORT, 0, 19);

    /* Configure interrupt channel 0 for the GPIO pin in SysCon block */
    //Chip_SWM_MovablePinAssign(SWM_SCT_IN1_I, 18);
    Chip_SYSCTL_SetPinInterrupt(1, 18);   // DIO1

    /* Configure channel 0 as wake up interrupt in SysCon block */
    Chip_SYSCTL_EnablePINTWakeup(1);

    /* Configure GPIO pin as input pin */
    Chip_GPIO_SetPinDIRInput(LPC_GPIO_PORT, 0, 18);
    
    Chip_IOCON_PinDisableOpenDrainMode(LPC_IOCON,IOCON_PIO17);  // SDN
    Chip_GPIO_SetPinDIROutput(LPC_GPIO_PORT, 0, 17);
    
    Chip_GPIO_SetPinDIR(LPC_GPIO_PORT,0,23,TRUE); // NRESET
}

void SX1276IoIrqInit( DioIrqHandler **irqHandlers )
{
    /* Configure channel 0 interrupt as edge sensitive and falling edge interrupt */
    Chip_PININT_SetPinModeEdge(LPC_PININT, PININTCH0);
    Chip_PININT_EnableIntHigh(LPC_PININT, PININTCH0);

    /* Enable interrupt in the NVIC */
    NVIC_SetPriority (PININT0_IRQn, (1<<__NVIC_PRIO_BITS) - 1);
    NVIC_EnableIRQ(PININT0_IRQn);

    /* Configure channel 0 interrupt as edge sensitive and falling edge interrupt */
    Chip_PININT_SetPinModeEdge(LPC_PININT, PININTCH1);
    Chip_PININT_EnableIntHigh(LPC_PININT, PININTCH1);

    /* Enable interrupt in the NVIC */
    NVIC_SetPriority (PININT1_IRQn, (1<<__NVIC_PRIO_BITS) - 1);
    NVIC_EnableIRQ(PININT1_IRQn);
}

void SX1276IoDeInit( void )
{
}

/*!
 * \brief Enables/disables the TCXO if available on board design.
 *
 * \param [IN] state TCXO enabled when true and disabled when false.
 */
static void SX1276SetBoardTcxo( uint8_t state )
{
    // No TCXO component available on this board design.
#if 0
    if( state == true )
    {
        TCXO_ON( );
        DelayMs( BOARD_TCXO_WAKEUP_TIME );
    }
    else
    {
        TCXO_OFF( );
    }
#endif
}

uint32_t SX1276GetBoardTcxoWakeupTime( void )
{
    return BOARD_TCXO_WAKEUP_TIME;
}

void SX1276Reset( void )
{
    //Chip_GPIO_SetPinOutLow(LPC_GPIO_PORT,0,14);
    Chip_GPIO_SetPinOutLow(LPC_GPIO_PORT,0,23);
    DelayMs(1);
    //Chip_GPIO_SetPinOutHigh(LPC_GPIO_PORT,0,14);
    Chip_GPIO_SetPinOutHigh(LPC_GPIO_PORT,0,23);
    DelayMs(6);
}

void SX1276SetRfTxPower( int8_t power )
{
    uint8_t paConfig = 0;
    uint8_t paDac = 0;

    paConfig = SX1276Read( REG_PACONFIG );
    paDac = SX1276Read( REG_PADAC );

    paConfig = ( paConfig & RF_PACONFIG_PASELECT_MASK ) | SX1276GetPaSelect( SX1276.Settings.Channel );
    paConfig = ( paConfig & RF_PACONFIG_MAX_POWER_MASK ) | 0x70;

    if( ( paConfig & RF_PACONFIG_PASELECT_PABOOST ) == RF_PACONFIG_PASELECT_PABOOST )
    {
        if( power > 17 )
        {
            paDac = ( paDac & RF_PADAC_20DBM_MASK ) | RF_PADAC_20DBM_ON;
        }
        else
        {
            paDac = ( paDac & RF_PADAC_20DBM_MASK ) | RF_PADAC_20DBM_OFF;
        }
        if( ( paDac & RF_PADAC_20DBM_ON ) == RF_PADAC_20DBM_ON )
        {
            if( power < 5 )
            {
                power = 5;
            }
            if( power > 20 )
            {
                power = 20;
            }
            paConfig = ( paConfig & RF_PACONFIG_OUTPUTPOWER_MASK ) | ( uint8_t )( ( uint16_t )( power - 5 ) & 0x0F );
        }
        else
        {
            if( power < 2 )
            {
                power = 2;
            }
            if( power > 17 )
            {
                power = 17;
            }
            paConfig = ( paConfig & RF_PACONFIG_OUTPUTPOWER_MASK ) | ( uint8_t )( ( uint16_t )( power - 2 ) & 0x0F );
        }
    }
    else
    {
        if( power < -1 )
        {
            power = -1;
        }
        if( power > 14 )
        {
            power = 14;
        }
        paConfig = ( paConfig & RF_PACONFIG_OUTPUTPOWER_MASK ) | ( uint8_t )( ( uint16_t )( power + 1 ) & 0x0F );
    }
    SX1276Write( REG_PACONFIG, paConfig );
    SX1276Write( REG_PADAC, paDac );
}

uint8_t SX1276GetPaSelect( uint32_t channel )
{
    if( channel < RF_MID_BAND_THRESH )
    {
        return RF_PACONFIG_PASELECT_PABOOST;
    }
    else
    {
        return RF_PACONFIG_PASELECT_RFO;
    }
}

void SX1276SetAntSwLowPower( bool status )
{
    if( RadioIsActive != status )
    {
        RadioIsActive = status;

        if( status == false )
        {
            SX1276SetBoardTcxo( true );
            SX1276AntSwInit( );
        }
        else
        {
            SX1276SetBoardTcxo( false );
            SX1276AntSwDeInit( );
        }
    }
}

void SX1276AntSwInit( void )
{
    Chip_IOCON_PinSetMode(LPC_IOCON,IOCON_PIO17,PIN_MODE_REPEATER);
    Chip_GPIO_SetPinDIROutput(LPC_GPIO_PORT, 0, 17);
}

void SX1276AntSwDeInit( void )
{
    Chip_IOCON_PinSetMode(LPC_IOCON,IOCON_PIO17,PIN_MODE_INACTIVE);
    Chip_GPIO_SetPinOutLow(LPC_GPIO_PORT,0,17);
}

void SX1276SetAntSw( uint8_t opMode )
{
    switch( opMode )
    {
    case RFLR_OPMODE_TRANSMITTER:
        Chip_GPIO_SetPinOutLow(LPC_GPIO_PORT,0,17);
        break;
    case RFLR_OPMODE_RECEIVER:
    case RFLR_OPMODE_RECEIVER_SINGLE:
    case RFLR_OPMODE_CAD:
    default:
        Chip_GPIO_SetPinOutHigh(LPC_GPIO_PORT,0,17);
        break;
    }
}

bool SX1276CheckRfFrequency( uint32_t frequency )
{
    // Implement check. Currently all frequencies are supported
    return true;
}
/*!
 * \brief DIO 0 IRQ callback
 */
void SX1276OnDio0Irq( void );

/*!
 * \brief DIO 1 IRQ callback
 */
void SX1276OnDio1Irq( void );

/*!
 * \brief DIO 2 IRQ callback
 */
void SX1276OnDio2Irq( void );

/*!
 * \brief DIO 3 IRQ callback
 */
void SX1276OnDio3Irq( void );

/*!
 * \brief DIO 4 IRQ callback
 */
void SX1276OnDio4Irq( void );

/*!
 * \brief DIO 5 IRQ callback
 */
void SX1276OnDio5Irq( void );

void PININT0_IRQHandler( void )
{
    //BoardDisableIrq( );
    Chip_PININT_ClearIntStatus(LPC_PININT, PININTCH0);
    SX1276OnDio0Irq();
    //BoardEnableIrq( );
}

void PININT1_IRQHandler( void )
{
    //BoardDisableIrq( );
    Chip_PININT_ClearIntStatus(LPC_PININT, PININTCH1);
    SX1276OnDio1Irq();
    //BoardEnableIrq( );
}
#if 0
void PININT2_IRQHandler( void )
{
    SX1276OnDio2Irq();
}

void PININT3_IRQHandler( void )
{
    SX1276OnDio3Irq();
}

void PININT4_IRQHandler( void )
{
    SX1276OnDio4Irq();
}

void PININT5_IRQHandler( void )
{
    SX1276OnDio5Irq();
}
#endif