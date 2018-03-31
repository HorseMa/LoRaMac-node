/*!
 * \file      rtc-board.c
 *
 * \brief     Target board RTC timer and low power modes management
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
#include <math.h>
#include "lpc824board.h"
#include "utilities.h"
#include "board.h"
#include "timer.h"
#include "gpio.h"
#include "rtc-board.h"

/*!
 * RTC Time base in ms
 */
#define RTC_ALARM_TICK_DURATION                     0.48828125      // 1 tick every 488us
#define RTC_ALARM_TICK_PER_MS                       2.048           // 1/2.048 = tick duration in ms

/*!
 * Maximum number of days (with some margin) that can be handled by the RTC alarm counter before overflow.
 */
#define RTC_ALARM_MAX_NUMBER_OF_DAYS                27


/*!
 * Flag used to indicates a the MCU has waken-up from an external IRQ
 */
volatile bool NonScheduledWakeUp = false;

/*!
 * \brief Hold the Wake-up time duration in ms
 */
volatile uint32_t McuWakeUpTime = 0;

void RtcInit( void )
{

}

void RtcSetTimeout( uint32_t timeout )
{
    //RtcStartWakeUpAlarm( timeout );
}

TimerTime_t RtcGetElapsedAlarmTime( void )
{
    TimerTime_t currentTime = 0;
    TimerTime_t contextTime = 0;

    //currentTime = RtcConvertCalendarTickToTimerTime( NULL );
    //contextTime = RtcConvertCalendarTickToTimerTime( &RtcCalendarContext );

    if( currentTime < contextTime )
    {
        return( currentTime + ( 0xFFFFFFFF - contextTime ) );
    }
    else
    {
        return( currentTime - contextTime );
    }
}

TimerTime_t RtcGetTimerValue( void )
{
    return 0;//( RtcConvertCalendarTickToTimerTime( NULL ) );
    //RtcStartWakeUpAlarm( timeout );
}

TimerTime_t RtcComputeFutureEventTime( TimerTime_t futureEventInTime )
{
    return( RtcGetTimerValue( ) + futureEventInTime );
}

TimerTime_t RtcComputeElapsedTime( TimerTime_t eventInTime )
{
    TimerTime_t elapsedTime = 0;

    // Needed at boot, cannot compute with 0 or elapsed time will be equal to current time
    if( eventInTime == 0 )
    {
        return 0;
    }

   // elapsedTime = RtcConvertCalendarTickToTimerTime( NULL );

    if( elapsedTime < eventInTime )
    { // roll over of the counter
        return( elapsedTime + ( 0xFFFFFFFF - eventInTime ) );
    }
    else
    {
        return( elapsedTime - eventInTime );
    }
}

void RtcEnterLowPowerStopMode( void )
{
}

/*!
 * \brief RTC IRQ Handler of the RTC Alarm
 */
void RTC_Alarm_IRQHandler( void )
{
    TimerIrqHandler( );
}

void RtcProcess( void )
{
    // Not used on this platform.
}
