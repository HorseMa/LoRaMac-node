/*!
 * \file      timer.c
 *
 * \brief     Timer objects and scheduling management implementation
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
#include "rtc-board.h"
#include "timer.h"
#include "modem.h"
#include "LoRaMac.h"

/*!
 * This flag is used to loop through the main several times in order to be sure
 * that all pending events have been processed.
 */
volatile uint8_t HasLoopedThroughMain = 0;

/*!
 * Timers list head pointer
 */
static TimerEvent_t *TimerListHead = NULL;

/*!
 * \brief Adds or replace the head timer of the list.
 *
 * \remark The list is automatically sorted. The list head always contains the
 *         next timer to expire.
 *
 * \param [IN]  obj Timer object to be become the new head
 * \param [IN]  remainingTime Remaining time of the previous head to be replaced
 */
static void TimerInsertNewHeadTimer( TimerEvent_t *obj, uint32_t remainingTime );

/*!
 * \brief Adds a timer to the list.
 *
 * \remark The list is automatically sorted. The list head always contains the
 *         next timer to expire.
 *
 * \param [IN]  obj Timer object to be added to the list
 * \param [IN]  remainingTime Remaining time of the running head after which the object may be added
 */
static void TimerInsertTimer( TimerEvent_t *obj, uint32_t remainingTime );

/*!
 * \brief Sets a timeout with the duration "timestamp"
 *
 * \param [IN] timestamp Delay duration
 */
static void TimerSetTimeout( TimerEvent_t *obj );

/*!
 * \brief Check if the Object to be added is not already in the list
 *
 * \param [IN] timestamp Delay duration
 * \retval true (the object is already in the list) or false
 */
static bool TimerExists( TimerEvent_t *obj );
extern TimerTime_t systick;
extern TimerTime_t aLarmTimerTime;
extern TimerTime_t sTartAlarmTimerTime;

/*!
 * \brief Read the timer value of the currently running timer
 *
 * \retval value current timer value
 */
TimerTime_t TimerGetValue( void );

void TimerInit( TimerEvent_t *obj, void ( *callback )( void ) )
{
    obj->Timestamp = 0;
    obj->ReloadValue = 0;
    obj->IsRunning = false;
    obj->Callback = callback;
    obj->Next = NULL;
}

void TimerStart( TimerEvent_t *obj )
{
    uint32_t elapsedTime = 0;
    uint32_t remainingTime = 0;

    BoardDisableIrq( );

    if( ( obj == NULL ) || ( TimerExists( obj ) == true ) )
    {
        BoardEnableIrq( );
        return;
    }

    obj->Timestamp = obj->ReloadValue;
    obj->IsRunning = false;

    if( TimerListHead == NULL )
    {
        TimerInsertNewHeadTimer( obj, obj->Timestamp );
    }
    else
    {
        if( TimerListHead->IsRunning == true )
        {
            elapsedTime = TimerGetValue( );
            if( elapsedTime > TimerListHead->Timestamp )
            {
                elapsedTime = TimerListHead->Timestamp; // security but should never occur
            }
            remainingTime = TimerListHead->Timestamp - elapsedTime;
        }
        else
        {
            remainingTime = TimerListHead->Timestamp;
        }

        if( obj->Timestamp < remainingTime )
        {
            TimerInsertNewHeadTimer( obj, remainingTime );
        }
        else
        {
             TimerInsertTimer( obj, remainingTime );
        }
    }
    BoardEnableIrq( );
}

static void TimerInsertTimer( TimerEvent_t *obj, uint32_t remainingTime )
{
    uint32_t aggregatedTimestamp = 0;      // hold the sum of timestamps
    uint32_t aggregatedTimestampNext = 0;  // hold the sum of timestamps up to the next event

    TimerEvent_t* prev = TimerListHead;
    TimerEvent_t* cur = TimerListHead->Next;

    if( cur == NULL )
    { // obj comes just after the head
        obj->Timestamp -= remainingTime;
        prev->Next = obj;
        obj->Next = NULL;
    }
    else
    {
        aggregatedTimestamp = remainingTime;
        aggregatedTimestampNext = remainingTime + cur->Timestamp;

        while( prev != NULL )
        {
            if( aggregatedTimestampNext > obj->Timestamp )
            {
                obj->Timestamp -= aggregatedTimestamp;
                if( cur != NULL )
                {
                    cur->Timestamp -= obj->Timestamp;
                }
                prev->Next = obj;
                obj->Next = cur;
                break;
            }
            else
            {
                prev = cur;
                cur = cur->Next;
                if( cur == NULL )
                { // obj comes at the end of the list
                    aggregatedTimestamp = aggregatedTimestampNext;
                    obj->Timestamp -= aggregatedTimestamp;
                    prev->Next = obj;
                    obj->Next = NULL;
                    break;
                }
                else
                {
                    aggregatedTimestamp = aggregatedTimestampNext;
                    aggregatedTimestampNext = aggregatedTimestampNext + cur->Timestamp;
                }
            }
        }
    }
}

static void TimerInsertNewHeadTimer( TimerEvent_t *obj, uint32_t remainingTime )
{
    TimerEvent_t* cur = TimerListHead;

    if( cur != NULL )
    {
        cur->Timestamp = remainingTime - obj->Timestamp;
        cur->IsRunning = false;
    }

    obj->Next = cur;
    obj->IsRunning = true;
    TimerListHead = obj;
    TimerSetTimeout( TimerListHead );
}

void TimerIrqHandler( void )
{
    uint32_t elapsedTime = 0;

    // Early out when TimerListHead is null to prevent null pointer
    if ( TimerListHead == NULL )
    {
        return;
    }

    elapsedTime = TimerGetValue( );

    if( elapsedTime >= TimerListHead->Timestamp )
    {
        TimerListHead->Timestamp = 0;
    }
    else
    {
        TimerListHead->Timestamp -= elapsedTime;
    }

    TimerListHead->IsRunning = false;

    while( ( TimerListHead != NULL ) && ( TimerListHead->Timestamp == 0 ) )
    {
        TimerEvent_t* elapsedTimer = TimerListHead;
        TimerListHead = TimerListHead->Next;

        if( elapsedTimer->Callback != NULL )
        {
            elapsedTimer->Callback( );
        }
    }

    // start the next TimerListHead if it exists
    if( TimerListHead != NULL )
    {
        if( TimerListHead->IsRunning != true )
        {
            TimerListHead->IsRunning = true;
            TimerSetTimeout( TimerListHead );
        }
    }
}

void TimerStop( TimerEvent_t *obj )
{
    BoardDisableIrq( );

    uint32_t elapsedTime = 0;
    uint32_t remainingTime = 0;

    TimerEvent_t* prev = TimerListHead;
    TimerEvent_t* cur = TimerListHead;

    // List is empty or the Obj to stop does not exist
    if( ( TimerListHead == NULL ) || ( obj == NULL ) )
    {
        BoardEnableIrq( );
        return;
    }

    if( TimerListHead == obj ) // Stop the Head
    {
        if( TimerListHead->IsRunning == true ) // The head is already running
        {
            elapsedTime = TimerGetValue( );
            if( elapsedTime > obj->Timestamp )
            {
                elapsedTime = obj->Timestamp;
            }

            remainingTime = obj->Timestamp - elapsedTime;

            TimerListHead->IsRunning = false;
            if( TimerListHead->Next != NULL )
            {
                TimerListHead = TimerListHead->Next;
                TimerListHead->Timestamp += remainingTime;
                TimerListHead->IsRunning = true;
                TimerSetTimeout( TimerListHead );
            }
            else
            {
                TimerListHead = NULL;
            }
        }
        else // Stop the head before it is started
        {
            if( TimerListHead->Next != NULL )
            {
                remainingTime = obj->Timestamp;
                TimerListHead = TimerListHead->Next;
                TimerListHead->Timestamp += remainingTime;
            }
            else
            {
                TimerListHead = NULL;
            }
        }
    }
    else // Stop an object within the list
    {
        remainingTime = obj->Timestamp;

        while( cur != NULL )
        {
            if( cur == obj )
            {
                if( cur->Next != NULL )
                {
                    cur = cur->Next;
                    prev->Next = cur;
                    cur->Timestamp += remainingTime;
                }
                else
                {
                    cur = NULL;
                    prev->Next = cur;
                }
                break;
            }
            else
            {
                prev = cur;
                cur = cur->Next;
            }
        }
    }
    BoardEnableIrq( );
}

static bool TimerExists( TimerEvent_t *obj )
{
    TimerEvent_t* cur = TimerListHead;

    while( cur != NULL )
    {
        if( cur == obj )
        {
            return true;
        }
        cur = cur->Next;
    }
    return false;
}

void TimerReset( TimerEvent_t *obj )
{
    TimerStop( obj );
    TimerStart( obj );
}

void TimerSetValue( TimerEvent_t *obj, uint32_t value )
{
    TimerStop( obj );
    obj->Timestamp = value;
    obj->ReloadValue = value;
}

TimerTime_t TimerGetValue( void )
{
    return TimerGetElapsedTime(sTartAlarmTimerTime);
    //return RtcGetElapsedAlarmTime( );
}

TimerTime_t TimerGetCurrentTime( void )
{
    return systick;
}

TimerTime_t TimerGetElapsedTime( TimerTime_t savedTime )
{
    TimerTime_t elapsedTime = 0;

    // Needed at boot, cannot compute with 0 or elapsed time will be equal to current time
    /*if( savedTime == 0 )
    {
        return 0;
    }*/

    elapsedTime = TimerGetCurrentTime();

    if( elapsedTime < savedTime )
    { // roll over of the counter
        return( elapsedTime + ( 0xFFFFFFFF - savedTime ) );
    }
    else
    {
        return( elapsedTime - savedTime );
    }
}

/*TimerTime_t TimerGetFutureTime( TimerTime_t eventInFuture )
{
    return RtcComputeFutureEventTime( eventInFuture );
}*/

static void TimerSetTimeout( TimerEvent_t *obj )
{
    aLarmTimerTime = obj->Timestamp;
    sTartAlarmTimerTime = TimerGetCurrentTime();
#if 0
    HasLoopedThroughMain = 0;
    obj->Timestamp = RtcGetAdjustedTimeoutValue( obj->Timestamp );
    RtcSetTimeout( obj->Timestamp );
#endif
}
extern RINGBUFF_T txring, rxring;
TimerTime_t uartflashtimer;
uint8_t alarmsendflag = true;
void TimerLowPowerHandler( void )
{
    uint8_t byte;
    TimerTime_t macflashtimer;
    Chip_WWDT_Feed(LPC_WWDT);
    while(Chip_UART_ReadRB(LPC_USART0, &rxring, &byte, 1) > 0)
    {
        Chip_WWDT_Feed(LPC_WWDT);
        uartflashtimer = TimerGetCurrentTime();
        frame_rx(byte);
    }
    extern bool bitNeedAck;
    extern TimerTime_t bitNeedAckTimer;
    if(bitNeedAck)
    {
        alarmsendflag = false;
        if(TimerGetElapsedTime(bitNeedAckTimer) > 500)
        {
            uint8_t sendlen = 1;
            uint8_t senddata[1];
            senddata[0] = Board_LED_Get(0);
            modemSendFrame(1,senddata,sendlen,false);
            //Chip_UART_SendRB(LPC_USART0, &txring, "ack\r\n", 5);
        }
        return;
    }

    if(persist.nodetype == CLASS_C)
    {
        return;
    }
    if((!RingBuffer_IsEmpty(&txring)) || (!RingBuffer_IsEmpty(&rxring)))
    {
        uartflashtimer = TimerGetCurrentTime();
    }

    extern TimerEvent_t TxDelayedTimer;
    extern TimerEvent_t RxWindowTimer1;
    extern TimerEvent_t RxWindowTimer2;
    extern TimerEvent_t AckTimeoutTimer;


    extern uint32_t LoRaMacState;
    if( TimerExists(&RxWindowTimer1) || TimerExists(&RxWindowTimer2) || TimerExists(&TxDelayedTimer) || TimerExists(&AckTimeoutTimer) || ( Radio.GetStatus() == RF_RX_RUNNING ) || ( Radio.GetStatus() == RF_TX_RUNNING ) || (LoRaMacState != 0))
    {
        return;
    }
    if( persist.flags & FLAGS_JOINPAR )
    {
        return;
    }
    /*extern uint32_t LoRaMacState;
    if(LoRaMacState != 0)
    {
        macflashtimer = TimerGetCurrentTime();
    }*/
    if((alarmsendflag == true) && ( persist.flags & FLAGS_SESSPAR ))
    {
        uint8_t sendlen = 1;
        uint8_t senddata[1];
        senddata[0] = Board_LED_Get(0);
        //Chip_UART_SendRB(LPC_USART0, &txring, "alarm\r\n", 7);
        if(modemSendFrame(1,senddata,sendlen,true))
        {
            alarmsendflag = false;
        }
        return;
    }
    if(TimerGetElapsedTime(uartflashtimer) > 10)// && (TimerGetElapsedTime(macflashtimer) > 10))
    {
        //BoardDisableIrq( );
        Radio.Sleep( );
        //DelayMs(10);
        /*Chip_Clock_EnablePeriphClock(SYSCTL_CLOCK_SWM);
        //Chip_SWM_EnableFixedPin(SWM_FIXED_ADC11);
        Chip_SWM_DisableFixedPin(SWM_FIXED_ADC11);
        Chip_SWM_MovablePinAssign(SWM_U0_RXD_I, 24);
        Chip_Clock_DisablePeriphClock(SYSCTL_CLOCK_SWM);
        */
        Chip_PMU_SetPowerDownControl(LPC_PMU, PMU_DPDCTRL_WAKEUPPHYS);
        if(persist.sesspar.alarm)
        {
            Chip_PMU_SetPowerDownControl(LPC_PMU, PMU_DPDCTRL_LPOSCEN | PMU_DPDCTRL_LPOSCDPDEN);
        }
        WakeupTest(WKT_CLKSRC_10KHZ,persist.sesspar.alarm,PMU_MCU_DEEP_PWRDOWN);
        //WakeupTest(WKT_CLKSRC_10KHZ,persist.sesspar.alarm,PMU_MCU_POWER_DOWN);
        Chip_PMU_ClearPowerDownControl(LPC_PMU, PMU_DPDCTRL_WAKEUPPHYS);
        Chip_Clock_EnablePeriphClock(SYSCTL_CLOCK_SWM);
        Chip_SWM_DisableFixedPin(SWM_FIXED_ADC11);
        Chip_SWM_MovablePinAssign(SWM_U0_RXD_I, 4);
        Chip_Clock_DisablePeriphClock(SYSCTL_CLOCK_SWM);

        //modem_wwdt_init();
        //while(1);
        alarmsendflag = true;

        //BoardEnableIrq( );
        //WakeupTest(WKT_CLKSRC_10KHZ,persist.sesspar.alarm,PMU_MCU_SLEEP);
    }
}

void TimerProcess( void )
{
    RtcProcess( );
}
