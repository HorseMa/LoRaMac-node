/*!
 * \file      main.c
 *
 * \brief     LoRaMac classA device implementation
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

/*! \file classA/SensorNode/main.c */

#include "utilities.h"
#include "board.h"
#include "lpc824board.h"
#include "LoRaMac.h"
#include "Commissioning.h"
#include "modem.h"

#define ACTIVE_REGION LORAMAC_REGION_CN470
#ifndef ACTIVE_REGION

#warning "No active region defined, LORAMAC_REGION_EU868 will be used as default."

#define ACTIVE_REGION LORAMAC_REGION_EU868

#endif

/*!
 * Defines the application data transmission duty cycle. 5s, value in [ms].
 */
#define APP_TX_DUTYCYCLE                            5000

/*!
 * Defines a random delay for application data transmission duty cycle. 1s,
 * value in [ms].
 */
#define APP_TX_DUTYCYCLE_RND                        1000

/*!
 * Default datarate
 */
#define LORAWAN_DEFAULT_DATARATE                    DR_0

/*!
 * LoRaWAN confirmed messages
 */
#define LORAWAN_CONFIRMED_MSG_ON                    false

/*!
 * LoRaWAN Adaptive Data Rate
 *
 * \remark Please note that when ADR is enabled the end-device should be static
 */
#define LORAWAN_ADR_ON                              0

#if defined( REGION_EU868 )

#include "LoRaMacTest.h"

/*!
 * LoRaWAN ETSI duty cycle control enable/disable
 *
 * \remark Please note that ETSI mandates duty cycled transmissions. Use only for test purposes
 */
#define LORAWAN_DUTYCYCLE_ON                        true

#endif

/*!
 * LoRaWAN application port
 */
#define LORAWAN_APP_PORT                            1

static uint8_t DevEui[] = LORAWAN_DEVICE_EUI;
static uint8_t AppEui[] = LORAWAN_APPLICATION_EUI;
static uint8_t AppKey[] = LORAWAN_APPLICATION_KEY;

#if( OVER_THE_AIR_ACTIVATION == 0 )

static uint8_t NwkSKey[] = LORAWAN_NWKSKEY;
static uint8_t AppSKey[] = LORAWAN_APPSKEY;

/*!
 * Device address
 */
static uint32_t DevAddr = LORAWAN_DEVICE_ADDRESS;

#endif

/*!
 * Application port
 */
static uint8_t AppPort = LORAWAN_APP_PORT;

/*!
 * User application data size
 */
static uint8_t AppDataSize = 16;
static uint8_t AppDataSizeBackup = 16;
/*!
 * User application data buffer size
 */
#define LORAWAN_APP_DATA_MAX_SIZE                           242

/*!
 * User application data
 */
static uint8_t AppData[LORAWAN_APP_DATA_MAX_SIZE];

/*!
 * Indicates if the node is sending confirmed or unconfirmed messages
 */
static uint8_t IsTxConfirmed = LORAWAN_CONFIRMED_MSG_ON;
static uint8_t IsLastTxConfirmed = LORAWAN_CONFIRMED_MSG_ON;
bool bitNeedAck = false;
TimerTime_t bitNeedAckTimer = 0;
/*!
 * Defines the application data transmission duty cycle
 */
static uint32_t TxDutyCycleTime = APP_TX_DUTYCYCLE;

/*!
 * Timer to handle the application data transmission duty cycle
 */
static TimerEvent_t TxNextPacketTimer;

/*!
 * Specifies the state of the application LED
 */
static bool AppLedStateOn = false;

uint8_t enableChannelsDRnum = 0;
uint8_t enableChannelDR[6] = {0};
/*!
 * Timer to handle the state of LED1
 */
//static TimerEvent_t Led1Timer;

/*!
 * Timer to handle the state of LED2
 */
//static TimerEvent_t Led2Timer;

/*!
 * Timer to handle the state of LED4
 */
//static TimerEvent_t Led4Timer;

/*!
 * Indicates if a new packet can be sent
 */
static bool NextTx = true;

/*!
 * Device states
 */
enum eDeviceState
{
    DEVICE_STATE_INIT,
    DEVICE_STATE_JOIN,
    DEVICE_STATE_SEND,
    DEVICE_STATE_CYCLE,
    DEVICE_STATE_SLEEP
}DeviceState;

/*!
 * LoRaWAN compliance tests support data
 */
struct ComplianceTest_s
{
    bool Running;
    uint8_t State;
    bool IsTxConfirmed;
    uint8_t AppPort;
    uint8_t AppDataSize;
    uint8_t *AppDataBuffer;
    uint16_t DownLinkCounter;
    bool LinkCheck;
    uint8_t DemodMargin;
    uint8_t NbGateways;
}ComplianceTest;

/*!
 * \brief   Prepares the payload of the frame
 */
static void PrepareTxFrame( uint8_t port )
{
    const LoRaMacRegion_t region = ACTIVE_REGION;

    switch( port )
    {
    case 1:
        AppDataSize = 1;
        AppData[0] = Board_LED_Get(0);
        break;
    case 224:
        if( ComplianceTest.LinkCheck == true )
        {
            ComplianceTest.LinkCheck = false;
            AppDataSize = 3;
            AppData[0] = 5;
            AppData[1] = ComplianceTest.DemodMargin;
            AppData[2] = ComplianceTest.NbGateways;
            ComplianceTest.State = 1;
        }
        else
        {
            switch( ComplianceTest.State )
            {
            case 4:
                ComplianceTest.State = 1;
                break;
            case 1:
                AppDataSize = 2;
                AppData[0] = ComplianceTest.DownLinkCounter >> 8;
                AppData[1] = ComplianceTest.DownLinkCounter;
                break;
            }
        }
        break;
    default:
        break;
    }
}

/*!
 * \brief   Prepares the payload of the frame
 *
 * \retval  [0: frame could be send, 1: error]
 */
static bool SendFrame( void )
{
    McpsReq_t mcpsReq;
    LoRaMacTxInfo_t txInfo;
    
    if( LoRaMacQueryTxPossible( AppDataSize, &txInfo ) != LORAMAC_STATUS_OK )
    {
        // Send empty frame in order to flush MAC commands
        mcpsReq.Type = MCPS_UNCONFIRMED;
        mcpsReq.Req.Unconfirmed.fBuffer = NULL;
        mcpsReq.Req.Unconfirmed.fBufferSize = 0;
        mcpsReq.Req.Unconfirmed.Datarate = persist.sesspar.JoinRequestTrials;
    }
    else
    {
        if( IsTxConfirmed == false )
        {
            mcpsReq.Type = MCPS_UNCONFIRMED;
            mcpsReq.Req.Unconfirmed.fPort = AppPort;
            mcpsReq.Req.Unconfirmed.fBuffer = AppData;
            mcpsReq.Req.Unconfirmed.fBufferSize = AppDataSize;
            mcpsReq.Req.Unconfirmed.Datarate = persist.sesspar.JoinRequestTrials;
        }
        else
        {
            mcpsReq.Type = MCPS_CONFIRMED;
            mcpsReq.Req.Confirmed.fPort = AppPort;
            mcpsReq.Req.Confirmed.fBuffer = AppData;
            mcpsReq.Req.Confirmed.fBufferSize = AppDataSize;
            mcpsReq.Req.Confirmed.NbTrials = enableChannelsDRnum * 2;
            mcpsReq.Req.Confirmed.Datarate = persist.sesspar.JoinRequestTrials;
        }
    }

    if( LoRaMacMcpsRequest( &mcpsReq ) == LORAMAC_STATUS_OK )
    {
        return false;
    }
    return true;
}

bool modemSendFrame(uint8_t port,uint8_t *data,uint8_t len,bool confirm)
{
    MibRequestConfirm_t mibReq;
    LoRaMacStatus_t status;

    TimerStop( &TxNextPacketTimer );
    mibReq.Type = MIB_NETWORK_JOINED;
    status = LoRaMacMibGetRequestConfirm( &mibReq );

    if( status == LORAMAC_STATUS_OK )
    {
        if( mibReq.Param.IsNetworkJoined == true )
        {
            //DeviceState = DEVICE_STATE_SEND;
            NextTx = true;
        }
        else
        {
            return false;
        }
    }
    else
    {
        return false;
    }
    if( NextTx == true )
    {
        //PrepareTxFrame( port );
        IsTxConfirmed = confirm;
        AppPort = port;
        AppDataSize = len;
        memcpy(AppData,data,len);
        NextTx = SendFrame( );
    }
    if((persist.nodetype == CLASS_C) && (persist.sesspar.alarm))
    {
        DEBUG_OUTPUT("TxDutyCycleTime = %d\r\n",TxDutyCycleTime);
        TxDutyCycleTime = persist.sesspar.alarm * 1000 + randr(-100, 100);
        DeviceState = DEVICE_STATE_CYCLE;
    }
    return !NextTx;
}
/*!
 * \brief Function executed on TxNextPacket Timeout event
 */
void OnTxNextPacketTimerEvent( void )
{
    MibRequestConfirm_t mibReq;
    LoRaMacStatus_t status;

    TimerStop( &TxNextPacketTimer );

    mibReq.Type = MIB_NETWORK_JOINED;
    status = LoRaMacMibGetRequestConfirm( &mibReq );

    if( status == LORAMAC_STATUS_OK )
    {
        if( mibReq.Param.IsNetworkJoined == true )
        {
            DeviceState = DEVICE_STATE_SEND;
            NextTx = true;
        }
        else
        {
            // Network not joined yet. Try to join again
            MlmeReq_t mlmeReq;
            mlmeReq.Type = MLME_JOIN;
            mlmeReq.Req.Join.DevEui = DevEui;
            mlmeReq.Req.Join.AppEui = AppEui;
            mlmeReq.Req.Join.AppKey = AppKey;
            mibReq.Type = MIB_CHANNELS_DEFAULT_DATARATE;
            status = LoRaMacMibGetRequestConfirm( &mibReq );
            if( status == LORAMAC_STATUS_OK )
            {
                mlmeReq.Req.Join.Datarate = mibReq.Param.ChannelsDefaultDatarate;
            }
            else
            {
                mlmeReq.Req.Join.Datarate = LORAWAN_DEFAULT_DATARATE;
            }

            if( LoRaMacMlmeRequest( &mlmeReq ) == LORAMAC_STATUS_OK )
            {
                DeviceState = DEVICE_STATE_SLEEP;
            }
            else
            {
                DeviceState = DEVICE_STATE_CYCLE;
            }
        }
    }
}

/*!
 * \brief   MCPS-Confirm event function
 *
 * \param   [IN] mcpsConfirm - Pointer to the confirm structure,
 *               containing confirm attributes.
 */
static void McpsConfirm( McpsConfirm_t *mcpsConfirm )
{
    if( mcpsConfirm->Status == LORAMAC_EVENT_INFO_STATUS_OK )
    {
        switch( mcpsConfirm->McpsRequest )
        {
            case MCPS_UNCONFIRMED:
            {
                IsLastTxConfirmed = false;
                // Check Datarate
                // Check TxPower
                break;
            }
            case MCPS_CONFIRMED:
            {
                IsLastTxConfirmed = true;
                // Check Datarate
                // Check TxPower
                // Check AckReceived
                // Check NbTrials
                break;
            }
            case MCPS_PROPRIETARY:
            {
                break;
            }
            default:
                break;
        }

        // Switch LED 1 ON
        //GpioWrite( &Led1, 0 );
        //TimerStart( &Led1Timer );
    }
    else
    {
        if(mcpsConfirm->Status == LORAMAC_EVENT_INFO_STATUS_RX2_TIMEOUT)
        {
            LMIC.txrxFlags = 0;
            LMIC.dataLen = 0;
            onEvent(EV_TXCOMPLETE);
        }
    }
    NextTx = true;
}

/*!
 * \brief   MCPS-Indication event function
 *
 * \param   [IN] mcpsIndication - Pointer to the indication structure,
 *               containing indication attributes.
 */
static void McpsIndication( McpsIndication_t *mcpsIndication )
{
    if( mcpsIndication->Status != LORAMAC_EVENT_INFO_STATUS_OK )
    {
        if(mcpsIndication->Status == LORAMAC_EVENT_INFO_STATUS_DOWNLINK_TOO_MANY_FRAMES_LOSS)
        {
            DEBUG_OUTPUT("Reset\r\n");
            //SX1276Reset();
            //DelayMs(7);
            //persist.flags &= ~FLAGS_SESSPAR;
            //persist.flags |= FLAGS_JOINPAR;
            //eeprom_write();
            BoardResetMcu();
        }
        return;
    }

    switch( mcpsIndication->McpsIndication )
    {
        case MCPS_UNCONFIRMED:
        {
            break;
        }
        case MCPS_CONFIRMED:
        {
            bitNeedAck = true;
            bitNeedAckTimer  = TimerGetCurrentTime();
            break;
        }
        case MCPS_PROPRIETARY:
        {
            break;
        }
        case MCPS_MULTICAST:
        {
            break;
        }
        default:
            break;
    }

    // Check Multicast
    // Check Port
    // Check Datarate
    // Check FramePending
    if( mcpsIndication->FramePending == true )
    {
        // The server signals that it has pending data to be sent.
        // We schedule an uplink as soon as possible to flush the server.
        OnTxNextPacketTimerEvent( );
    }
    // Check Buffer
    // Check BufferSize
    // Check Rssi
    // Check Snr
    // Check RxSlot

    if( ComplianceTest.Running == true )
    {
        ComplianceTest.DownLinkCounter++;
    }

    if( mcpsIndication->RxData == true )
    {
        switch( mcpsIndication->Port )
        {
        case 1: // The application LED can be controlled on port 1 or 2
            if( mcpsIndication->BufferSize == 1 )
            {
                AppLedStateOn = mcpsIndication->Buffer[0] & 0x01;
                Board_LED_Set( 0, AppLedStateOn );
            }
            break;
        case 224:
            if( ComplianceTest.Running == false )
            {
            }
            break;
        default:
            break;
        }
        LMIC.txrxFlags = 0;
        if(IsLastTxConfirmed == true)
        {
            LMIC.txrxFlags |= (mcpsIndication->AckReceived ? TXRX_ACK : TXRX_NACK);
        }
        mcpsIndication->RxSlot?(LMIC.txrxFlags |= TXRX_DNW2):(LMIC.txrxFlags |= TXRX_DNW1);
        LMIC.txrxFlags |= TXRX_PORT;
        LMIC.dataBeg = 1;
        LMIC.frame[LMIC.dataBeg - 1] = mcpsIndication->Port;
        LMIC.dataLen = mcpsIndication->BufferSize;
        memcpy(LMIC.frame + LMIC.dataBeg,mcpsIndication->Buffer,mcpsIndication->BufferSize);
        onEvent(EV_RXCOMPLETE);
    }

    // Switch LED 2 ON for each received downlink
    //GpioWrite( &Led2, 0 );
    //TimerStart( &Led2Timer );
}

/*!
 * \brief   MLME-Confirm event function
 *
 * \param   [IN] mlmeConfirm - Pointer to the confirm structure,
 *               containing confirm attributes.
 */
static void MlmeConfirm( MlmeConfirm_t *mlmeConfirm )
{
    switch( mlmeConfirm->MlmeRequest )
    {
        case MLME_JOIN:
        {
            if( mlmeConfirm->Status == LORAMAC_EVENT_INFO_STATUS_OK )
            {
                // Status is OK, node has joined the network
                if(persist.nodetype == CLASS_A)
                {
                    DeviceState = DEVICE_STATE_SLEEP;
                }
                else
                {
                    DeviceState = DEVICE_STATE_SEND;
                }
                //onEvent(EV_JOINED);
                //Board_LED_Set(0,0);
            }
            else
            {
                // Join was not successful. Try to join again
                MlmeReq_t mlmeReq;
                mlmeReq.Type = MLME_JOIN;
                mlmeReq.Req.Join.DevEui = DevEui;
                mlmeReq.Req.Join.AppEui = AppEui;
                mlmeReq.Req.Join.AppKey = AppKey;
                mlmeReq.Req.Join.Datarate = LORAWAN_DEFAULT_DATARATE;

                if( LoRaMacMlmeRequest( &mlmeReq ) == LORAMAC_STATUS_OK )
                {
                    DeviceState = DEVICE_STATE_SLEEP;
                }
                else
                {
                    DeviceState = DEVICE_STATE_CYCLE;
                }
            }
            break;
        }
        case MLME_LINK_CHECK:
        {
            if( mlmeConfirm->Status == LORAMAC_EVENT_INFO_STATUS_OK )
            {
                // Check DemodMargin
                // Check NbGateways
                if( ComplianceTest.Running == true )
                {
                    ComplianceTest.LinkCheck = true;
                    ComplianceTest.DemodMargin = mlmeConfirm->DemodMargin;
                    ComplianceTest.NbGateways = mlmeConfirm->NbGateways;
                }
            }
            break;
        }
        default:
            break;
    }
    NextTx = true;
}

/*!
 * \brief   MLME-Indication event function
 *
 * \param   [IN] mlmeIndication - Pointer to the indication structure.
 */
static void MlmeIndication( MlmeIndication_t *mlmeIndication )
{
    switch( mlmeIndication->MlmeIndication )
    {
        case MLME_SCHEDULE_UPLINK:
        {// The MAC signals that we shall provide an uplink as soon as possible
            OnTxNextPacketTimerEvent( );
            break;
        }
        default:
            break;
    }
}

/**
 * Main application entry point.
 */
int main( void )
{
    LoRaMacPrimitives_t LoRaMacPrimitives;
    LoRaMacCallback_t LoRaMacCallbacks;
    MibRequestConfirm_t mibReq;

    BoardInitMcu( );

    DEBUG_OUTPUT("Power on\r\n");
    DeviceState = DEVICE_STATE_INIT;
    modem_wkt_init();
    modem_wwdt_init();
    while( 1 )
    {
        switch( DeviceState )
        {
            case DEVICE_STATE_INIT:
            {
                srand1( BoardGetRandomSeed( ) );
                LoRaMacPrimitives.MacMcpsConfirm = McpsConfirm;
                LoRaMacPrimitives.MacMcpsIndication = McpsIndication;
                LoRaMacPrimitives.MacMlmeConfirm = MlmeConfirm;
                LoRaMacPrimitives.MacMlmeIndication = MlmeIndication;
                //LoRaMacCallbacks.GetBatteryLevel = BoardGetBatteryLevel;
                LoRaMacInitialization( &LoRaMacPrimitives, &LoRaMacCallbacks, ACTIVE_REGION );

                TimerInit( &TxNextPacketTimer, OnTxNextPacketTimerEvent );

                mibReq.Type = MIB_ADR;
                mibReq.Param.AdrEnable = LORAWAN_ADR_ON;
                LoRaMacMibSetRequestConfirm( &mibReq );

                mibReq.Type = MIB_PUBLIC_NETWORK;
                mibReq.Param.EnablePublicNetwork = LORAWAN_PUBLIC_NETWORK;
                LoRaMacMibSetRequestConfirm( &mibReq );

                mibReq.Type = MIB_SYSTEM_MAX_RX_ERROR;
                mibReq.Param.SystemMaxRxError = 50;
                LoRaMacMibSetRequestConfirm( &mibReq );

                mibReq.Type = MIB_CHANNELS_DEFAULT_TX_POWER;
                mibReq.Param.ChannelsDefaultTxPower = TX_POWER_7;
                LoRaMacMibSetRequestConfirm( &mibReq );
                
                mibReq.Type = MIB_DEVICE_CLASS;
                mibReq.Param.Class = persist.nodetype;;
                LoRaMacMibSetRequestConfirm( &mibReq );

                static uint16_t ChannelsDefaultMask[6];
                mibReq.Param.ChannelsMask = ChannelsDefaultMask;
                mibReq.Type = MIB_CHANNELS_DEFAULT_MASK;
                memset(ChannelsDefaultMask,0,sizeof(ChannelsDefaultMask));
                memcpy(ChannelsDefaultMask,persist.startchannelid,sizeof(ChannelsDefaultMask));
                LoRaMacMibSetRequestConfirm( &mibReq );
                mibReq.Type = MIB_CHANNELS_MASK;
                LoRaMacMibSetRequestConfirm( &mibReq );
                
                mibReq.Type = MIB_CHANNELS_DEFAULT_DATARATE;
                mibReq.Param.ChannelsDefaultDatarate = enableChannelDR[0];
                LoRaMacMibSetRequestConfirm( &mibReq );
                
#if defined( REGION_EU868 )
                LoRaMacTestSetDutyCycleOn( LORAWAN_DUTYCYCLE_ON );
#endif
                DeviceState = DEVICE_STATE_JOIN;
                break;
            }
            case DEVICE_STATE_JOIN:
            {
                if( persist.flags & FLAGS_JOINPAR )
                {
                    MlmeReq_t mlmeReq;
                    LoRaMacStatus_t status;
                    // Initialize LoRaMac device unique ID
                    BoardGetUniqueId( DevEui );

                    mlmeReq.Type = MLME_JOIN;
                                    
                    mlmeReq.Req.Join.DevEui = persist.joinpar.deveui;
                    mlmeReq.Req.Join.AppEui = persist.joinpar.appeui;
                    mlmeReq.Req.Join.AppKey = persist.joinpar.devkey;
                    mibReq.Type = MIB_CHANNELS_DEFAULT_DATARATE;
                    status = LoRaMacMibGetRequestConfirm( &mibReq );
                    if( status == LORAMAC_STATUS_OK )
                    {
                        mlmeReq.Req.Join.Datarate = mibReq.Param.ChannelsDefaultDatarate;
                    }
                    else
                    {
                        mlmeReq.Req.Join.Datarate = LORAWAN_DEFAULT_DATARATE;
                    }

                    if( LoRaMacMlmeRequest( &mlmeReq ) == LORAMAC_STATUS_OK )
                    {
                        onEvent(EV_JOINING);
                        DeviceState = DEVICE_STATE_SLEEP;
                    }
                    else
                    {
                        DeviceState = DEVICE_STATE_CYCLE;
                    }
                }
                else
                {
                    // Choose a random device address if not already defined in Commissioning.h
                    /*if( DevAddr == 0 )
                    {
                        // Random seed initialization
                        //srand1( BoardGetRandomSeed( ) );

                        // Choose a random device address
                        DevAddr = randr( 0, 0x01FFFFFF );
                    }*/

                    mibReq.Type = MIB_NET_ID;
                    mibReq.Param.NetID = persist.sesspar.netid;
                    LoRaMacMibSetRequestConfirm( &mibReq );

                    mibReq.Type = MIB_DEV_ADDR;
                    mibReq.Param.DevAddr = persist.sesspar.devaddr;
                    LoRaMacMibSetRequestConfirm( &mibReq );

                    mibReq.Type = MIB_NWK_SKEY;
                    mibReq.Param.NwkSKey = persist.sesspar.nwkkey;
                    LoRaMacMibSetRequestConfirm( &mibReq );

                    mibReq.Type = MIB_APP_SKEY;
                    mibReq.Param.AppSKey = persist.sesspar.artkey;
                    LoRaMacMibSetRequestConfirm( &mibReq );

                    mibReq.Type = MIB_NETWORK_JOINED;
                    mibReq.Param.IsNetworkJoined = true;
                    LoRaMacMibSetRequestConfirm( &mibReq );
                    
                    mibReq.Type = MIB_CHANNELS_DEFAULT_DATARATE;
                    mibReq.Param.ChannelsDefaultDatarate = persist.sesspar.JoinRequestTrials;
                    LoRaMacMibSetRequestConfirm( &mibReq );
                    if(persist.nodetype == CLASS_C)
                    {
                        DeviceState = DEVICE_STATE_SEND;
                    }
                    else
                    {
                        DeviceState = DEVICE_STATE_SLEEP;
                    }
                }
                break;
            }
            case DEVICE_STATE_SEND:
            {
                if( NextTx == true )
                {
                    AppPort = LORAWAN_APP_PORT;
                    IsTxConfirmed = true;
                    PrepareTxFrame( AppPort );

                    NextTx = SendFrame( );
                }
                /*if( ComplianceTest.Running == true )
                {
                    // Schedule next packet transmission
                    TxDutyCycleTime = 5000; // 5000 ms
                }
                else*/
                {
                    // Schedule next packet transmission
                    TxDutyCycleTime = persist.sesspar.alarm * 1000 + randr(-100, 100);
                }
                DEBUG_OUTPUT("TxDutyCycleTime = %d\r\n",TxDutyCycleTime);
                DeviceState = DEVICE_STATE_CYCLE;
                break;
            }
            case DEVICE_STATE_CYCLE:
            {
                DeviceState = DEVICE_STATE_SLEEP;

                // Schedule next packet transmission
                if(persist.sesspar.alarm)
                {
                    TimerSetValue( &TxNextPacketTimer, TxDutyCycleTime );
                    TimerStart( &TxNextPacketTimer );
                    DEBUG_OUTPUT("Start tx packet timer\r\n");
                }
                break;
            }
            case DEVICE_STATE_SLEEP:
            {
                // Wake up through events
                TimerLowPowerHandler( );
                break;
            }
            default:
            {
                DeviceState = DEVICE_STATE_INIT;
                break;
            }
        }
    }
}
