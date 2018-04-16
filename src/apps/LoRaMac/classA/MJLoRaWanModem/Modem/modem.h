/*
 * Copyright (c) 2014-2016 IBM Corporation.
 * All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *  * Neither the name of the <organization> nor the
 *    names of its contributors may be used to endorse or promote products
 *    derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
#ifndef __MODEM_H__
#define __MODEM_H__
#include "lpc824board.h"
#include "timer.h"
#include <string.h>
#include "debug.h"

// Device address
typedef uint32_t devaddr_t;

// Global maximum frame length
enum { STD_PREAMBLE_LEN  =  8 };
enum { MAX_LEN_FRAME     = 128 };
enum { LEN_DEVNONCE      =  2 };
enum { LEN_ARTNONCE      =  3 };
enum { LEN_NETID         =  3 };
enum { DELAY_JACC1       =  5 }; // in secs
enum { DELAY_DNW1        =  1 }; // in secs down window #1
enum { DELAY_EXTDNW2     =  1 }; // in secs
enum { DELAY_JACC2       =  DELAY_JACC1+(int)DELAY_EXTDNW2 }; // in secs
enum { DELAY_DNW2        =  DELAY_DNW1 +(int)DELAY_EXTDNW2 }; // in secs down window #1
enum { BCN_INTV_exp      = 7 };
enum { BCN_INTV_sec      = 1<<BCN_INTV_exp };
enum { BCN_INTV_ms       = BCN_INTV_sec*1000L };
enum { BCN_INTV_us       = BCN_INTV_ms*1000L };
enum { BCN_RESERVE_ms    = 2120 };   // space reserved for beacon and NWK management
enum { BCN_GUARD_ms      = 3000 };   // end of beacon period to prevent interference with beacon
enum { BCN_SLOT_SPAN_ms  =   30 };   // 2^12 reception slots a this span
enum { BCN_WINDOW_ms     = BCN_INTV_ms-(int)BCN_GUARD_ms-(int)BCN_RESERVE_ms };
enum { BCN_RESERVE_us    = 2120000 };
enum { BCN_GUARD_us      = 3000000 };
enum { BCN_SLOT_SPAN_us  =   30000 };
enum {
    // Data frame format
    OFF_DAT_HDR      = 0,
    OFF_DAT_ADDR     = 1,
    OFF_DAT_FCT      = 5,
    OFF_DAT_SEQNO    = 6,
    OFF_DAT_OPTS     = 8,
};
enum { MAX_LEN_PAYLOAD = MAX_LEN_FRAME-(int)OFF_DAT_OPTS-4 };

// modem version
#define VERSION_MAJOR 1
#define VERSION_MINOR 2
#define VERSION_STR   "VERSION 1.2 ("__DATE__" "__TIME__")"
#define MAX_LEN_FRAME   128
// LED ids
#define LED_SESSION 1  // (IMST: yellow, LRSC: green)
#define LED_POWER   2  // (IMST: green,  LRSC: red)

/* Last sector address */
#define START_ADDR_LAST_SECTOR  0x00007C00

/* Size of each sector */
#define SECTOR_SIZE             1024

/* LAST SECTOR */
#define IAP_LAST_SECTOR         31

/* Number of bytes to be written to the last sector */
#define IAP_NUM_BYTES_TO_WRITE  (64 * 2)

/* Number elements in array */
#define ARRAY_ELEMENTS          (IAP_NUM_BYTES_TO_WRITE / sizeof(uint32_t))

/* Data array to write to flash */
extern uint32_t src_iap_array_data[ARRAY_ELEMENTS];

#define EEPROM_BASE (0x8000 - 64 * 2)//(0x8000 - 0x80)

// patch patterns
//#define PATTERN_JOINCFG_STR "g0CMw49rRbav6HwQN0115g42OpmvTn7q" // (32 bytes)
//#define PATTERN_JOINCFG_HEX "6730434d7734397252626176364877514e303131356734324f706d76546e3771"
//#define PATTERN_JOINCFG_CRC 0x6B3D
//#define PATTERN_SESSCFG_STR "Bmf3quaCJwVKURWWREeGKtm0pqLD0Yhr5cpPkP6s" // (40 bytes)
//#define PATTERN_SESSCFG_HEX "426d6633717561434a77564b55525757524565474b746d3070714c4430596872356370506b503673"
//#define PATTERN_SESSCFG_CRC 0xC9D5

// layout of join paramters
typedef struct {
    uint8_t deveui[8];
    uint8_t appeui[8];
    uint8_t devkey[16];
    bool isPublic;
} joinparam_t;

// layout of session parameters
typedef struct {
    uint32_t netid;
    devaddr_t devaddr;
    uint8_t nwkkey[16];
    uint8_t artkey[16];
    uint32_t alarm;
    uint32_t JoinRequestTrials;
} sessparam_t;

// persistent state
typedef struct {
    uint32_t cfghash;
    uint8_t flags;
    joinparam_t joinpar;
    sessparam_t sesspar;
    uint32_t seqnoDn;
    uint32_t seqnoUp;
    uint32_t eventmask;
    uint8_t startchannelid;
    uint8_t channeltoenable;
    uint8_t nodetype;
} persist_t;

typedef              uint8_t* xref2u1_t;
typedef unsigned int       uint;
uint16_t os_crc16 (xref2u1_t data, uint len);

struct lmic_t {
    // Radio settings TX/RX (also accessed by HAL)
    //ostime_t    txend;
    //ostime_t    rxtime;
    uint32_t        freq;
    int8_t        rssi;
    int8_t        snr;
    //rps_t       rps;
    uint8_t        rxsyms;
    uint8_t        dndr;
    int8_t        txpow;     // dBm

    //osjob_t     osjob;

    // Channel scheduling
#if defined(CFG_eu868)
    //band_t      bands[MAX_BANDS];
    uint32_t        channelFreq[MAX_CHANNELS];
    uint16_t        channelDrMap[MAX_CHANNELS];
    uint16_t        channelMap;
#elif defined(CFG_us915)
    uint32_t        xchFreq[MAX_XCHANNELS];    // extra channel frequencies (if device is behind a repeater)
    uint16_t        xchDrMap[MAX_XCHANNELS];   // extra channel datarate ranges  ---XXX: ditto
    uint16_t        channelMap[(72+MAX_XCHANNELS+15)/16];  // enabled bits
    uint16_t        chRnd;        // channel randomizer
#endif
    uint8_t        txChnl;          // channel for next TX
    uint8_t        globalDutyRate;  // max rate: 1/2^k
    //ostime_t    globalDutyAvail; // time device can send again
    
    uint32_t        netid;        // current network id (~0 - none)
    uint16_t        opmode;
    uint8_t        upRepeat;     // configured up repeat
    int8_t        adrTxPow;     // ADR adjusted TX power
    uint8_t        datarate;     // current data rate
    uint8_t        errcr;        // error coding rate (used for TX only)
    uint8_t        rejoinCnt;    // adjustment for rejoin datarate
    int16_t        drift;        // last measured drift
    int16_t        lastDriftDiff;
    int16_t        maxDriftDiff;

    uint8_t        pendTxPort;
    uint8_t        pendTxConf;   // confirmed data
    uint8_t        pendTxLen;    // +0x80 = confirmed
    uint8_t        pendTxData[MAX_LEN_PAYLOAD];

    uint16_t        devNonce;     // last generated nonce
    uint8_t        nwkKey[16];   // network session key
    uint8_t        artKey[16];   // application router session key
    //devaddr_t   devaddr;
    uint32_t        seqnoDn;      // device level down stream seqno
    uint32_t        seqnoUp;

    uint8_t        dnConf;       // dn frame confirm pending: LORA::FCT_ACK or 0
    int8_t        adrAckReq;    // counter until we reset data rate (0=off)
    uint8_t        adrChanged;

    uint8_t        margin;
    uint8_t       ladrAns;      // link adr adapt answer pending
    uint8_t       devsAns;      // device status answer pending
    uint8_t        adrEnabled;
    uint8_t        moreData;     // NWK has more data pending
    uint8_t       dutyCapAns;   // have to ACK duty cycle settings
    uint8_t        snchAns;      // answer set new channel
    // 2nd RX window (after up stream)
    uint8_t        dn2Dr;
    uint8_t        dn2Freq;
    uint8_t        dn2Ans;       // 0=no answer pend, 0x80+ACKs

    // Class B state
    uint8_t        missedBcns;   // unable to track last N beacons
    uint8_t        bcninfoTries; // how often to try (scan mode only)
    uint8_t        pingSetAns;   // answer set cmd and ACK bits
    //rxsched_t   ping;         // pingable setup

    // Public part of MAC state
    uint8_t        txCnt;
    uint8_t        txrxFlags;  // transaction flags (TX-RX combo)
    uint8_t        dataBeg;    // 0 or start of data (dataBeg-1 is port)
    uint8_t        dataLen;    // 0 no data or zero length data, >0 byte count of data
    uint8_t        frame[MAX_LEN_FRAME];

    uint8_t        bcnChnl;
    uint8_t        bcnRxsyms;    // 
    //ostime_t    bcnRxtime;
    //bcninfo_t   bcninfo;      // Last received beacon info

    uint8_t        noRXIQinversion;
};

// TX-RX transaction flags - report back to user
enum { TXRX_ACK    = 0x80,   // confirmed UP frame was acked
       TXRX_NACK   = 0x40,   // confirmed UP frame was not acked
       TXRX_NOPORT = 0x20,   // set if a frame with a port was RXed, clr if no frame/no port
       TXRX_PORT   = 0x10,   // set if a frame with a port was RXed, LMIC.frame[LMIC.dataBeg-1] => port
       TXRX_DNW1   = 0x01,   // received in 1st DN slot
       TXRX_DNW2   = 0x02,   // received in 2dn DN slot
       TXRX_PING   = 0x04 }; // received in a scheduled RX slot
// Event types for event callback
enum _ev_t { EV_SCAN_TIMEOUT=1, EV_BEACON_FOUND,
             EV_BEACON_MISSED, EV_BEACON_TRACKED, EV_JOINING,
             EV_JOINED, EV_RFU1, EV_JOIN_FAILED, EV_REJOIN_FAILED,
             EV_TXCOMPLETE, EV_LOST_TSYNC, EV_RESET,
             EV_RXCOMPLETE, EV_LINK_DEAD, EV_LINK_ALIVE, EV_SCAN_FOUND,
             EV_TXSTART, EV_RXSTARTWIN1, EV_RXSTARTWIN2, EV_RXTIMEOUT };
typedef enum _ev_t ev_t;

typedef enum eLedSension
{
    EN_LED_SENSSION_TX,
    EN_LED_SENSSION_RX,
    EN_LED_SENSSION_NET,
}LedSension_t;

#define FLAGS_JOINPAR 0x01
#define FLAGS_SESSPAR 0x02

#define PERSIST ((persist_t*)EEPROM_BASE)


// frame rx/tx state
#define FRAME_PREAMBLE_FF   0xA5
#define FRAME_PREAMBLE_55   0xA6
#define FRAME_PREAMBLE_AA   0xA7
#define FRAME_PREAMBLE_END   0xA8
#define FRAME_INIT   0x00
#define FRAME_A_A    0xA1
#define FRAME_A_T    0xA2
#define FRAME_A_OK   0xA3
#define FRAME_A_ERR  0xA4
#define FRAME_B_B    0xB1
#define FRAME_B_LEN  0xB2
#define FRAME_B_LRC  0xB3
#define FRAME_B_OK   0xB4
#define FRAME_B_ERR  0xB5

typedef struct {
    uint8_t state;
    uint8_t *buf;
    uint16_t len;
    uint16_t max;
    uint8_t lrc;
 } FRAME;

extern TimerEvent_t Led1Timer_OnLine;
extern TimerEvent_t Led1Timer_OffLine;
extern struct lmic_t LMIC;
extern persist_t persist;
void modem_init (void);
void OnTxNextPacketTimerEvent( void );

void modem_rxdone (void);
void modem_txdone (void);

void buffer_init (void);
uint8_t* buffer_alloc (uint16_t len);
void buffer_free (uint8_t* buf, uint16_t len);

void queue_init (void);
void queue_add (uint8_t* buf, uint16_t len);
uint8_t queue_shift (FRAME* f);

void frame_init (FRAME* f, uint8_t* buf, uint16_t max);
uint16_t frame_tx (uint8_t next);
uint8_t frame_rx (uint8_t c);

void usart_init (void);
void usart_starttx (void);
void usart_startrx (void);

void leds_init (void);
void leds_set (uint8_t id, uint8_t state);
void LedIndication(void);

uint8_t gethex (uint8_t* dst, const uint8_t* src, uint16_t len);
uint8_t puthex (uint8_t* dst, const uint8_t* src, uint8_t len);
uint8_t int2hex (uint8_t* dst, uint32_t v);
uint8_t hex2int (uint32_t* n, const uint8_t* src, uint8_t len);
uint8_t dec2int (uint32_t* n, const uint8_t* src, uint8_t len);
void reverse (uint8_t* dst, const uint8_t* src, uint8_t len);
uint8_t tolower (uint8_t c);
uint8_t toupper (uint8_t c);

uint8_t cpystr (uint8_t* dst, const char* src);
uint8_t cmpstr (uint8_t* buf, uint8_t len, char* str);
void onEvent (ev_t ev);
void eeprom_erase (void);
void eeprom_write (void);
//void eeprom_copy (void* dst, const void* src, uint16_t len);
void modem_wkt_init(void);
void modem_wwdt_init(void);
void WakeupTest(WKT_CLKSRC_T clkSrc, uint32_t timeoutInSecs, CHIP_PMU_MCUPOWER_T powerTest);
void funWktAlarm(void);
void caculateDr(int8_t snr);
bool modemSendFrame(uint8_t port,uint8_t *data,uint8_t len,bool confirm);

#endif