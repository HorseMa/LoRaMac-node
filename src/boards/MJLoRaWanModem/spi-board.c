/*!
 * \file      spi-board.c
 *
 * \brief     Target board SPI driver implementation
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
#include "board.h"
#include "gpio.h"
#include "spi-board.h"

void SpiInit( Spi_t *obj, SpiId_t spiId, PinNames mosi, PinNames miso, PinNames sclk, PinNames nss )
{
    SPI_DELAY_CONFIG_T DelayConfigStruct;
    BoardDisableIrq( );
    Chip_Clock_EnablePeriphClock(SYSCTL_CLOCK_SWM);

    //Chip_IOCON_PinSetMode(LPC_IOCON,IOCON_PIO23,PIN_MODE_REPEATER);
    //Chip_IOCON_PinDisableOpenDrainMode(LPC_IOCON,IOCON_PIO23);
    Chip_GPIO_SetPinOutHigh(LPC_GPIO_PORT,0,23); // NRESET
    //Chip_IOCON_PinSetMode(LPC_IOCON,IOCON_PIO14,PIN_MODE_REPEATER);
    //Chip_IOCON_PinDisableOpenDrainMode(LPC_IOCON,IOCON_PIO14);
    Chip_SWM_MovablePinAssign(SWM_SPI1_SSEL0_IO, 14);
    Chip_GPIO_SetPinOutHigh(LPC_GPIO_PORT, 0, 14);
    //Chip_IOCON_PinSetMode(LPC_IOCON,IOCON_PIO25,PIN_MODE_REPEATER);
    Chip_IOCON_PinDisableOpenDrainMode(LPC_IOCON,IOCON_PIO25);
    Chip_SWM_MovablePinAssign(SWM_SPI1_SCK_IO, 25);
    //Chip_IOCON_PinSetMode(LPC_IOCON,IOCON_PIO7,PIN_MODE_REPEATER);
    //Chip_IOCON_PinDisableOpenDrainMode(LPC_IOCON,IOCON_PIO7);
    Chip_SWM_MovablePinAssign(SWM_SPI1_MISO_IO, 7);
    //Chip_IOCON_PinSetMode(LPC_IOCON,IOCON_PIO6,PIN_MODE_REPEATER);
    Chip_IOCON_PinDisableOpenDrainMode(LPC_IOCON,IOCON_PIO6);
    Chip_SWM_MovablePinAssign(SWM_SPI1_MOSI_IO, 6);
    Chip_Clock_DisablePeriphClock(SYSCTL_CLOCK_SWM);
    /*
      ConfigStruct.Mode = SPI_MODE_MASTER;
      ConfigStruct.ClkDiv = Chip_SPI_CalClkRateDivider(LPC_SPI, 100000);
      ConfigStruct.ClockMode = SPI_CLOCK_CPHA0_CPOL0;
      ConfigStruct.DataOrder = SPI_DATA_MSB_FIRST;
      ConfigStruct.SSELPol = SPI_SSEL_ACTIVE_LO;
    */
    Chip_SPI_Init(LPC_SPI1);
    Chip_SPI_ConfigureSPI(LPC_SPI1, SPI_MODE_MASTER |  /* Enable master/Slave mode */
                            SPI_CLOCK_CPHA0_CPOL0 |   /* Set Clock polarity to 0 */
                            SPI_CFG_MSB_FIRST_EN |/* Enable MSB first option */
                            SPI_CFG_SPOL_LO); /* Chipselect is active low */
    Chip_SPIM_SetClockRate(LPC_SPI1,1000000);
    DelayConfigStruct.FrameDelay = 0;
    DelayConfigStruct.PostDelay = 0;
    DelayConfigStruct.PreDelay = 0;
    DelayConfigStruct.TransferDelay = 0;
    Chip_SPI_DelayConfig(LPC_SPI1, &DelayConfigStruct);
    Chip_SPI_Enable(LPC_SPI1);
    Chip_Clock_EnablePeriphClock(SYSCTL_CLOCK_SPI1);

    BoardEnableIrq( );
}

void SpiDeInit( Spi_t *obj )
{
}


uint16_t SpiInOut( Spi_t *obj, uint16_t outData )
{
    Chip_SPI_ClearStatus(LPC_SPI1, SPI_STAT_CLR_RXOV | SPI_STAT_CLR_TXUR | SPI_STAT_CLR_SSA | SPI_STAT_CLR_SSD);
    while (!(Chip_SPI_GetStatus(LPC_SPI1) & SPI_STAT_TXRDY)) {};
    Chip_SPI_SendMidFrame(LPC_SPI1, outData);
    while (!(Chip_SPI_GetStatus(LPC_SPI1) & SPI_STAT_RXRDY)) {};
    return Chip_SPI_ReceiveFrame(LPC_SPI1);
#if 0
    SPI_DATA_SETUP_T pXfSetup;
    uint16_t tx,rx;
    
    tx = outData;
    pXfSetup.Length = 1;
    pXfSetup.pTx = &tx;
    pXfSetup.pRx = &rx;
    pXfSetup.RxCnt = pXfSetup.TxCnt = 0;
    pXfSetup.DataSize = 8;
    //Chip_GPIO_SetPinOutLow(LPC_GPIO_PORT,0,14);
    BoardDisableIrq( );
    Chip_SPI_RWFrames_Blocking(LPC_SPI1, &pXfSetup);
    //Chip_GPIO_SetPinOutHighe(LPC_GPIO_PORT,0,14);
    BoardEnableIrq( );
    return *pXfSetup.pRx;
#endif
}

