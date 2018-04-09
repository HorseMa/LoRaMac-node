/*!
 * \file      eeprom-board.c
 *
 * \brief     Target board EEPROM driver implementation
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
#include "utilities.h"
#include "eeprom-board.h"
#include "modem.h"
#include "board.h"

uint32_t src_iap_array_data[ARRAY_ELEMENTS];

// write 32-bit word to EEPROM memory
void eeprom_write (void) {
    uint8_t ret_code;
    uint16_t joincfgcrc;
    uint16_t sesscfgcrc;

    joincfgcrc = os_crc16((uint8_t*)&persist.joinpar, sizeof(joinparam_t));
    sesscfgcrc = os_crc16((uint8_t*)&persist.sesspar, sizeof(sessparam_t));
    persist.cfghash = (joincfgcrc << 16) | sesscfgcrc;
    memset(src_iap_array_data,0,IAP_NUM_BYTES_TO_WRITE);
    memcpy(src_iap_array_data,&persist,sizeof(persist));
    Chip_WWDT_Feed(LPC_WWDT);
    /* Disable interrupt mode so it doesn't fire during FLASH updates */
    BoardDisableIrq();
    /* IAP Flash programming */
    /* Prepare to write/erase the last sector */
    ret_code = Chip_IAP_PreSectorForReadWrite(IAP_LAST_SECTOR, IAP_LAST_SECTOR);

    /* Error checking */
    if (ret_code != IAP_CMD_SUCCESS) {
        //Print_Val("Command failed to execute, return code is: ", ret_code);
    }

    /* Erase the last sector */
    //ret_code = Chip_IAP_EraseSector(IAP_LAST_SECTOR, IAP_LAST_SECTOR);
    ret_code = Chip_IAP_ErasePage(EEPROM_BASE / 64,EEPROM_BASE / 64 + 1);
    /* Error checking */
    if (ret_code != IAP_CMD_SUCCESS) {
        //Print_Val("Command failed to execute, return code is: ", ret_code);
    }

    /* Prepare to write/erase the last sector */
    ret_code = Chip_IAP_PreSectorForReadWrite(IAP_LAST_SECTOR, IAP_LAST_SECTOR);

    /* Error checking */
    if (ret_code != IAP_CMD_SUCCESS) {
        //Print_Val("Command failed to execute, return code is: ", ret_code);
    }

    /* Write to the last sector */
    //ret_code = Chip_IAP_CopyRamToFlash(START_ADDR_LAST_SECTOR, &val, 4);
    ret_code = Chip_IAP_CopyRamToFlash(EEPROM_BASE, src_iap_array_data, IAP_NUM_BYTES_TO_WRITE);

    /* Error checking */
    if (ret_code != IAP_CMD_SUCCESS) {
        //Print_Val("Command failed to execute, return code is: ", ret_code);
    }

    /* Re-enable interrupt mode */
    BoardEnableIrq();

    /* Start the signature generator for the last sector */
    Chip_FMC_ComputeSignatureBlocks(START_ADDR_LAST_SECTOR, (SECTOR_SIZE / 16));

    /* Check for signature geenration completion */
    while (Chip_FMC_IsSignatureBusy()) {}

    /* Get the generated FLASH signature value */
}