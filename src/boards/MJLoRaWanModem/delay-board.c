/*!
 * \file      delay-board.c
 *
 * \brief     Target board delay implementation
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
 *
 * \author    Johannes Bruder ( STACKFORCE )
 */
#include "board.h"
#include "delay-board.h"

void DelayMs( uint32_t ms )
{
    uint32_t i,j,k;
    for(k = 0;k < ms;k ++)
    for(i = 0;i < 1000;i++)
      for(j = 0;j < 5;j++);
}

void DelayUs( uint32_t us )
{
    uint32_t j,k;
    for(k = 0;k < us;k ++)
      for(j = 0;j < 5;j++);
}
