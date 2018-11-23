/**
 * Copyright (c) 2018 autonity <contact@autonity.de>
 * 
 * This file is part of the spaxxity project.
 * 
 * spaxxity is free software; you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation; either version 3 of the License, or
 * any later version.
 * 
 * spaxxity is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU Lesser General Public License for more details.
 * 
 * You should have received a copy of the GNU Lesser General Public License
 * along with spaxxity; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301 
 * USA
 * 
 * Author: Daniel Berenguer, Lucian Nutiu 
 * Creation date: 03/03/2011
 */

#ifndef _SWPACKET_H
#define _SWPACKET_H

#include "ccpacket.h"


/**
 * SWAP definitions
 */
#define SWAP_DATA_HEAD_LEN     6
#define SWAP_REG_VAL_LEN       CC1101_DATA_LEN - SWAP_DATA_HEAD_LEN   // SWAP data payload - max length
#define SWAP_BCAST_ADDR        0x00                                   // SWAP broadcast address
#define SWAP_NB_TX_TRIES       3                                      // Number of transmission retries
#define SWAP_TX_DELAY          20         // Delay before sending

#define ACK_HIGHBYTE 0xF0
#define ACK_LOWBYTE 0xF0
/**
 * SWAP message functions
 */
enum SWAPFUNCT
{
  SWAPFUNCT_STA = 0x00, // status packet
  SWAPFUNCT_QRY, // query packet
  SWAPFUNCT_CMD, // command packet
  SWAPFUNCT_ACK // packet aknowledgment
};

/**
 * Structure: SWDATA
 * 
 * Description:
 * SWAP data structure
 */
struct SWDATA
{
    /**
     * Data buffer
     */
    volatile byte *data;

    /**
     * Data length
     */
    byte length;
};

class SWPACKET : public CCPACKET
{
  public:
    /**
     * Destination address
     */
    byte destAddr;

    /**
     * Source address
     */
    byte srcAddr;

    /**
     * Hop counter. Incremented each time the message is repeated
     */
    byte hop;

    /**
     * Packet number 
     */
    byte packetNo;

    /**
     * Function byte
     */
    byte function;

    /**
     * Register address
     */
    byte regAddr;

    /**
     * Register id
     */
    byte regId;

    /**
     * Register value
     */
    SWDATA value;

    /**
     * SWPACKET
     * 
     * Class constructor
     * 
     * 'packet'	Raw CC1101 packet
     */
    SWPACKET(volatile CCPACKET* packet);

    /**
     * SWPACKET
     * 
     * Class constructor
     */
    SWPACKET(void);
    
    /**
     * send
     * 
     * Send SWAP packet
     *
     * Return:
     *  True if the transmission succeeds
     *  False otherwise
     */
    CCPACKET prepare(void);
    char* asString(char* buffer);
};

#endif
