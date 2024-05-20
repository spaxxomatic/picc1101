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

#define SWAP_BCAST_ADDR       0x00                                   // SWAP broadcast address
#define SWAP_MASTER_ADDRESS   0X01

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
  SWAPFUNCT_ACK, // packet aknowledgment
  SWAPFUNCT_ALARM,  // alarm packet
  SWAPFUNCT_DEV_INDENTIFIER
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
    byte *chardata;
    long int bytedata;
    /**
     * Data length
     */
    byte length;
    bool is_string;
};

class SWPACKET 
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
    SWDATA payload;

    /* link quality indicator */
    uint8_t lqi; 
    uint8_t rssi;
    /* packet came encrypted or must be sent encrypted */
    bool encrypted;
    bool request_ack;
    /**
     * SWPACKET
     * 
     * Class constructor
     * 
     * 'packet'	Raw CC1101 packet
     */
    SWPACKET(CCPACKET* packet);

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
    void prepare(CCPACKET* packet);
    char* as_string(char* buffer);
    char* val_to_string(char* buffer);
};

#endif
