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

#ifndef _CCPACKET_H
#define _CCPACKET_H
#include "../types.h"
/**
 * Buffer and data lengths
 */
#define CC1101_BUFFER_LEN        64
#define CC1101_DATA_LEN          CC1101_BUFFER_LEN - 3
#include <string> 
using namespace std; 

/**
 * Class: CCPACKET
 * 
 * Description:
 * CC1101 data packet class
 */
class CCPACKET
{
  public:
    CCPACKET(); //constuctor
    CCPACKET(const CCPACKET* source); //copy constuctor
    void copy(const CCPACKET* source);
    
    /**
     * ERROR flag
     */
    uint8_t errorCode;
    /**
     * Data length
     */
    uint8_t length;

    /**
     * Data buffer, 2 more bytes for RSSI and CRC, 
     * see 15 Packet Handling Hardware Support in CC1101 datasheet
     */
    byte data[CC1101_DATA_LEN+2];
  
    /**
     * Received Strength Signal Indication
     */
    uint8_t rssi;

    /**
     * Link Quality Index
     */
    uint8_t lqi;

    void printAsHex();
    std::string to_string() const;
    bool ack_ok = false; //packet has been aknowledged
    void incr_retry_cnt () ;
    int retry = 0; //number of resends
};

#endif

