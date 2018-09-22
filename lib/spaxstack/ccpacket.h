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


/**
 * Class: CCPACKET
 * 
 * Description:
 * CC1101 data packet class
 */
class CCPACKET
{
  public:
    void copy(const CCPACKET &p2); 
    char* getHumanReadableError();
    /**
     * ERROR flag
     */
    byte errorCode;
    /**
     * Data length
     */
    byte length;

    /**
     * Data buffer, 2 more bytes for RSSI and CRC, 
     * see 15 Packet Handling Hardware Support in CC1101 datasheet
     */
    byte data[CC1101_DATA_LEN+2];
  
    /**
     * Received Strength Signal Indication
     */
    byte rssi;

    /**
     * Link Quality Index
     */
    byte lqi;

};

#endif

