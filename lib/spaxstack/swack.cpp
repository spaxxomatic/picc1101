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
 * Author: Lucian Nutiu 
 * Creation date: 03/01/2019
 */

#include "swack.h"
#include <stdlib.h>
#include "spaxstack.h"
#include <ctype.h>
#include <string.h>


/**
 * SWACK
 * 
 * Class constructor
 * 
 * 'dAddr'	  Destination address
 * 'rAddr'	  Register address
 */
SWACK::SWACK(byte dAddr, byte pktNo, byte errorCode)
{
  destAddr = dAddr;
  srcAddr = MASTER_ADDRESS;
  hop = 0;
  function = SWAPFUNCT_ACK;
  packetNo = pktNo;
  request_ack=false;
  regAddr = errorCode; //the SWACK stores the error code in the regAddr (which goes to byte 5 of the CCPACKET)
}

void SWACK::prepare(CCPACKET* packet)
{
  packet->length = 6;
  packet->data[0] = destAddr;
  packet->data[1] = srcAddr;
  packet->data[2] = hop;
  packet->data[3] = packetNo;
  packet->data[4] = SWAPFUNCT_ACK; //the function  
  packet->data[5] = regAddr; //the function  
}