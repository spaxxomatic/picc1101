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

#include "swpacket.h"
#include "spaxstack.h"
#include <stdio.h>
#include "../../util.h"
#include "../types.h"
#include "memory.h"

/**
 * SWPACKET
 * 
 * Class constructor from CCPACKET, used for reception
 * 
 * 'packet'	Raw CC1101 packet
 */
SWPACKET::SWPACKET(CCPACKET* packet) 
{
  destAddr = packet->data[0];
  srcAddr = packet->data[1];
  byte ctrlbyte = packet->data[2];
  hop = ( ctrlbyte >> 4) & 0x0F;
  packetNo = packet->data[3];
  function = packet->data[4];
  lqi = packet->lqi;
  rssi = packet->rssi;
  regAddr = packet->data[5];
  if (function != SWAPFUNCT_ACK){
    regId = packet->data[6];
    value.length = packet->length - SWAP_DATA_HEAD_LEN - 1;
    encrypted = bitRead(ctrlbyte, 1);
    request_ack = bitRead(ctrlbyte, 2);
    //lsb of data[2] indicates the data type, 1->string, 0->number
    value.is_string = bitRead(ctrlbyte, 0);
    if (value.is_string){
      value.chardata = packet->data + 7;
    }else{
      value.bytedata = *(packet->data + 7);
    }
  }
}

/**
 * SWPACKET
 * 
 * Class constructor
 */
SWPACKET::SWPACKET(void) 
{
}

/**
 * prepare
 * 
 * Fills up a CCPACKET structure for sending.
 * 
 */
void SWPACKET::prepare(CCPACKET* packet)
{
  byte i;
  bool res;
  packetNo = commstack.sentPacketNo++;
  packet->length = value.length + SWAP_DATA_HEAD_LEN + 1;
  packet->errorCode = 0;
  packet->data[0] = destAddr;
  packet->data[1] = srcAddr;
  packet->data[2] = (hop << 4) & 0xF0;
  if (encrypted) bitSet(packet->data[2], 1);
  if (request_ack) bitSet(packet->data[2], 2);
  packet->data[3] = packetNo;
  packet->data[4] = function;
  packet->data[5] = regAddr;
  packet->data[6] = regId;
  
  if (value.is_string){
    bitSet(packet->data[2], 0); //bit 0 is set, indicating a string value
    for(i=0 ; i<value.length ; i++)
      packet->data[i+7] = value.chardata[i];
  }else{
    packet->data[7] = value.bytedata;
  }
}

char* SWPACKET::as_string(char* buffer){
  sprintf(buffer, "DEST: %i SRC: %i PKTNO: %i FUNC: %i REGADDR: %02X REGID: %02X LEN: %i ENC: %i RACK: %i LQI: %i\n ", 
  destAddr, srcAddr, packetNo, function, regAddr, regId, value.length, encrypted, request_ack, lqi);
  return buffer;
}

char* SWPACKET::val_to_string(char* buffer){
  if (value.is_string){
    memcpy(buffer, value.chardata, value.length);
    buffer[value.length] = '0'; //terminate string
  }else{
    sprintf(buffer, "%ld", value.bytedata);
  }
  return buffer;
}
