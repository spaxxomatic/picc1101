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

const char* SWWAPFUNCT_NAMES[]  = {
  "Status", "Query", "Command", "Ack", "Alarm"
};

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
    payload.length = packet->length - SWAP_DATA_HEAD_LEN - 1;
    encrypted = bitRead(ctrlbyte, 1);
    request_ack = bitRead(ctrlbyte, 2);
    //lsb of data[2] indicates the data type, 1->string, 0->numeric
    payload.is_string = bitRead(ctrlbyte, 0);
    if (payload.is_string){
      payload.chardata = packet->data + SWAP_DATA_HEAD_LEN + 1;
      verbprintf(1,"SWPACKET test message %s\n", payload.chardata);
    }else{
      if (payload.length > 4){
        payload.bytedata = 0;
        fprintf(stderr, "Invalid pkt len %i", payload.length);
      }else{
        payload.bytedata = 0;
        memcpy(&payload.bytedata, packet->data + SWAP_DATA_HEAD_LEN + 1, payload.length);
      }
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
  packet->length = payload.length + SWAP_DATA_HEAD_LEN + 1;
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
  
  if (payload.is_string){
    bitSet(packet->data[2], 0); //bit 0 is set, indicating a string payload
    for(i=0 ; i<payload.length ; i++)
      packet->data[i+7] = payload.chardata[i];
  }else{
    packet->data[7] = payload.bytedata;
  }
}

char* SWPACKET::as_string(char* buffer){
  sprintf(buffer, "DEST: %i SRC: %i PKTNO: %i FUNC: %i REGADDR: %02X REGID: %02X ENC: %i RACK: %i LQI: %i RSSI: %i LEN: %i ", 
  destAddr, srcAddr, packetNo, function, regAddr, regId, encrypted, request_ack, lqi, rssi, payload.length);
  return buffer;
}

char* SWPACKET::val_to_string(char* buffer){
  if (payload.is_string){
    memcpy(buffer, payload.chardata, payload.length);
    buffer[payload.length] = '0'; //terminate string
  }else{
    sprintf(buffer, "%ld", payload.bytedata);
  }
  return buffer;
}
