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

/**
 * SWPACKET
 * 
 * Class constructor for transmission
 * 
 * 'packet'	Raw CC1101 packet
 */
/*
SWPACKET::SWPACKET(byte destAddr, byte function, byte regId, SWDATA data ) 
{
  this->destAddr = destAddr;
  this->srcAddr = MASTER_ADDRESS;
  this->hop = 0;
  this->packetNo = 0;
  this->function = function;
  this->regAddr = packet->data[5];
  regId = packet->data[6];
  value.data = packet->data + 7;
  value.length = packet->length - SWAP_DATA_HEAD_LEN - 1;
}
*/
/**
 * SWPACKET
 * 
 * Class constructor from CCPACKET, used for reception
 * 
 * 'packet'	Raw CC1101 packet
 */
SWPACKET::SWPACKET(volatile CCPACKET* packet) 
{
//TODO: cleanup
  //#ifdef _DEBUG
  for(int i = 0; i < packet->length ; i++){
      if (i > 0) printf(":");
      printf("%02X", packet->data[i]);
  }
  printf("\n");
  //#endif
  
  destAddr = packet->data[0];
  srcAddr = packet->data[1];
  hop = (packet->data[2] >> 4) & 0x0F;
  packetNo = packet->data[3];
  function = packet->data[4];
  regAddr = packet->data[5];
  regId = packet->data[6];
  value.data = packet->data + 7;
  value.length = packet->length - SWAP_DATA_HEAD_LEN - 1;
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

  packet->length = value.length + SWAP_DATA_HEAD_LEN + 1;
  packet->errorCode = 0;
  packet->data[0] = destAddr;
  packet->data[1] = srcAddr;
  packet->data[2] = (hop << 4) & 0xF0;
  packet->data[3] = commstack.sentPacketNo;
  packet->data[4] = function;
  packet->data[5] = regAddr;
  packet->data[6] = regId;

  for(i=0 ; i<value.length ; i++)
    packet->data[i+7] = value.data[i];

  //commstack.sentPacketNo += 1; //increment packet number for next transmission
  //commstack.stackState = STACKSTATE_WAIT_ACK;
}


char* SWPACKET::asString(char* buffer){
  sprintf(buffer, "DEST: %i SRC: %i PKTNO: %i FUNC: %i REGADDR: %02X REGID: %02X LEN: %i\n ", destAddr, srcAddr, packetNo, function, regAddr, regId, value.length);
  return buffer;
}
