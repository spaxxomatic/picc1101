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

/**
 * SWPACKET
 * 
 * Class constructor
 * 
 * 'packet'	Raw CC1101 packet
 */
SWPACKET::SWPACKET(volatile CCPACKET* packet) 
{
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
 * Prepares and returns a CCPACKET ready to be sent.
 *
 * Return:
 *  CCPACKET
 */
CCPACKET SWPACKET::prepare(void)
{
  CCPACKET packet;
  byte i;
  bool res;

  packet.length = value.length + SWAP_DATA_HEAD_LEN + 1;
  packet.data[0] = destAddr;
  packet.data[1] = srcAddr;
  packet.data[2] = (hop << 4) & 0xF0;
  packet.data[3] = commstack.sentPacketNo;
  packet.data[4] = function;
  packet.data[5] = regAddr;
  packet.data[6] = regId;

  for(i=0 ; i<value.length ; i++)
    packet.data[i+7] = value.data[i];

  commstack.sentPacketNo+=1; //increment packet number for next transmission
  commstack.stackState = STACKSTATE_WAIT_ACK;
  return packet;
}


char* SWPACKET::asString(char* buffer){
  sprintf(buffer, "SRC: %i DEST: %i \nFUNC: %i REG: %02X REGVAL: %02X\n ", srcAddr, destAddr, function, regAddr, regId);
  return buffer;
}
