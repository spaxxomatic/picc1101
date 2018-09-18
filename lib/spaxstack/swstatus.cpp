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

#include "swstatus.h"
#include "spaxstack.h"

/**
 * SWSTATUS
 * 
 * Class constructor
 * 
 * 'rId'	Register id
 * '*val'	New value
 * 'len'	Buffer length
 */
SWSTATUS::SWSTATUS(byte registerId, byte *val, byte len) 
{
  destAddr = SWAP_BCAST_ADDR;
  srcAddr = commstack.cc1101.devAddress;
  hop = 0;
  function = SWAPFUNCT_STA;
  regAddr = commstack.cc1101.devAddress;
  regId = registerId;
  value.length = len;
  value.data = val;
}

/**
 * SWACK
 * 
 * Class constructor
 * 
 * 'rId'	Register id
 * '*val'	New value
 * 'len'	Buffer length
 */
SWACK::SWACK(byte dAddr)
{
  destAddr = dAddr;
  srcAddr = commstack.cc1101.devAddress;
  hop = 0;
  function = SWAPFUNCT_ACK;
}
