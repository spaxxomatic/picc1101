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

#include "spaxstack.h"
#include "protocol.h"
#include "debug.h"
#include "stddef.h"

#define SYNCWORD1        0xB5    // Synchronization word, high byte
#define SYNCWORD0        0x47    // Synchronization word, low byte

/**
 * Array of registers
 */
extern REGISTER* regTable[];

/**
 * commstack
 *
 * Class constructor
 */
SPAXSTACK::SPAXSTACK(void)
{
  seqNo = 0;
  bDebug = false;
}


/**
 * getRegister
 *
 * Return pointer to register with ID = regId
 *
 * 'regId'  Register ID
 */
REGISTER * getRegister(byte regId)
{
  //nutiu TODO
  /*
  if (regId >= regTableSize)
    return NULL;
  return regTable[regId]; 
  */
 return NULL;
}

/**
 * init
 * 
 * Initialize commstack board
 */
void SPAXSTACK::init() 
{
  // Default values
  sentPacketNo = 0;
  errorCode = 0;
  stackState = 0;
}

/**
 * Pre-instantiate SPAXSTACK object
 */
SPAXSTACK commstack;