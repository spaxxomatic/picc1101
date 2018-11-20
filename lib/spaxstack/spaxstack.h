/**
 * Copyright (c) 2011 autonity <contact@autonity.de>
 * 
 * This file is derived from the commstack project. 
 * Credits to Daniel Berenguer
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
 * Creation date: 03/09/2018
 */

#ifndef _SPAXSTACK_H
#define _SPAXSTACK_H

#include "register.h"
#include "swpacket.h"
#include "swquery.h"
#include "config.h"

#define BROADCAST_ADDRESS 0X00
#define MASTER_ADDRESS 0X01

enum STACKSTATE
{
  STACKSTATE_WAIT_CONFIG = 0,
  STACKSTATE_WAIT_ACK,
  STACKSTATE_READY
};

void enterDeepSleepWithRx();

typedef struct
{
    byte wait_resp_timeout;
    byte state;
} cor_state;

/**
 * Class: SPAXSTACK
 * 
 * Description:
 * SPAXSTACK main class
 */
class SPAXSTACK
{
  public:
    /**
     * repeater
     *
     * Pointer to repeater object
     */
    
    byte seqNo ; //sequence number of the received packet
    bool bEnterSleep;
    bool bDebug;
    //flags reception available, will by set by the ISR
    byte sentPacketNo;
    /**
     * Stack error code
     */
    byte errorCode;
    /**
     * System state
     */
    byte stackState;
    /**
     * SPAXSTACK
     *
     * Class constructor
     */
    SPAXSTACK(void);

    /**
     * init
     * 
     * Initialize commstack 
     */
    void init(void);

    /**
     * reset
     * 
     * Reset commstack
     */
    void reset(void);
};

/**
 * Global SPAXSTACK object
 */
extern SPAXSTACK commstack;

/**
 * getRegister
 *
 * Return pointer to register with ID = regId
 *
 * 'regId'  Register ID
 */
REGISTER * getRegister(byte regId);

#endif

