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
 * Creation date: 07/06/2011
 */

#ifndef _COMMONREGS_H
#define _COMMONREGS_H
#include "swstatus.h"
#include "spaxstack.h"

const void setFreqChannel(uint8_t id, uint8_t *channel);
const void setNodeAddress(uint8_t id, uint8_t *addr);
const void setNetworkId(uint8_t rId, uint8_t *nId);
const void setTxInterval(uint8_t id, uint8_t *interval);

/**
 * The registers common to all SWAP devices
 */
enum CUSTOM_REGINDEX                    
{                                                       
  REGI_FREQCHANNEL,                     
  REGI_NETWORKID,                       
  REGI_DEVADDRESS,                      
  REGI_TXINTERVAL
};

typedef struct t_common_registers {
        typeof(commstack.cc1101.channel) channel,
        typeof(commstack.cc1101.syncWord) syncWord,
        typeof(commstack.cc1101.devAddress) devAddress,
        typeof(commstack.cc1101.txInterval) txInterval
};


#endif