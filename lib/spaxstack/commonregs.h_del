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
 * Macros for the definition of common register indexes
 */
#define DEFINE_COMMON_REGINDEX_START()  \
enum CUSTOM_REGINDEX                    \
{                                       \
  REGI_PRODUCTCODE = 0,                 \
  REGI_HWVERSION,                       \
  REGI_FWVERSION,                       \
  REGI_FREQCHANNEL,                     \
  REGI_NETWORKID,                       \
  REGI_DEVADDRESS,                      \
  REGI_TXINTERVAL,

#define DEFINE_COMMON_REGINDEX_END()    };

#define DEFINE_REGINDEX_START()   DEFINE_COMMON_REGINDEX_START()
#define DEFINE_REGINDEX_END()     DEFINE_COMMON_REGINDEX_END()

/**
 * Macro for the definition of registers common to all SWAP devices
 */
#define DEFINE_COMMON_REGISTERS()                                                                                            \
/* Product code */                                                                                                           \
static byte dtProductCode[8] = {SWAP_MANUFACT_ID >> 24, SWAP_MANUFACT_ID >> 16 , SWAP_MANUFACT_ID >> 8, SWAP_MANUFACT_ID,    \
                       SWAP_PRODUCT_ID >> 24, SWAP_PRODUCT_ID >> 16 , SWAP_PRODUCT_ID >> 8, SWAP_PRODUCT_ID};                \
REGISTER regProductCode(dtProductCode, sizeof(dtProductCode), NULL, NULL);                                                   \
/* Hardware version */                                                                                                       \
static byte dtHwVersion[4] = {HARDWARE_VERSION >> 24, HARDWARE_VERSION >> 16 , HARDWARE_VERSION >> 8, HARDWARE_VERSION};     \
REGISTER regHwVersion(dtHwVersion, sizeof(dtHwVersion), NULL, NULL);                                                         \
/* Firmware version */                                                                                                       \
static byte dtFwVersion[4] = {FIRMWARE_VERSION >> 24, FIRMWARE_VERSION >> 16 , FIRMWARE_VERSION >> 8, FIRMWARE_VERSION};     \
REGISTER regFwVersion(dtFwVersion, sizeof(dtFwVersion), NULL, NULL);                                                         \
/* Frequency channel */                                                                                                      \
REGISTER regFreqChannel(&commstack.cc1101.channel, sizeof(commstack.cc1101.channel), NULL, &setFreqChannel);                   \
/* Network Id */                                                                                                             \
REGISTER regNetworkId(commstack.cc1101.syncWord, sizeof(commstack.cc1101.syncWord), NULL, &setNetworkId);                      \
/* Device address */                                                                                                         \
REGISTER regDevAddress(&commstack.cc1101.devAddress, sizeof(commstack.cc1101.devAddress), NULL, &setNodeAddress);               \
/* Periodic Tx interval */                                                                                                   \
REGISTER regTxInterval(commstack.txInterval, sizeof(commstack.txInterval), NULL, &setTxInterval);

/**
 * Macros for the declaration of global table of registers
 */
#define DECLARE_REGISTERS_START()    \
REGISTER *regTable[] = {             \
        &regProductCode,             \
        &regHwVersion,               \
        &regFwVersion,               \
        &regFreqChannel,             \
        &regNetworkId,               \
        &regDevAddress,              \
        &regTxInterval,

#define DECLARE_REGISTERS_END()      \
};                                   \
/* Size of regTable */               \
byte regTableSize = sizeof(regTable)/sizeof(*regTable);

/**
 * Macro for the declaration of getter/setter functions related to all common registers
 */
#define DECLARE_COMMON_CALLBACKS()                          \
const void setFreqChannel(byte id, byte *channel);          \
const void setDevAddress(byte id, byte *addr);              \
const void setNetworkId(byte rId, byte *nId);               \
const void setTxInterval(byte id, byte *interval);

/**
 * Macro for the definition of getter/setter functions related to all common registers
 */
#define DEFINE_COMMON_CALLBACKS()                           \
                                                            \
/**                                                         \
 * setFreqChannel                                           \
 *                                                          \
 * Set frequency channel                                    \
 *                                                          \
 * 'id'       Register ID                                   \
 * 'channel'  New channel                                   \
 */                                                         \
const void setFreqChannel(byte id, byte *channel)           \
{                                                           \
  if (channel[0] != regFreqChannel.value[0])                \
  {                                                         \
    /* Send status message before entering the new          \
    frequency channel */                                    \
    SWSTATUS packet = SWSTATUS(regFreqChannel.id, channel, regFreqChannel.length); \
    packet.send();                                     \
    /* Update register value */                             \
    commstack.cc1101.setChannel(channel[0], true);           \
    /* Restart device */                                    \
    commstack.reset();                                       \
  }                                                         \
}                                                           \
                                                            \
                                                            \
/**                                                         \
 * setNodeAddress                                            \
 *                                                          \
 * Set device address                                       \
 *                                                          \
 * 'id'    Register ID                                      \
 * 'addr'  New device address                               \
 */                                                         \
const void setNodeAddress(byte id, byte *addr)               \
{                                                           \
  if ((addr[0] > 0) && (addr[0] != regDevAddress.value[0])) \
  {                                                         \
    /* Send status before setting the new address */        \
    SWSTATUS packet = SWSTATUS(regDevAddress.id, addr, regDevAddress.length); \
    commstack.sendAck();                                          \
    /* Update register value */                             \
    commstack.cc1101.setDevAddress(addr[0], true);           \
  }                                                         \
}                                                           \
                                                            \
/**                                                         \
 * setNetworkId                                             \
 *                                                          \
 * Set network id                                           \
 *                                                          \
 * 'rId' Register ID                                        \
 * 'nId' New network id                                     \
 */                                                         \
const void setNetworkId(byte rId, byte *nId)                \
{                                                           \
  if ((nId[0] != regNetworkId.value[0]) ||                  \
      (nId[1] != regNetworkId.value[1]))                    \
  {                                                         \
    /* Send status before taking the new network ID */      \
    SWSTATUS packet = SWSTATUS(regNetworkId.id, nId, regNetworkId.length); \
    packet.send();                                          \
    /* Update register value */                             \
    commstack.cc1101.setSyncWord(nId, true);                 \
  }                                                         \
}                                                           \
/**                                                         \
 * setTxInterval                                            \
 *                                                          \
 * Set periodic Tx interval                                 \
 *                                                          \
 * 'id'        Register ID                                  \
 * 'interval'  New interval (in seconds)                    \
 */                                                         \
const void setTxInterval(byte id, byte *interval)           \
{                                                           \
  if ((interval[0] != regTxInterval.value[0]) ||            \
      (interval[1] != regTxInterval.value[1]))              \
  {                                                         \
    commstack.setTxInterval(interval, true);                 \
  }                                                         \
}
#endif