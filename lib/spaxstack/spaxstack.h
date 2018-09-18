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
#include "repeater.h"


/**
 * RTC definitions
 */
#define RTC_250MS    0x03   // Timer 2 prescaler = 32
#define RTC_500MS    0x04   // Timer 2 prescaler = 64
#define RTC_1S       0x05   // Timer 2 prescaler = 128
#define RTC_2S       0x06   // Timer 2 prescaler = 256
#define RTC_8S       0x07   // Timer 2 prescaler = 1024

/**
 * Macros
 */
#define setSwapStatusCallBack(ptrFunc)   statusReceived = ptrFunc

#define setHighTxPower()    cc1101.setTxPowerAmp(PA_LongDistance)
#define setLowTxPower()     cc1101.setTxPowerAmp(PA_LowPower)

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
  private:
    /**
     * setup_watchdog
     * 
     * 'time'	Watchdog timer value
     */
    void setup_watchdog(byte time);
    void(*showActivity)(void);
    /**
     * setup_rtc
     *
     * Setup software (Timer 2) RTC
     *
     * 'time'   Timer2 prescaler
     *
     *          RTC_1S = 128 for 1 sec
     *          RTC_2S = 256 for 2 sec
     *          RTC_8S = 1024 for 8 sec
     */
    void setup_rtc(byte time);
    // a flag that a wireless packet has been received

  public:
    /**
     * repeater
     *
     * Pointer to repeater object
     */
    
    byte seqNo ; //sequence number of the received packet
    bool bEnterSleep;
    bool bDebug;
    bool ping(void) ;
    void receive_loop();
    //counter for watchdog timer
    volatile byte f_wdt ;
    //flags reception available, will by set by the ISR
    volatile bool packetAvailable ; 

    REPEATER *repeater;

    /**
     * True if the external 32.768 KHz crystal is enabled
     */
    bool rtcCrystal;

    /**
     * CC1101 radio interface
     */
    CC1101 cc1101;
    byte master_address[2];
    /**
     * Packet number
     */
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
     * Interval between periodic transmissions. 0 for asynchronous transmissions
     */
    byte txInterval[2];

    void enterSleep(void);
    /**
     * enableRepeater
     *
     * Enable repeater mode
     */
    void enableRepeater(void);

    /**
     * enableRepeater
     *
     * Enable repeater mode
     *
     * 'maxHop'  MAximum repeater count. Zero if omitted
     */
    void enableRepeater(byte maxHop=0);

    /**
     * SWAP status packet received. Callaback function
     */
    void (*statusReceived)(SWPACKET *status);

    /**
     * SPAXSTACK
     *
     * Class constructor
     */
    SPAXSTACK(void);

    /**
     * init
     * 
     * Initialize commstack board
     */
    void init(void);

    /**
     * reset
     * 
     * Reset commstack
     */
    void reset(void);

    /**
     * sleepWd
     * 
     * Put commstack into Power-down state during "time".
     * This function uses the internal watchdog timer in order to exit (interrupt)
     * from the power-doen state
     * 
     * 'time'	Sleeping time:
     *  WDTO_15MS  = 15 ms
     *  WDTO_30MS  = 30 ms
     *  WDTO_60MS  = 60 ms
     *  WDTO_120MS  = 120 ms
     *  WDTO_250MS  = 250 ms
     *  WDTO_500MS  = 500 ms
     *  WDTO_1S = 1 s
     *  WDTO_2S = 2 s
     *  WDTO_4S = 4 s
     *  WDTO_8S = 8 s
     */
    void sleepWd(byte time);

    /**
     * sleepRtc
     * 
     * Put commstack into Power-down state during "time".
     * This function uses Timer 2 connected to an external 32.768KHz crystal
     * in order to exit (interrupt) from the power-down state
     * 
     * 'time'	Sleeping time:
     *  RTC_250MS  = 250 ms
     *  RTC_500MS  = 500 ms
     *  RTC_1S = 1 s
     *  RTC_2S = 2 s
     *  RTC_8S = 8 s
     */
    void sleepRtc(byte time);

    /**
     * wakeUp
     *
     * Wake from sleep mode
     *
     * 'rxOn' Enter RX_ON state after waking up
     */
    void wakeUp(bool rxOn=true);

    /**
     * goToSleep
     *
     * Sleep whilst in power-down mode. This function currently uses sleepWd in a loop
     *
     */
    void enterSleepWithRadioOff(void);


    /**
     * getAddress
     * 
     * Sends a broadcast request for a device address. When addr is received, sets it and enables cc1101 packet filtering
     * 
     */
    bool getAddress(void);

    /**
     * getInternalTemp
     * 
     * Read internal (ATMEGA328 only) temperature sensor
     * Reference: http://playground.arduino.cc/Main/InternalTemperatureSensor
     * 
     * Return:
     * 	Temperature in degrees Celsius
     */
    long getInternalTemp(void);

    /**
     * setTxInterval
     * 
     * Set interval for periodic transmissions
     * 
     * 'interval'	New periodic interval. 0 for asynchronous devices
     * 'save'     If TRUE, save parameter in EEPROM
     */
    void setTxInterval(byte* interval, bool save);

    /**
     * sendAck
     * 
     * Sends a confirmation of packet reception
     * 
     */

    void sendAck(void);

    /**
     * getCapabilities
     * 
     * Sends the module capabilities 
     *  
     * 
     */
    void getCapabilities(void);
    
    bool waitState(cor_state* cs);
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

