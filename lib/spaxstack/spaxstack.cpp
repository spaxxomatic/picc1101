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
#include "commonregs.h"
#include "coroutine.h"
#include "calibration.h"
#include "wdt.h"
#include "protocol.h"
#include "debug.h"

#define enableIRQ_GDO0()          attachInterrupt(0, isrGDO0event, FALLING);
//#define enableIRQ_GDO0()          attachInterrupt(0, cc1101Interrupt, FALLING);
#define disableIRQ_GDO0()         detachInterrupt(0);

DEFINE_COMMON_REGINDEX_START()
DEFINE_COMMON_REGINDEX_END()


#define SYNCWORD1        0xB5    // Synchronization word, high byte
#define SYNCWORD0        0x47    // Synchronization word, low byte

/**
 * Array of registers
 */
extern REGISTER* regTable[];
extern byte regTableSize;


/**
 * isrGDO0event
 *
 * Event on GDO0 pin (INT0). Triggered by the CC1101 on packet receive
 */
void isrGDO0event(void)
{
  // Disable interrupt
  disableIRQ_GDO0();

  if (commstack.cc1101.rfState == RFSTATE_RX)
  {
    static CCPACKET ccPacket;
    static SWPACKET swPacket;
    REGISTER *reg;

    if (commstack.cc1101.receiveData(&ccPacket) > 0)
    {
      if (ccPacket.crc_ok)
      {
        swPacket = SWPACKET(ccPacket);
        // Repeater enabled?
        if (commstack.repeater != NULL)
          commstack.repeater->packetHandler(&swPacket);
          // Function
        switch(swPacket.function)
        {
            case SWAPFUNCT_ACK:
              if (swPacket.destAddr != commstack.cc1101.devAddress){
                if (commstack.stackState == STACKSTATE_WAIT_ACK){
                  //check packet no
                  if (swPacket.packetNo == commstack.sentPacketNo){
                    commstack.stackState = STACKSTATE_READY;
                  }
                }else{
                  commstack.errorCode = STACKERR_ACK_WITHOUT_SEND;
                }
              }else{
                commstack.errorCode = STACKERR_WRONG_DEST_ADDR;
              }
                break;                
            case SWAPFUNCT_CMD:
              // Command not addressed to us?
              if (swPacket.destAddr != commstack.cc1101.devAddress)
                break;
              // Destination address and register address must be the same
              if (swPacket.destAddr != swPacket.regAddr)
                break;
              // Valid register?
              if ((reg = getRegister(swPacket.regId)) == NULL)
                break;
              // Filter incorrect data lengths
              if (swPacket.value.length == reg->length)
                reg->setData(swPacket.value.data);
              else
                reg->sendSwapStatus();
              break;
            case SWAPFUNCT_QRY:
              // Only Product Code can be broadcasted
              if (swPacket.destAddr == SWAP_BCAST_ADDR)
              {
                if (swPacket.regId != REGI_PRODUCTCODE)
                  break;
              }
              // Query not addressed to us?
              else if (swPacket.destAddr != commstack.cc1101.devAddress)
                break;
              // Current version does not support data recording mode
              // so destination address and register address must be the same
              if (swPacket.destAddr != swPacket.regAddr)
                break;
              // Valid register?
              if ((reg = getRegister(swPacket.regId)) == NULL)
                break;
              reg->getData();
              break;
            case SWAPFUNCT_STA:
              // User callback function declared?
              if (commstack.statusReceived != NULL)
                commstack.statusReceived(&swPacket);
              break;
            default:
              break;
        }
      }else{
        Serial.println("CRC ERR");
      }
    }
  }
  // Enable interrupt
  enableIRQ_GDO0();
}

/**
 * commstack
 *
 * Class constructor
 */
SPAXSTACK::SPAXSTACK(void)
{
  statusReceived = NULL;
  repeater = NULL;
  // a flag that a wireless packet has been received
  packetAvailable = false;
  bEnterSleep = false;
  f_wdt = 0;
  seqNo = 0;
  bDebug = false;
}

/**
 * enableRepeater
 *
 * Enable repeater mode
 *
 * 'maxHop'  MAximum repeater count. Zero if omitted
 */
void SPAXSTACK::enableRepeater(byte maxHop)
{
  if (repeater == NULL)
  {
    static REPEATER repe;
    repeater = &repe;
    repeater->init(maxHop);
  }

  if (maxHop == 0)
    repeater->enabled = false;
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
  if (regId >= regTableSize)
    return NULL;
  return regTable[regId]; 
}

/** ----------- ISR section --------- **/
/* Handle interrupt from CC1101 (INT0) gdo0 on pin2 */
void cc1101Interrupt(void){
// set the flag that a package is available
  sleep_disable();
  detachInterrupt(0);
  commstack.packetAvailable = true;
}

void enterDeepSleepWithRx(){
      //bring AVR to sleep. It will be woken up by the radio on packet receive

    WDTCSR |= (1<<WDCE) | (1<<WDE);
    //WDTCSR = 1<<WDP0 | 1<<WDP3; // set new watchdog timeout prescaler to 8.0 seconds    
    WDTCSR = WDPS_1S ;
    WDTCSR |= _BV(WDIE); // Enable the WD interrupt (no reset)
    //cc1101.setPowerDownState();
    
    sleep_enable();
    attachInterrupt(0, cc1101Interrupt, HIGH);
    /* 0, 1, or many lines of code here */
    set_sleep_mode(SLEEP_MODE_PWR_DOWN);
    cli();
    sleep_bod_disable();
    sei();
    sleep_cpu();
    // wake up here ->
    sleep_disable();
}

void SPAXSTACK::enterSleep(){
  enableIRQ_GDO0();
  if (bEnterSleep) enterDeepSleepWithRx();
}


/**
 * init
 * 
 * Initialize commstack board
 */
void SPAXSTACK::init() 
{
  // Calibrate internal RC oscillator
  rtcCrystal = rcOscCalibrate();

  // Setup CC1101
  cc1101.init();
  //first start after flashing, the EEPROM is not set yet
  //freq offset must be set to 0
  if (cc1101.offset_freq0 == 0xFF && cc1101.offset_freq1 == 0xFF ){ 
    Serial.print("Resetting freq regs to 0"); 
    cc1101.adjustFreq(0x00, 0x00 ,true);
  }  
  cc1101.setSyncWord(SYNCWORD1, SYNCWORD0);
  cc1101.setDevAddress(0xFF, false);
  cc1101.setCarrierFreq(CFREQ_433);
  cc1101.disableAddressCheck(); //if not specified, will only display "packet received"

  // Read periodic Tx interval from EEPROM
  txInterval[0] = EEPROM.read(EEPROM_TX_INTERVAL);
  txInterval[1] = EEPROM.read(EEPROM_TX_INTERVAL + 1);

  delayMicroseconds(50);  

  // Enter RX state
  cc1101.setRxState();

  // Attach callback function for GDO0 (INT0)
  enableIRQ_GDO0();

  // Default values
  sentPacketNo = 0;
  errorCode = 0;
  stackState = SYSTATE_RXON;
}


/**
 * reset
 * 
 * Reset commstack
 */
void SPAXSTACK::reset() 
{
  // Tell the network that our spaxxity is restarting
  stackState = SYSTATE_RESTART;
  // Reset commstack
  wdt_disable();  
  wdt_enable(WDTO_15MS);
  while (1) {}
}

/**
 * sleepWd
 * 
 * Put commstack into Power-down state during "time".
 * This function uses the internal watchdog timer in order to exit (interrupt)
 * from the power-down state
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
void SPAXSTACK::sleepWd(byte time) 
{
  // Power-down CC1101
  cc1101.setPowerDownState();
  // Power-down commstack
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  sleep_enable();
  setup_watchdog(time);
  delayMicroseconds(10);
  // Disable ADC
  ADCSRA &= ~(1 << ADEN);
  // Unpower functions
  PRR = 0xFF;
  //power_all_disable();
  //clock_prescale_set(clock_div_8);
  // Enter sleep mode
  sleep_mode();

  // ZZZZZZZZ...

  // Wake-up!!
  wakeUp(false);
}

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
void SPAXSTACK::sleepRtc(byte time) 
{
  // Power-down CC1101
  cc1101.setPowerDownState();
  // Power-down commstack
  set_sleep_mode(SLEEP_MODE_PWR_SAVE);
  sleep_enable();
  setup_rtc(time);
  delayMicroseconds(10);
  // Disable ADC
  ADCSRA &= ~(1 << ADEN);
  // Unpower functions
  PRR = 0xFF;
  // Enter sleep mode
  sleep_mode();

  // ZZZZZZZZ...

  // Wake-up!!
  wakeUp(false);
}

/**
 * wakeUp
 *
 * Wake from sleep mode
 *
 * 'rxOn' Enter RX_ON state after waking up
 */
void SPAXSTACK::wakeUp(bool rxOn) 
{
  // Exit from sleep
  sleep_disable();
  //wdt_disable();
  // Re-enable functions
  //clock_prescale_set(clock_div_1);
  power_all_enable();
  // Enable ADC
  ADCSRA |= (1 << ADEN);
  
  // If 32.768 KHz crystal enabled
  if (rtcCrystal)
  {
    // Disable timer2A overflow interrupt
    TIMSK2 = 0x00;
  }

  // Reset CC1101 IC
  cc1101.wakeUp();

  if (rxOn)
    stackState = SYSTATE_RXON;
}

/**
 * goToSleep
 *
 * Sleep whilst in power-down mode. This function currently uses sleepWd in a loop
 */
void SPAXSTACK::enterSleepWithRadioOff(void)
{
  // Get the amount of seconds to sleep from the internal register
  int intInterval = txInterval[0] * 0x100 + txInterval[1];
  int i, loops;
  byte minTime;
  
  // No interval? Then return
  if (intInterval == 0)
    return;

  // Search the maximum sleep time passed as argument to sleepWd that best
  // suits our desired interval
  if (intInterval % 8 == 0)
  {
    loops = intInterval / 8;
    
    if (rtcCrystal)
      minTime = RTC_8S;
    else
      minTime = WDTO_8S;
  }
  else if (intInterval % 4 == 0)
  {
    if (rtcCrystal)
    {
      loops = intInterval / 2;
      minTime = RTC_2S;
    }
    else
    {
      loops = intInterval / 4;
      minTime = WDTO_4S;
    }
  }
  else if (intInterval % 2 == 0)
  {
    loops = intInterval / 2;
    if (rtcCrystal)    
      minTime = RTC_2S;
    else
      minTime = WDTO_2S;
  }
  else
  {
    loops = intInterval;
    if (rtcCrystal)
      minTime = RTC_1S;
    else
      minTime = WDTO_1S;
  }

  stackState = SYSTATE_RXOFF;

  // Sleep
  for (i=0 ; i<loops ; i++)
  {
    // Exit sleeping loop?
    if (stackState == SYSTATE_RXON)
      break;

    if (rtcCrystal)
      sleepRtc(minTime);
    else
      sleepWd(minTime);
  }
  stackState = SYSTATE_RXON;
}


/**
 * getInternalTemp
 * 
 * Read internal (ATMEGA328 only) temperature sensor
 * Reference: http://playground.arduino.cc/Main/InternalTemperatureSensor
 * 
 * Return:
 * 	Temperature in degrees Celsius
 */
long SPAXSTACK::getInternalTemp(void) 
{
  unsigned int wADC;
  long t;

  // The internal temperature has to be used
  // with the internal reference of 1.1V.
  // Channel 8 can not be selected with
  // the analogRead function yet.

  // Set the internal reference and mux.
  ADMUX = (_BV(REFS1) | _BV(REFS0) | _BV(MUX3));
  ADCSRA |= _BV(ADEN);  // enable the ADC

  delay(20);            // wait for voltages to become stable.

  ADCSRA |= _BV(ADSC);  // Start the ADC

  // Detect end-of-conversion
  while (bit_is_set(ADCSRA,ADSC));

  // Reading register "ADCW" takes care of how to read ADCL and ADCH.
  wADC = ADCW;

  // The offset of 324.31 could be wrong. It is just an indication.
  t = (wADC - 324.31 ) / 1.22;

  // The returned temperature is in degrees Celcius.
  return (t);
}

/**
 * setTxInterval
 * 
 * Set interval for periodic transmissions
 * 
 * 'interval'	New periodic interval. 0 for asynchronous devices
 * 'save'     If TRUE, save parameter in EEPROM
 */
void SPAXSTACK::setTxInterval(byte* interval, bool save)
{
  memcpy(txInterval, interval, sizeof(txInterval));

  // Save in EEPROM
  if (save)
  {
    EEPROM.write(EEPROM_TX_INTERVAL, interval[0]);
    EEPROM.write(EEPROM_TX_INTERVAL + 1, interval[1]);
  }
}


void SPAXSTACK::sendAck(void){
  SWACK swack(master_address);  

};

/**
 * waitState
 * 
 * Waits that the RX IRQ sets the stack state to the given state. 
 * 
 * 'state'	Expected state
 */
bool SPAXSTACK::waitState(cor_state* cs){  
  while (cs->wait_resp_timeout-- != 0 ){
    if (stackState == cs->state) return true;  
    showActivity(); 
  }
  return false;
}

/**
 * getAddress
 * 
 * Sends a broadcast request for a device address. When addr is received, sets it and enables cc1101 packet filtering
 * 
 */
bool SPAXSTACK::getAddress(void)
{
  // Broadcast addr request
  byte retry = 0;
  cor_state cs = {MAX_WAIT_RESPONSE, STACKSTATE_READY};
  while (retry++ > MAX_RETRY_SEND_DATA){
    stackState = STACKSTATE_WAIT_CONFIG;
    //SWSTATUS packet = SWSTATUS(REGI_DEVADDRESS, 0, length);
    SWQUERY(0,0,REGI_DEVADDRESS).send();
    //Wait for a response. When the status is set to SYSTATE_READY, all went fine
    if (waitState(&cs)){
      SERIAL_DEBUGC("Got address");
      SERIAL_DEBUG(cc1101.devAddress);
      return true;
    }
  }
  // if we're here, no response was received 
  //for the address request broadcast query
  SERIAL_DEBUG("No addr");
  return false;
}

/**
 * ping
 * 
 * Checks if a server can be reached
 */
bool SPAXSTACK::ping(void) {
  SWQUERY(0,0,REGI_DEVADDRESS).send(); //stack state is set to  STACKSTATE_WAIT_ACK
  cor_state cs = {MAX_WAIT_RESPONSE, STACKSTATE_READY};
  return waitState(&cs);
}

/**
 * Pre-instantiate SPAXSTACK object
 */
SPAXSTACK commstack;