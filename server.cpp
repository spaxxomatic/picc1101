/******************************************************************************/
/* PiCC1101  - Radio serial link using CC1101 module and Raspberry-Pi         */
/*                                                                            */
/* KISS AX.25 blocks handling                                                 */
/*                                                                            */
/*                      (c) Edouard Griffiths, F4EXB, 2015                    */
/*                                                                            */
/******************************************************************************/

#include <string.h>
#include <sys/time.h>

#include "server.h"
#include <stddef.h>
#include "lib/spaxstack/swpacket.h"

#include "lib/radio/radio.h"
#include "util.h"


// === Public functions ===========================================================================

// ------------------------------------------------------------------------------------------------
// Initialize the common parameters to defaults
void server_init(arguments_t *arguments)
// ------------------------------------------------------------------------------------------------
{

}


uint8_t server_command(uint8_t *block)
// ------------------------------------------------------------------------------------------------
{
    uint8_t command_code = block[1] & 0x0F;
    uint8_t kiss_port = (block[1] & 0xF0)>>4;
    uint8_t command_arg = block[2];

    verbprintf(4, "Command %02X %02X\n", block[1], block[2]);

    switch (command_code)
    {
        case 0: // data block
            return 0;
        case 1: // TXDELAY
            //tnc_tx_keyup_delay = command_arg * 10000; // these are tenths of ms
            break;
        case 2: // Persistence parameter
            //kiss_persistence = (command_arg + 1) / 256.0;
            break;
        case 3: // Slot time
            //kiss_slot_time = command_arg * 10000; // these are tenths of ms
            break;
        default:
            break;
    }
    return 1;
}

// ------------------------------------------------------------------------------------------------
// Run the server 
void server_run(serial_t *serial_parms, spi_parms_t *spi_parms, arguments_t *arguments)
// ------------------------------------------------------------------------------------------------
{
   /*
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
      */
//#############################################    
    static const size_t   bufsize = RADIO_BUFSIZE;
    uint32_t timeout_value;
    uint8_t  rx_buffer[bufsize], tx_buffer[bufsize];
    uint8_t  rtx_toggle; // 1:Tx, 0:Rx
    uint8_t  rx_trigger; 
    uint8_t  tx_trigger; 
    uint8_t  force_mode;
    int      rx_count, tx_count, byte_count, ret;
    uint64_t timestamp;
    struct timeval tp;  

    set_serial_parameters(serial_parms, arguments);
    init_radio_int(spi_parms, arguments);
    memset(rx_buffer, 0, bufsize);
    memset(tx_buffer, 0, bufsize);
    radio_flush_fifos(spi_parms);
    
    verbprintf(1, "Starting...\n");

    force_mode = 1;
    rtx_toggle = 0;
    rx_trigger = 0;
    tx_trigger = 0;
    rx_count = 0;
    tx_count = 0;
    radio_init_rx(spi_parms); // init for new packet to receive Rx
    radio_turn_rx(spi_parms);            // Turn Rx on
    /*
    while(1)
    {    
        byte_count = radio_receive_packet(spi_parms, arguments, &rx_buffer[rx_count]); // check if anything was received on radio link

        if (byte_count > 0)
        {
            rx_count += byte_count;  // Accumulate Rx
            
            gettimeofday(&tp, NULL);
            timestamp = tp.tv_sec * 1000000ULL + tp.tv_usec;
            timeout_value = arguments->tnc_radio_window;
            force_mode = (timeout_value == 0);

            if (rtx_toggle) // Tx to Rx transition
            {
                tx_trigger = 1; // Push Tx
            }
            else
            {
                tx_trigger = 0;
            }

            radio_init_rx(spi_parms, arguments); // Init for new packet to receive
            rtx_toggle = 0;
        }

        byte_count = read_serial(serial_parms, &tx_buffer[tx_count], bufsize - tx_count);

        if (byte_count > 0)
        {
            tx_count += byte_count;  // Accumulate Tx

            gettimeofday(&tp, NULL);
            timestamp = tp.tv_sec * 1000000ULL + tp.tv_usec;
            timeout_value = arguments->tnc_serial_window;
            force_mode = (timeout_value == 0);

            if (!rtx_toggle) // Rx to Tx transition
            {
                rx_trigger = 1;
            }
            else
            {
                rx_trigger = 0;
            }

            rtx_toggle = 1;
        }

        if ((rx_count > 0) && ((rx_trigger) || (force_mode))) // Send bytes received on air to serial
        {
            radio_wait_free();            // Make sure no radio operation is in progress
            radio_turn_idle(spi_parms);   // Inhibit radio operations
            verbprintf(2, "Received %d bytes\n", rx_count);
            ret = write_serial(serial_parms, rx_buffer, rx_count);
            verbprintf(2, "Sent %d bytes on serial\n", ret);
            radio_init_rx(spi_parms, arguments); // Init for new packet to receive Rx
            radio_turn_rx(spi_parms);            // Put back into Rx
            rx_count = 0;
            rx_trigger = 0;
        }

        if ((tx_count > 0) && ((tx_trigger) || (force_mode))) // Send bytes received on serial to air 
        {

                radio_wait_free();            // Make sure no radio operation is in progress
                radio_turn_idle(spi_parms);   // Inhibit radio operations (should be superfluous since both Tx and Rx turn to IDLE after a packet has been processed)
                radio_flush_fifos(spi_parms); // Flush result of any Rx activity

                verbprintf(2, "%d bytes to send\n", tx_count);

                radio_send_packet(spi_parms, arguments, tx_buffer, tx_count);

                radio_init_rx(spi_parms, arguments); // init for new packet to receive Rx
                radio_turn_rx(spi_parms);            // put back into Rx
            tx_count = 0;
            tx_trigger = 0;            
        }

        radio_wait_a_bit(4);
        
    }
    */
}