/******************************************************************************/
/* PiCC1101  - Radio serial link using CC1101 module and Raspberry-Pi         */
/*                                                                            */
/* Test routines                                                              */
/*                                                                            */
/*                      (c) Edouard Griffiths, F4EXB, 2015                    */
/*                                                                            */
/******************************************************************************/

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include "test.h"
#include "lib/radio/radio.h"
#include "util.h"

// === Public functions ===========================================================================

// ------------------------------------------------------------------------------------------------
// Transmission test with interrupt handling
int radio_transmit_test_int( arguments_t *arguments)
// ------------------------------------------------------------------------------------------------
{
    init_radio_int();
			
	CCPACKET test ;
	SWQUERY(0,0,0).prepare(&test);
	PI_CC_SPIWriteBurstReg(PI_CCxxx0_TXFIFO, (uint8_t *)test.data, test.length); //write the payload data
	PI_CC_SPIWriteReg(PI_CCxxx0_TXFIFO, test.length); //first pos in fifo is the packet length	
    PI_CC_SPIStrobe( PI_CCxxx0_SFTX); // Flush Tx FIFO
    
    verbprintf(0, "Sending 10 test packets of size %d\n", PI_CCxxx0_FIFO_SIZE);

    while(packets_sent < 10)
    {
        radio_wait_free(); // make sure no radio operation is in progress
        //nutiu fixme radio_send_packet( arguments, arguments->test_phrase, strlen(arguments->test_phrase));
        //radio_wait_a_bit(10);
    } 
}


