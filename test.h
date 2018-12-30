/******************************************************************************/
/* PiCC1101  - Radio serial link using CC1101 module and Raspberry-Pi         */
/*                                                                            */
/* Test routines                                                              */
/*                                                                            */
/*                      (c) Edouard Griffiths, F4EXB, 2015                    */
/*                                                                            */
/******************************************************************************/

#ifndef _TEST_H_
#define _TEST_H_

#include "lib/radio/params.h"
#include "lib/radio/pi_cc_spi.h"
#include "lib/radio/radio.h"

int  radio_transmit_test_int( arguments_t *arguments);
int  radio_receive_test_int( arguments_t *arguments);

#endif