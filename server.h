/******************************************************************************/
/* PiCC1101  - Radio serial link using CC1101 module and Raspberry-Pi         */
/*                                                                            */
/* KISS AX.25 blocks handling                                                 */
/*                                                                            */
/*                      (c) Edouard Griffiths, F4EXB, 2015                    */
/*                                                                            */
/******************************************************************************/
#ifndef _KISS_H_
#define _KISS_H_

#include <stdint.h>
#include <stdlib.h>

#include "main.h"
#include "pi_cc_spi.h"
#include "serial.h"

void server_run(serial_t *serial_parms, spi_parms_t *spi_parms, arguments_t *arguments);
void server_init(arguments_t *arguments);

#endif