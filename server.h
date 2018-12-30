/******************************************************************************/
/*                                                                            */
/******************************************************************************/
#ifndef _SERVER_H_
#define _SERVER_H_

#include <stdint.h>
#include <stdlib.h>

#include "lib/radio/params.h"
#include "lib/radio/pi_cc_spi.h"
#include "lib/inih/inireader.h"

void server_run( arguments_t *arguments);
void server_init(arguments_t *arguments);
void die(char* msg);
void readIniFile();
int registerNewNode();
#endif