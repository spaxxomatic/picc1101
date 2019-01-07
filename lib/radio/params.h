#ifndef _MAIN_H_
#define _MAIN_H_

#include <inttypes.h>
#include <termios.h> 

#define ALLOW_VAR_BLOCKS 0
#define ALLOW_REAL_TIME  1

typedef struct arguments_s {
    uint8_t      verbose_level;        // Verbose level
    // --- spi link radio ---
    char         *ini_file;            // ini file name
    uint8_t      print_radio_status;   // Print radio status and exit
    uint8_t      real_time;            // Engage so called "real time" scheduling
} arguments_t;

#endif
