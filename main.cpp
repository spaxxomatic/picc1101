/******************************************************************************/
/* Radio server using CC1101 on Raspberry-Pi 
/*                      (c) Edouard Griffiths, F4EXB, 2015                    
/*                      (c) Lucian Nutiu, 2018
/******************************************************************************/

#include <stdio.h>      // standard input / output functions
#include <stdlib.h>
#include <iostream>
#include <argp.h>
#include <string.h>
#include <signal.h>

#include "lib/radio/params.h"
#include "util.h"
#include "lib/radio/pi_cc_spi.h"
#include "lib/radio/radio.h"
#include "server.h"
#include "test.h"

arguments_t   arguments;

using namespace std;

/***** Argp configuration start *****/

const char *argp_program_version = "SPAXSTACK 0.1";
static char doc[] = "Spaxstack -- Raspberry Pi packet radio link using the CC1101 module";
static char args_doc[] = "";

static struct argp_option options[] = {
    {"verbose",  'v', "VERBOSITY_LEVEL", 0, "Verbosity level: 0 quiet else verbose level (default : quiet)"},
    {"real-time",  'T', 0, 0, "Engage so called \"real time\" scheduling (defalut 0: no)"},
    {"ini",  'i', "INI_FILE", 0, "INI file, (default : /.spaxxserver.ini)"},
    {"radio-status",  's', 0, 0, "Print radio status and exit"},
    {0}
};

static void delete_args(arguments_t *arguments);

// ------------------------------------------------------------------------------------------------
// Terminator
static void terminate(const int signal_) {
// ------------------------------------------------------------------------------------------------
    printf("PICC: Terminating with signal %d\n", signal_);
    delete_args(&arguments);
    exit(1);
}

// ------------------------------------------------------------------------------------------------
// Init arguments
static void init_args(arguments_t *arguments)
// ------------------------------------------------------------------------------------------------
{
    arguments->verbose_level = 0;
    arguments->print_radio_status = 0;
    arguments->real_time = 0;
}

// ------------------------------------------------------------------------------------------------
// Delete arguments
void delete_args(arguments_t *arguments)
// ------------------------------------------------------------------------------------------------
{
    if (arguments->ini_file)
    {
        free(arguments->ini_file);
    }
}

// ------------------------------------------------------------------------------------------------
// Print MFSK data
static void print_args(arguments_t *arguments)
// ------------------------------------------------------------------------------------------------
{
    fprintf(stderr, "-- options --\n");
    fprintf(stderr, "Verbosity ...........: %d\n", arguments->verbose_level);
    fprintf(stderr, "Real time ...........: %s\n", (arguments->real_time ? "yes" : "no"));
    fprintf(stderr, "Ini file ...........: %s\n", arguments->ini_file );
}

// ------------------------------------------------------------------------------------------------
// Option parser 
static error_t parse_opt (int key, char *arg, struct argp_state *state)
// ------------------------------------------------------------------------------------------------
{
    arguments_t *arguments = (arguments_t*) state->input;
    char        *end;  // Used to indicate if ASCII to int was successful
    uint8_t     i8;
    uint32_t    i32;

    switch (key){
        // Verbosity 
        case 'v':
            arguments->verbose_level = strtol(arg, &end, 10);
            if (*end)
                argp_usage(state);
            else
                verbose_level = arguments->verbose_level;
            break; 
        // Real time scheduling
        case 'T':
            if (ALLOW_REAL_TIME)
            {
                arguments->real_time = 1;
            }
            else
            {
                fprintf(stderr, "Real time scheduling is not allowed\n");
            }
            break;
        // INI file
        case 'i':
            arguments->ini_file = strdup(arg);
            break;
        // Print radio status and exit
        case 's':
            arguments->print_radio_status = 1;
            break;
        default:
            return ARGP_ERR_UNKNOWN;
    }

    return 0;
}

// ------------------------------------------------------------------------------------------------
static struct argp argp = {options, parse_opt, args_doc, doc};
// ------------------------------------------------------------------------------------------------

/***** ARGP configuration stop *****/

// ------------------------------------------------------------------------------------------------
int main (int argc, char **argv)
// ------------------------------------------------------------------------------------------------
{
    int i, ret;
    // unsolicited termination handling
    struct sigaction sa;
    // Catch all signals possible on process exit!
    for (i = 1; i < 64; i++) 
    {
        // skip SIGUSR2 for Wiring Pi
        if (i == 17)
            continue; 

        // These are uncatchable or harmless or we want a core dump (SEGV) 
        if (i != SIGKILL
            && i != SIGSEGV
            && i != SIGSTOP
            && i != SIGHUP
            && i != SIGVTALRM
            && i != SIGWINCH
            && i != SIGPROF) 
        {
            memset(&sa, 0, sizeof(sa));
            sa.sa_handler = terminate;
            sigaction(i, &sa, NULL);
        }
    }

    // Set argument defaults
    init_args(&arguments); 

    // Parse arguments 
    argp_parse (&argp, argc, argv, 0, 0, &arguments);
    
    if (!arguments.ini_file)
    {
        arguments.ini_file = strdup(DEF_INI_FILE);
    }

    print_args(&arguments);
    
    readIniFile(arguments.ini_file);
	const char* spi_device = inireader->Get("SPI","device", "/dev/spidev0.0").c_str();
  
    setup_spi(&arguments, spi_device);
    
    server_run( &arguments);    
    
    delete_args(&arguments);
    return 0;
}
