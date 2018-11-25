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

#include <stdlib.h>
#include <stdio.h>
#include <signal.h>
#include "mqtt.h"

const char* inifile = "spaxxserver.ini";
INIReader* inireader ;
extern uint8_t radio_process_packet();

void server_shutdown(); 
// === Public functions ===========================================================================
void die(const char* msg){
  fprintf(stderr, "%s\n", msg);
  server_shutdown();
  exit(1);
}

void sig_handler(int signo)
{
  if (signo == SIGINT){
    die("Received SIGINT\n");
  }
}

void server_shutdown(){
  sem_destroy(&sem_radio_irq);
  mqtt_stop();
  delete inireader;
}

void readIniFile(){
      //read ini file
    if (inireader == NULL){
      inireader = new INIReader(inifile);
      if (inireader->ParseError() < 0) {
          fprintf(stderr, "%s missing or corrupt\n", inifile);
          die("Cannot continue");
      }
      verbprintf(1, "Config loaded from ", inifile);
    }
}

void server_init(arguments_t *arguments)
{
  if (signal(SIGINT, sig_handler) == SIG_ERR) printf("\ncan't catch SIGINT\n");
  readIniFile();
  if (!mqtt_init()) die("Mqtt failure, exiting");
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
void server_run(arguments_t *arguments)
// ------------------------------------------------------------------------------------------------
{
    uint32_t timeout_value;
    uint64_t timestamp;
    struct timeval tp;  

    init_radio_int(arguments);
    radio_flush_fifos();
    
    verbprintf(1, "Starting...\n");
    server_init(arguments);
    //connect to mqtt broker

    //enable radio rx
    radio_init_rx(); // init for new packet to receive Rx
    radio_turn_rx(); // Turn Rx on
    sem_init(&sem_radio_irq, 0, 0);
    //server loop
    while(1){
        sem_wait(&sem_radio_irq); //waiting for radio data in this thread
        if (radio_process_packet()){
             mqtt_send("/CCRADIO/MSG","Got a packet");
        };        
        tx_handler();
      }
}