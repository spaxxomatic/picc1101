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

#include <sys/time.h>

INIReader* inireader;

extern uint8_t radio_process_packet();

void server_shutdown(); 
// === Public functions ===========================================================================
void die(const char* msg){
  fprintf(stderr, "%s\n", msg);
  server_shutdown();
  exit(1);
}


void second_tick  (int signum)
{
 static int count = 0;
 //printf (" periodic task in C++ timer %d \n", ++count);
 ackAwaitQueue.trigger();
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

void readIniFile(char* inifile){
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
  readIniFile(arguments->ini_file);
  init_radio_int();
  
  int ret = reset_radio();
  if (ret != 0) die("Cannot initialize radio link\n");
  //connect to mqtt broker
  if (!mqtt_init()) die("Mqtt failure, exiting\n");

  sem_init(&sem_radio_irq, 0, 0);

  struct sigaction sa;
  struct itimerval timer;

  /* Install periodic_task as the signal second_tick for SIGALRM. */
  if (signal(SIGALRM, second_tick) == SIG_ERR) die("can't catch SIGALRM \n");
  timer.it_value.tv_sec = 1; 
  timer.it_value.tv_usec = 0;
  timer.it_interval.tv_sec = 1; //each second 
  timer.it_interval.tv_usec = 0;
  if (setitimer (ITIMER_REAL, &timer, NULL) == -1) die ("can't set timer\n");

}

// ------------------------------------------------------------------------------------------------
// Run the server 
void server_run(arguments_t *arguments)
// ------------------------------------------------------------------------------------------------
{
    uint32_t timeout_value;
    uint64_t timestamp;
    struct timeval tp;  
    verbprintf(1, "Starting...\n");
    server_init(arguments);

    //server loop
    while(1){
        sem_wait(&sem_radio_irq); //waiting for radio data in this thread
        radio_process_packet();
        //tx_handler();
      }
}