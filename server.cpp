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
#include "lib/mqtt/mosquitto.h"
#include "lib/radio/radio.h"

const char* inifile = "spaxmatic.ini";
INIReader* inireader ;

struct mosquitto *mosq = NULL;
void server_shutdown(); 
// === Public functions ===========================================================================
void die(char* msg){
  fprintf(stderr, "%s\n", msg);
  server_shutdown();
  exit(1);
}

int on_mqtt_message(struct mosquitto *mosq, void *userdata, const struct mosquitto_message *msg)
{
	printf("%s %s (%d)\n", msg->topic, (const char *)msg->payload, msg->payloadlen);
	return 0;
}
// ------------------------------------------------------------------------------------------------
// Initialize the server, read ini file, init mqtt, etc

void on_message_callback(struct mosquitto *mosq, void *userdata, const struct mosquitto_message *message)
{
	if(message->payloadlen){
		printf("%s %s\n", message->topic, message->payload);
	}else{
		printf("%s (null)\n", message->topic);
	}
	fflush(stdout);
}

void on_connect_callback(struct mosquitto *mosq, void *userdata, int result)
{
	int i;
	if(!result){
		/* Subscribe to broker information topics on successful connect. */
		mosquitto_subscribe(mosq, NULL, "$SYS/#", 2);
	}else{
		fprintf(stderr, "Connect failed\n");
	}
}

void on_subscribe_callback(struct mosquitto *mosq, void *userdata, int mid, int qos_count, const int *granted_qos)
{
	int i;
	verbprintf(2, "Subscribed (mid: %d): %d", mid, granted_qos[0]);
	for(i=1; i<qos_count; i++){
		verbprintf(2, ", %d", granted_qos[i]);
	}
	verbprintf(2, "\n");
}

void on_log_callback(struct mosquitto *mosq, void *userdata, int level, const char *str)
{
  verbprintf(4, "mqtt log: %s\n", str);
}

void server_shutdown(){
  mosquitto_disconnect(mosq);
  mosquitto_loop_stop(mosq, true);
	mosquitto_destroy(mosq);  
  mosquitto_lib_cleanup();
  delete inireader;
}

void readIniFile(){
      //read ini file
    if (inireader == nullptr){
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
  readIniFile();
  mosquitto_lib_init();
	mosq = mosquitto_new(NULL, true, NULL);
	if(!mosq){
		fprintf(stderr, "Error: Out of memory.\n");
		return;
	}
	mosquitto_log_callback_set(mosq, on_log_callback);
	mosquitto_connect_callback_set(mosq, on_connect_callback);
	mosquitto_message_callback_set(mosq, on_message_callback);

	int conn_ret = mosquitto_connect(mosq, inireader->Get("mqtt", "broker_ip", "localhost").data(),
      inireader->GetInteger("mqtt", "broker_port", 1833),
      inireader->GetInteger("mqtt", "keepalive", 60)
      )  ;
  if( conn_ret != MOSQ_ERR_SUCCESS){
		fprintf(stderr, "Unable to connect to mqtt broker.\n");
		return ;
	}

	if (mosquitto_loop_start(mosq) != MOSQ_ERR_SUCCESS){
    fprintf(stderr, "Unable to start mosquitto loop.\n");
  };

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
void server_run(spi_parms_t *spi_parms, arguments_t *arguments)
// ------------------------------------------------------------------------------------------------
{
    uint32_t timeout_value;
    uint64_t timestamp;
    struct timeval tp;  

    init_radio_int(spi_parms, arguments);
    radio_flush_fifos(spi_parms);
    
    verbprintf(1, "Starting...\n");
    server_init(arguments);
    //connect to mqtt broker

    //enable radio rx
    radio_init_rx(spi_parms); // init for new packet to receive Rx
    radio_turn_rx(spi_parms); // Turn Rx on

    //server loop
    while(1){
        tx_handler(spi_parms);
      }
}