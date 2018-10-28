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
#include "mosquitto.h"

struct mosquitto *mosq = NULL;

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

void on_mqtt_connect(struct mosquitto *mosq, void *userdata, int result)
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
  mosquitto_disconnect();
  mosquitto_loop_stop(mosq, true);
	mosquitto_destroy(mosq);  
  mosquitto_lib_cleanup();
  delete ini;
}

void readIniFile(){
      //read ini file
    if (ini == nullptr){
      ini = new INIReader(inifile);
      if (ini->ParseError() < 0) {
          fprintf(stderr, "%s missing or corrupt\n", inifile);
          die("Cannot continue");
      }
      verbprintf(1, "Config loaded from ", inifile);
    }
}

void server_init()
{
  readIniFile();
  mosquitto_lib_init();
	mosq = mosquitto_new(NULL, true, NULL);
	if(!mosq){
		fprintf(stderr, "Error: Out of memory.\n");
		return 1;
	}
	mosquitto_log_callback_set(mosq, on_log_callback);
	mosquitto_connect_callback_set(mosq, on_connect_callback);
	mosquitto_message_callback_set(mosq, on_message_callback);
  
	if(mosquitto_connect(mosq, ini->Get("mqtt", "broker_ip", "localhost"),
      ini->GetInteger("mqtt", "broker_port", 1833),
      ini->GetInteger("mqtt", "keepalive", 60)
      ) != MOSQ_ERR_SUCCESS
    ){
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

uint8_t radio_process_packet(){
    if (radio_int_data.rx_buff_idx != radio_int_data.rx_buff_read_idx){ //packets have been received and are waiting processing
        CCPACKET packet = radio_int_data.rx_buf[radio_int_data.rx_buff_read_idx++];
    }
}
// ------------------------------------------------------------------------------------------------
// Run the server 
void server_run(serial_t *serial_parms, spi_parms_t *spi_parms, arguments_t *arguments)
// ------------------------------------------------------------------------------------------------
{
    uint32_t timeout_value;
    uint64_t timestamp;
    struct timeval tp;  

    set_serial_parameters(serial_parms, arguments);
    init_radio_int(spi_parms, arguments);
    radio_flush_fifos(spi_parms);
    
    verbprintf(1, "Starting...\n");
    server_init();
    //connect to mqtt broker

    //enable radio rx
    radio_init_rx(spi_parms); // init for new packet to receive Rx
    radio_turn_rx(spi_parms);            // Turn Rx on

    //server loop
    while(1){
        radio_process_packet();
        tx_handler();
      }
    }
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