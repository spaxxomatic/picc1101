#include "mqtt.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "util.h"
#include "lib/inih/inireader.h"
extern INIReader* inireader ;

struct mosquitto *mosq = NULL;

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
		mosquitto_subscribe(mosq, NULL, "CCRADIO/#", 1);
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

void mqtt_stop(){
    mosquitto_disconnect(mosq);
    mosquitto_loop_stop(mosq, true);
    mosquitto_destroy(mosq);  
    mosquitto_lib_cleanup();
}

bool mqtt_init(){
    mosquitto_lib_init();
    mosq = mosquitto_new(NULL, true, NULL);
    if(!mosq){
        fprintf(stderr, "Error: Out of memory.\n");
        return false;
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
        return false;
    }

    if (mosquitto_loop_start(mosq) != MOSQ_ERR_SUCCESS){
        fprintf(stderr, "Unable to start mosquitto loop.\n");
        return false;
    };
    return true;
};

int mqtt_send(char* topic, char* msg){
    return mosquitto_publish(mosq, NULL, topic, strlen(msg), msg, 1, true);
}
