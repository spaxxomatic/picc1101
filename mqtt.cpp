#include "mqtt.h"

#include <stdio.h>
#include <pthread.h>
#include <sys/types.h>
#include <unistd.h>
#include <string>
#include <regex>
#include <stdlib.h>
#include "util.h"
#include "lib/inih/inireader.h"

#include "lib/spaxstack/spaxstack.h"

extern INIReader* inireader ;
extern void die(const char* msg);
extern bool transmit_packet(CCPACKET* p_packet);
struct mosquitto *mosq = NULL;

static std::string subscribe_actors_topic;
static std::string subscribe_config_topic;
static std::string subscribe_radionodes_descr_topic;
static std::string client_name;
static std::string publish_to;
// ------------------------------------------------------------------------------------------------
// Initialize the server, read ini file, init mqtt, etc
std::string get_actor_id_from_topic(std::string &prefix, char* topic){
    std::regex e (prefix + "(.*)");   // matches words beginning by "sub"
    std::string actor_id = std::regex_replace (topic,e,"$1");
    //printf("actor id %s", actor_id.c_str());
    return actor_id;
}

void handle_actor_message(std::string actor_id, std::string payload){
    printf("handle_actor_message %s %s\n", actor_id.c_str(), payload.c_str() );
    byte destAddr = 01;
    byte registerId = 02;
    SWCOMMAND command = SWCOMMAND(destAddr, MASTER_ADDRESS, registerId, (byte*) payload.c_str(), (byte) payload.length());
    transmit_packet(&command);
    };

void handle_radionodes_descr(std::string actor_id, std::string payload){
    printf("handle_radionodes_descr %s %s\n", actor_id.c_str(), payload.c_str() );
};

void handle_config_message(std::string actor_id, std::string payload){
    printf("handle_config_message %s %s\n", actor_id.c_str(), payload.c_str() );
};

void on_message_callback(struct mosquitto *mosq, void *userdata, const struct mosquitto_message *message)
{
    verbprintf(3,"Msg: Thread id %i\n",getpid());    
	std::string payload =std::string((char*) message->payload, message->payloadlen);
 	verbprintf(3,"Topic: %s Payload: %s\n", message->topic, payload.c_str());
	std::string topic = std::string(message->topic);
    if (topic.compare(0,subscribe_actors_topic.length(), subscribe_actors_topic)==0){
        //actor message incoming, needs to be routed to the corresponding device
        handle_actor_message(get_actor_id_from_topic(subscribe_actors_topic,message->topic), payload);
        return;
    }else if (topic.compare(0,subscribe_config_topic.length(), subscribe_config_topic)==0)
    {
        handle_config_message(get_actor_id_from_topic(subscribe_config_topic,message->topic), payload);
        return;
    }else if (topic.compare(0,subscribe_radionodes_descr_topic.length(), subscribe_radionodes_descr_topic)==0)
    {
        handle_radionodes_descr(get_actor_id_from_topic(subscribe_radionodes_descr_topic,message->topic), payload);
        return;
    };
}

void on_connect_callback(struct mosquitto *mosq, void *userdata, int result)
{
	int i;
    verbprintf(3,"Connect: Thread id %i\n",getpid());    
	
	if(!result){
        /* Subscribe to broker information topics on successful connect. */
         char* t1 = strdup((subscribe_actors_topic + "#").c_str());
         char* t2 = strdup((subscribe_config_topic + "#").c_str());
         char* t3 = strdup((subscribe_radionodes_descr_topic + "#").c_str());
        verbprintf(3,"Subscribing to %s %s %s \n",t1, t2, t3);
        mosquitto_subscribe(mosq, NULL, t1, 1);
        mosquitto_subscribe(mosq, NULL, t2, 1);
        mosquitto_subscribe(mosq, NULL, t3, 2);
        free(t1); free(t2); free(t3);
	}else{
		fprintf(stderr, "Connect failed\n");
	}
}

void on_subscribe_callback(struct mosquitto *mosq, void *userdata, int mid, int qos_count, const int *granted_qos)
{
	int i;
	verbprintf(2, "Subscribed  (mid: %d): %d", mid, granted_qos[0]);
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

#define UNDEF "UNDEF"
bool mqtt_init(){
    verbprintf(3,"MQTT Init: Thread id %i\n",getpid());    
	subscribe_actors_topic.assign(inireader->Get("mqtt","subscribe_actors_topic", UNDEF));
    if(subscribe_actors_topic == UNDEF){
        die("Missing subscribe_actors_topic in mqtt section of ini file");
    }
    subscribe_config_topic.assign(inireader->Get("mqtt","subscribe_config_topic", UNDEF));
    if(subscribe_config_topic == UNDEF){
        die("Missing subscribe_config_topic in mqtt section of ini file");
    }    
    subscribe_radionodes_descr_topic.assign(inireader->Get("mqtt","subscribe_radionodes_descr_topic", UNDEF));    
    if(subscribe_radionodes_descr_topic == UNDEF){
        die("Missing subscribe_radionodes_descr_topic in mqtt section of ini file");
    }    
    client_name = inireader->Get("mqtt","subscribe_radionodes_descr_topic", "SPAXXSERVER");    
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
