#include "mqtt.h"
#include <stdio.h>
#include <string>
#include <regex>
#include <stdlib.h>
#include "util.h"
#include "lib/inih/inireader.h"

extern INIReader* inireader ;
extern void die(const char* msg);

struct mosquitto *mosq = NULL;

std::string subscribe_actors_topic="";
std::string subscribe_config_topic="";
std::string subscribe_radionodes_descr_topic="";
std::string client_name="";
std::string publish_to="";
// ------------------------------------------------------------------------------------------------
// Initialize the server, read ini file, init mqtt, etc
std::string get_actor_id_from_topic(char* topic){
    std::regex e (subscribe_actors_topic + "\/(.*)");   // matches words beginning by "sub"
    std::string actor_id = std::regex_replace (topic,e,"$1");
    return actor_id;
}

void handle_actor_message(std::string actor_id, const mosquitto_message *message){
    printf("handle_actor_message %s %s", actor_id, (char*) message->payload );
};

void handle_radionodes_descr(std::string actor_id, const mosquitto_message *message){
    printf("handle_radionodes_descr %s %s", actor_id, (char*) message->payload );
};

void handle_config_message(std::string actor_id, const mosquitto_message *message){
    printf("handle_config_message %s %s", actor_id, (char*) message->payload );
};

void on_message_callback(struct mosquitto *mosq, void *userdata, const struct mosquitto_message *message)
{
    if(message->payloadlen){
		verbprintf(3,"%s %s\n", message->topic, (char*) message->payload);
	}else{
		verbprintf(3,"%s (null)\n", message->topic);
	}    
    std::string topic = std::string(message->topic);
    if (topic.compare(0,subscribe_actors_topic.length(), subscribe_actors_topic)){
        //actor message incoming, needs to be routed to the corresponding device
        handle_actor_message(get_actor_id_from_topic(message->topic), message);
        return;
    }else if (topic.compare(0,subscribe_config_topic.length(), subscribe_config_topic))
    {
        handle_config_message(get_actor_id_from_topic(message->topic), message);
        return;
    }else if (topic.compare(0,subscribe_radionodes_descr_topic.length(), subscribe_radionodes_descr_topic))
    {
        handle_radionodes_descr(get_actor_id_from_topic(message->topic), message);
        return;
    };
}

void on_connect_callback(struct mosquitto *mosq, void *userdata, int result)
{
	int i;
	if(!result){
printf("%s %s %s  \n",(subscribe_actors_topic+"#").data(), subscribe_config_topic.data(), subscribe_radionodes_descr_topic.data() );
            
		/* Subscribe to broker information topics on successful connect. */
		const char* t1 = (subscribe_actors_topic + std::string("#")).data();
        const char* t2 = (subscribe_config_topic + std::string("#")).data();
        const char* t3 = (subscribe_radionodes_descr_topic + std::string("#")).data();
        printf("Subscribing to %s %s %s \n",t1, t2, t3);
        mosquitto_subscribe(mosq, NULL, t1, 1);
        mosquitto_subscribe(mosq, NULL, t2, 1);
        mosquitto_subscribe(mosq, NULL, t3, 2);
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
    mosquitto_lib_init();
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
