#include "mqtt.h"

#include <stdio.h>
#include <pthread.h>
#include <sys/types.h>
#include <unistd.h>
#include <iostream>
#include <string>
#include <regex>
#include <stdlib.h>
#include "util.h"
#include "lib/inih/inireader.h"

#include "lib/spaxstack/spaxstack.h"
#include "lib/radio/radio.h"

extern INIReader* inireader ;
extern void die(const char* msg);
extern bool transmit_packet(SWPACKET* p_packet);
struct mosquitto *mosq = NULL;

static std::string subscribe_actors_topic;
static std::string subscribe_config_topic;
static std::string subscribe_radionodes_stat_topic;
static std::string subscribe_stat_topic;
static std::string publish_status_topic;
static std::string errorlog_topic;
static std::string client_name;
static std::string publish_to;

#define CK_VALID_UINT8_T(V, ERR_CODE)  if (V <0 || V  >= 0xFF) return ERR_CODE;
// ------------------------------------------------------------------------------------------------
// Initialize the server, read ini file, init mqtt, etc
int get_actor_id_from_topic(std::string &prefix, char* topic){
    std::regex e (prefix + "(.*)");  
    std::string actor_id = std::regex_replace (topic,e,"$1");
    return std::stoi(actor_id);     // raises an exception if not an int
}

typedef struct  {
    char regId;
    char regVal;
} payloadRegister ;

const char* MQTT_ERR_MSG[MQTT_ERR_INVALID_REGISTER_VAL+1] = {
    NULL, 
    "Invalid actor ID", 
    "Invalid payload. Should be in format <register_addr>:<register_value>",
    "Invalid register ID", 
    "Invalid register value", 
};

class mqtt_msg_ex: public exception
{
  public:
  const char* reason;
  mqtt_msg_ex(const char* r) {
      this->reason = r;
  }
  virtual const char* what() const throw()
  {
    return reason;
  }
} ;

int mqtt_send_actor_state(int actor_id, int register_addr, SWDATA* data){    
    std::ostringstream topic;
    topic << publish_status_topic <<  std::to_string(actor_id) << std::to_string(register_addr) ;
    return mosquitto_publish(mosq, NULL, topic.str().c_str(), data->length, data->data, 2, true);
}

int mqtt_send(const char* topic, const char* msg){
    return mosquitto_publish(mosq, NULL, topic, strlen(msg), msg, 1, true);
}

bool decode_actor_payload(std::string &payload, payloadRegister* ret){
    std::regex e ("([0-9]+):([0-9]+)");  
    std::smatch pieces_match;
    if (std::regex_match(payload, pieces_match, e)) {
        if (pieces_match.size() != 3){
            return false;
        }else{
                //std::ssub_match sub_match = pieces_match[i];
            std::ssub_match p1 = pieces_match[1];
            std::ssub_match p2 = pieces_match[2];
            std::string rAddr = p1.str();
            std::string rVal = p2.str();
            std::cout << "  Reg: " << rAddr << " Val: " << rVal << '\n';  
            ret->regId = std::stoi( rAddr ); 
            ret->regVal = std::stoi( rVal );
            return true;
        }   
    }
}


int handle_actor_message(uint8_t actor_id, std::string payload){
    //printf("handle_actor_message %i %s\n", actor_id, payload.c_str() );    
    payloadRegister res ;
    if (! decode_actor_payload(payload, &res)){
            return MQTT_ERR_INVALID_PAYLOAD;
    };
    SWCOMMAND command = SWCOMMAND(actor_id, actor_id, res.regId, &res.regVal, 1);
    transmit_packet(&command);
    return MQTT_OK;
}    

int handle_radionodes_descr(uint8_t actor_id, std::string payload){
    //printf("handle_radionodes_descr %i %s\n", actor_id, payload.c_str() );    
    int regId = std::stoi(payload); 
    CK_VALID_UINT8_T(regId,MQTT_ERR_INVALID_REGISTER_ID);
    SWQUERY query = SWQUERY(actor_id, actor_id, regId);
    transmit_packet(&query);
    
    return MQTT_OK;
};

int handle_config_message(uint8_t actor_id, std::string payload){
    //printf("handle_config_message %i %s\n", actor_id, payload.c_str() );
    return MQTT_OK;
};

int handle_stat_message(){
    printf("handle_stat_message \n");
    printf("--- Radio state ---\n");
    print_radio_status();
    return MQTT_OK;
};

void logError(const std::string &topic, const std::string &payload, const char* source, const char* errmsg){
    std::ostringstream stringStream;
    stringStream << "ERROR: mqtt exception handling msg " << topic << " with payload >" << payload << "< : " << source << " " << errmsg;
    std::string em = stringStream.str();
    fprintf(stderr, "%s\n",em.c_str());
    //log the error on the mqtt server
    mqtt_send(errorlog_topic.c_str(), em.c_str());    
}

void check_result(int mqtt_handle_res){
    if (mqtt_handle_res != MQTT_OK){
        const char* errmsg = MQTT_ERR_MSG[mqtt_handle_res];
        throw mqtt_msg_ex(errmsg);
    };
}


void on_message_callback(struct mosquitto *mosq, void *userdata, const struct mosquitto_message *message)
{
    verbprintf(3,"Msg: Thread id %i\n",getpid());    
	std::string payload = std::string((char*) message->payload, message->payloadlen);
 	verbprintf(3,"Topic: %s Payload: %s\n", message->topic, payload.c_str());
	std::string topic = std::string(message->topic);
    std::string errmsg ;
    try{
        if (topic.compare(0,subscribe_actors_topic.length(), subscribe_actors_topic)==0){
            //actor message incoming, needs to be routed to the corresponding device
            check_result(
                handle_actor_message(get_actor_id_from_topic(subscribe_actors_topic,message->topic), payload)
            ); return ;
        }else if (topic.compare(0,subscribe_config_topic.length(), subscribe_config_topic)==0)
        {
            check_result(
                handle_config_message(get_actor_id_from_topic(subscribe_config_topic,message->topic), payload)
            ); return ;
        }else if (topic.compare(0,subscribe_radionodes_stat_topic.length(), subscribe_radionodes_stat_topic)==0)
        {
            check_result( 
                handle_radionodes_descr(get_actor_id_from_topic(subscribe_radionodes_stat_topic,message->topic), payload)
            ); return ;
        }else if (topic.compare(0,subscribe_stat_topic.length(), subscribe_stat_topic)==0)
        {
            check_result(
            handle_stat_message()
            ); return ;
        }
    } catch (const std::invalid_argument& ex){
        logError(topic, payload, "Invalid argument: ", ex.what());
    } catch (const std::exception& ex) {    
        logError(topic, payload, "Generic exception: ", ex.what());
    }

}

void subscribe_topic(std::string topic, int qos){
    char* tp = strdup((topic + "#").c_str());
    verbprintf(3,"Subscribing to %s\n",tp);
    mosquitto_subscribe(mosq, NULL, tp, qos);
    free(tp);
}

void on_connect_callback(struct mosquitto *mosq, void *userdata, int result)
{
	int i;
    verbprintf(3,"Connect: Thread id %i\n",getpid());	
	if(!result){
        /* Subscribe to broker information topics on successful connect. */
        subscribe_topic(subscribe_actors_topic,1);
        subscribe_topic(subscribe_config_topic,1);
        subscribe_topic(subscribe_radionodes_stat_topic,2);
        subscribe_topic(subscribe_stat_topic,1);
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
    subscribe_stat_topic.assign(inireader->Get("mqtt","subscribe_stat_topic", UNDEF));
    if(subscribe_stat_topic == UNDEF){
        die("Missing subscribe_stat_topic in mqtt section of ini file");
    }       
    subscribe_radionodes_stat_topic.assign(inireader->Get("mqtt","subscribe_radionodes_stat_topic", UNDEF));    
    if(subscribe_radionodes_stat_topic == UNDEF){
        die("Missing subscribe_radionodes_stat_topic in mqtt section of ini file");
    }   
    
    publish_status_topic.assign(inireader->Get("mqtt","publish_status_topic", UNDEF));    
    if(publish_status_topic == UNDEF){
        die("Missing publish_status_topic in mqtt section of ini file");
    }    

    errorlog_topic.assign(inireader->Get("mqtt","errorlog_topic", UNDEF));    
    if(errorlog_topic == UNDEF){
        die("Missing errorlog_topic in mqtt section of ini file");
    }        
    
    client_name = inireader->Get("mqtt","client_name", "SPAXXSERVER");    
    mosquitto_lib_init();

    mosq = mosquitto_new(client_name.c_str(), true, NULL);
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

