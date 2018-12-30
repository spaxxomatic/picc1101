#ifndef __MQTT_H
#define __MQTT_H


#include <stddef.h>
#include "lib/mqtt/mosquitto.h"

void mqtt_stop();
bool mqtt_init();
int mqtt_send(char* topic, char* msg);

enum MQTT_ERRORCODES 
{
    MQTT_OK = 0, //all ok
    MQTT_ERR_INVALID_ACTOR_ID, //actor id is out of range
    MQTT_ERR_INVALID_PAYLOAD,
    MQTT_ERR_INVALID_REGISTER_ID,
    MQTT_ERR_INVALID_REGISTER_VAL
};

const char* MQTT_ERR_MSG[MQTT_ERR_INVALID_REGISTER_VAL+1] = {
    NULL, 
    "Invalid actor ID", 
    "Invalid payload. Should be in format <register_addr>:<register_value>",
    "Invalid register ID", 
    "Invalid register value", 
};

#endif