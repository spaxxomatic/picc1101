#ifndef __MQTT_H
#define __MQTT_H

#include <sys/types.h>
#include <stdio.h>
#include <stdlib.h>
#include <stddef.h>
#include <string>
#include "lib/types.h"
#include "lib/mqtt/mosquitto.h"
#include "lib/spaxstack/swpacket.h"

void mqtt_stop();
bool mqtt_init();
void mqtt_send(char* topic, char* msg);
void mqtt_send_actor_state(int actor_id, int register_addr, SWDATA* data);
void mqtt_send_alarm(int actor_id, const char* alarm_text);
void mqtt_send_avail(int actor_id, bool avail); 

enum MQTT_ERRORCODES 
{
    MQTT_OK = 0, //all ok
    MQTT_ERR_INVALID_ACTOR_ID, //actor id is out of range
    MQTT_ERR_INVALID_PAYLOAD,
    MQTT_ERR_INVALID_REGISTER_ID,
    MQTT_ERR_INVALID_REGISTER_VAL,
    MQTT_ERR_INVALID_TOPIC
};

#endif