#ifndef __MQTT_H
#define __MQTT_H


#include <stddef.h>
#include "lib/mqtt/mosquitto.h"

void mqtt_stop();
bool mqtt_init();
int mqtt_send(char* topic, char* msg);


#endif