#include "mqttClient.h"

mqttClient::mqttClient(const char * _id,const char * _topic, const char * _host, int _port) : mosquittopp(_id)
{
 mosqpp::lib_init();        // Mandatory initialization for mosquitto library
 this->keepalive = 60;    // Basic configuration setup for myMosq class
 this->id = _id;
 this->port = _port;
 this->host = _host;
 this->topic = _topic;
 connect_async(host,     // non blocking connection to broker request
 port,
 keepalive);
 loop_start();            // Start thread managing connection / publish / subscribe
};