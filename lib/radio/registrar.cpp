#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <pthread.h>
#include <map>
#include <iterator>
#include <sstream>
#include "registrar.h"
#include "../../mqtt.h"
#include "../../util.h"
#include "../spaxstack/swquery.h"
#include "../spaxstack/protocol.h"
#include "radio.h"

RADIOLINK::RADIOLINK(const RADIOLINK& source){
        address = source.address; 
        channel = source.channel; 
        lqi = source.lqi; 
}

#define circular_incr(index) (index<0xFF)?(index++):(index=1)

void Registrar::send_nextheartbeat(){    
    circular_incr(nextHeartbeatAddr);
    send_heartbeat(nextHeartbeatAddr);
}

RADIOLINK* Registrar::getLink(uint8_t address){
    std::map<int, RADIOLINK>::iterator it = mapRadioLinks.find(address);
	if (it != mapRadioLinks.end()){
        int remote_addr = it->first;
        return &it->second;
    }
    return NULL;
}

void Registrar::incrErrCnt(uint8_t address){
    RADIOLINK* link = getLink(address);
    if(link != NULL){
        link->error_count++;
        if (link->error_count >= CNT_ERROR_DEREGISTER){
            verbprintf(2,"Link lost with address %i, deregistering \n", address);
            deregisterLink(address);
        }
    } 

};

void Registrar::incrSentCnt(uint8_t address){
    RADIOLINK* link = getLink(address);
    if(link != NULL) link->packets_sent++;
};

void Registrar::send_heartbeat(uint8_t address){
    std::map<int, RADIOLINK>::iterator it = mapRadioLinks.find(address);
	if (it != mapRadioLinks.end()){
        verbprintf(1,"Reg Heartbeat for %i \n", address);
        SWQUERY query = SWQUERY(address, address, REGI_DEVADDRESS);
        enque_tx_packet(&query, true);
    }
}

void Registrar::deregisterLink(uint8_t address){
        verbprintf(1,"Link %i lost. Deregistering\n", address);
        mapRadioLinks.erase(address);
        mqtt_send_radio_link_avail(address, false);
}

void Registrar::registerLink(uint8_t address, uint8_t lqi){
    if (address != BROADCAST_ADDRESS && address != MASTER_ADDRESS && address != 0xFF){
        RADIOLINK* link = getLink(address);
        if(link == NULL){
            mapRadioLinks[address] = RADIOLINK(address, 0, lqi);
            mqtt_send_radio_link_avail(address, true);
        }else{
            link->packets_sent++;
            link->lqi = lqi;
        }
    }
}