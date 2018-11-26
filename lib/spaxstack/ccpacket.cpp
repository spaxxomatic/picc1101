#include "ccpacket.h"
#include "memory.h"
#include <iostream> 
#include <string> 
using namespace std; 

void CCPACKET::copy(CCPACKET* source) { //copy constructor
  if (source->errorCode == 0){ //copy the net data only if the packet has no errors
    length = source->length; 
    rssi = source->rssi; 
    lqi = source->lqi; 
    memcpy(data, source->data, length);
  }      
} 
