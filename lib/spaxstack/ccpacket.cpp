#include "ccpacket.h"
#include "memory.h"
#include <iostream> 
#include <string> 
using namespace std; 

void CCPACKET::copy(CCPACKET* source) { //copy constructor
  errorCode = source->errorCode; 
  if (source == 0){ //copy the net data only if the packet has no errors
    length = source->length; 
    rssi = source->rssi; 
    lqi = source->lqi; 
    memcpy(data, source->data, length);
  }      
} 

void CCPACKET::dataAsHex(string buff){
  char tmpfbuff[10];
  for (int i = 0; i < length+2 ; i++){
      if (i > 0) sprintf(tmpfbuff, ":");
      sprintf(tmpfbuff, "%02X", data[i]);
      buff.append(tmpfbuff);
  }
} 
