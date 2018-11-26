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

void CCPACKET::dataAsHex(char* buff){
  char tmpfbuff[10];
  size_t pos = 0;
  for (int i = 0; i < length+2 ; i++){
      if (i > 0){
	   sprintf(tmpfbuff, ":");
	  }
      sprintf(tmpfbuff, "%02X", data[i]);
      strcpy(buff + pos, tmpfbuff);
	  pos += strlen(tmpfbuff);
  }
} 
