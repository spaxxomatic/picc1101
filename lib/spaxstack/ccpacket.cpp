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

void CCPACKET::printAsHex(){
  printf("LEN: %i ERR: %i DATA:", length, errorCode);  
  for (int i = 0; i < length; i++){
      if (i > 0){
	        printf(":");
	    }
    printf("%02X", data[i]);
  }
  printf("\n");
} 
