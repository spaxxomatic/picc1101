#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include "ccpacket.h"
#include "memory.h"

void CCPACKET::copy(CCPACKET* source) { //copy constructor
      errorCode = source->errorCode; 
      if (source == 0){ //copy the net data only if the packet has no errors
        length = source->length; 
        rssi = source->rssi; 
        lqi = source->lqi; 
        memcpy(data, source->data, length);
      }
    } ; 
