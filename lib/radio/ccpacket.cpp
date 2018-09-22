#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include "ccpacket.h"

void CCPACKET::copy(const CCPACKET &p2) { //copy constructor
      errorCode = p2.errorCode; 
      if (errorCode == 0){ //copy the net data only if the packet has no errors
        length = p2.length; 
        rssi = p2.rssi; 
        lqi = p2.lqi; 
        memcpy(data, &p2.data, length);
      }
    } ; 

char* CCPACKET::getHumanReadableError() { //copy constructor
        return ERR_DESCRIPTIONS[errorCode];
    } ;     