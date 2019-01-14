#include "ccpacket.h"
#include "memory.h"
#include <iostream> 
#include <string> 
#include <sstream>
#include <iomanip>
using namespace std; 

CCPACKET::CCPACKET(){
}

CCPACKET::CCPACKET(const CCPACKET* source){
  this->copy(source);  
}

void CCPACKET::copy(const CCPACKET* source) { //copy constructor
  if (source->errorCode == 0){ //copy the net data only if the packet has no errors
    length = source->length; 
    rssi = source->rssi; 
    lqi = source->lqi; 
    memcpy(data, source->data, length);
  }      
} 

std::string CCPACKET::to_string() const {
  std::stringstream ss;
  ss << "PKT: ( ERR " << (int) errorCode << ") LEN " << std::to_string(length) << ": ";
  for (int i = 0; i < length; i++){
      if (i > 0) ss << ":";
    ss << std::hex << (int) data[i];
  }
  return ss.str();
} 

void CCPACKET::incr_retry_cnt() {
  retry++;
}

void CCPACKET::printAsHex(){
  printf("LEN: %i ERR: %i DATA:", length, errorCode);  
  for (int i = 0; i < length; i++){
      if (i > 0) printf(":");
    printf("%02X", data[i]);
  }
  printf("\n");
} 
