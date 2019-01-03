
#ifndef __QUEUE_H__
#define __QUEUE_H__

#include <iostream>
#include <string>
#include <future>
#include <mutex>
#include <chrono>
#include <queue>
#include <map>
#include <iterator>
#include "../spaxstack/ccpacket.h"

#define MAX_RESEND_RETRY 4

class AckAwaitQueue
{
public:
  AckAwaitQueue();
  void ack_packet( short destId, short packetNo );
  void append( short destAddr, short packetNo, const CCPACKET& packet );
  ~AckAwaitQueue();
  void trigger();
  
private:
  std::map<int, CCPACKET> mapPacketAwaitAck; // Buffer of messages awaiting aknowledgement
    
  std::mutex mutex;
  std::condition_variable condition;
  std::chrono::milliseconds taskDelay;
  std::chrono::steady_clock::time_point timeout;
  std::queue< int > work;
  std::future< void > task;
  bool closing = false;
  bool running = false;
  std::chrono::steady_clock::time_point startTime;
  
  void processWork();
};

#endif