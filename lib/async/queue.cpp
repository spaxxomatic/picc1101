#include "queue.h"
#include "../radio/radio.h"


#define PKTKEY(a,b) a<<8|b

  AckAwaitQueue::AckAwaitQueue() : taskDelay( 500 ),
      startTime( std::chrono::steady_clock::now() ) // only used for debugging
  {
  }

  void AckAwaitQueue::trigger(){
    // increase the time to process the queue to "now + x seconds"
    timeout = std::chrono::steady_clock::now() + taskDelay;
    if ( !running )
    {
      // launch a new asynchronous task which will process the queue
      task = std::async( std::launch::async, [this]{ processWork(); } );
      running = true;
    }
  }

  void AckAwaitQueue::append( short destAddr, short packetNo, const CCPACKET& packet )
  {
    int key = PKTKEY( destAddr, packetNo);
    std::unique_lock< std::mutex > lock( mutex );
    std::cout << "Q packet no "<< packetNo << " for " << destAddr << " key " << key << "\n";
    mapPacketAwaitAck[key] = CCPACKET(packet);
    //std::cout << " -----xxxx- " << CCPACKET(packet).to_string() << "\n";
    trigger();
  }

  void AckAwaitQueue::ack_packet( short destAddr, short packetNo )
  {
    int key = PKTKEY( destAddr, packetNo);
    std::cout << "R Ack packet no "<< packetNo << " for " << destAddr << " key " << key << "\n";
    std::map<int, CCPACKET>::iterator it = mapPacketAwaitAck.find(key);
	  if (it != mapPacketAwaitAck.end()){
          CCPACKET& packet = it->second;
          packet.ack_ok = true;
          //TODO: as of now, we hold a single packet for each destination. 
          //If we implement a multi-packet queue, we'll have to find the right packet with the given packetNo 
          //and aknowledge only this one
          std::cout << "Ack packet no "<< packetNo << " for " << destAddr << "\n";
    }
  }

  AckAwaitQueue::~AckAwaitQueue()
  {
    std::unique_lock< std::mutex > lock( mutex );
    // stop processing the queue
    closing = true;
    bool wasRunning = running;
    condition.notify_all();
    lock.unlock();
    if ( wasRunning )
    {
      // wait for the async task to complete
      task.wait();
    }
  }

  void AckAwaitQueue::processWork()
  {
    std::unique_lock< std::mutex > lock( mutex );
    // loop until std::chrono::steady_clock::now() > timeout
    auto wait = timeout - std::chrono::steady_clock::now();
    while ( !closing && wait > std::chrono::seconds( 0 ) )
    {
      condition.wait_for( lock, wait );
      wait = timeout - std::chrono::steady_clock::now();
    }
    if ( !closing )
    {
      std::cout << "processing queue at " << std::chrono::duration_cast< std::chrono::milliseconds >( std::chrono::steady_clock::now() - startTime ).count() << "ms\n";
      //while ( !work.empty() )
      //for (auto const& x : mapPacketAwaitAck)   
      for (auto it = mapPacketAwaitAck.begin(), next_it = it; it != mapPacketAwaitAck.end(); it = next_it) /* no increment !!*/   
      {
        //int key = work.front();
        ++next_it;
        int key = it->first;
	      //std::map<int, CCPACKET>::iterator it = mapPacketAwaitAck.find(key);
	      //if (it == mapPacketAwaitAck.end()){
        //  std::cerr << "Ugly bug. Queued packet is lost";
        //}else{
          CCPACKET* packet = &it->second;
          if (packet->ack_ok){
            std::cout << "ACK OK for key " << key << ". Dequeueing\n";
            mapPacketAwaitAck.erase(it);
            //work.pop();
          }else{ //resend packet and increment the counter
            if (packet->retry >= MAX_RESEND_RETRY){
              std::cout << "Max resend reached for key " << key << ". Drop packet\n";
              //mapPacketAwaitAck.erase(lastkey);   
              mapPacketAwaitAck.erase(it);   
            }else{
              packet->incr_retry_cnt();
              std::cout << "Resend no " << packet->retry <<" for key " << key <<"\n";
              //std::cout << " ------ " << packet->to_string() << "\n";
              resend_packet(packet);
            }
          //}
          //work.pop();        
          //std::cout << packet.to_string() << "\n";
        } 
      }
      std::cout << std::flush;
    }
    else
    {
      std::cout << "aborting queue processing at " << std::chrono::duration_cast< std::chrono::milliseconds >( std::chrono::steady_clock::now() - startTime ).count() << "ms with " 
      << mapPacketAwaitAck.size() << " remaining items\n";
    }
    running = false;
  };
