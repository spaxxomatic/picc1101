#include "queue.h"
#include "../radio/radio.h"
#include "../../util.h"

#define PKTKEY(a,b) a<<8|b

  AckAwaitQueue::AckAwaitQueue() : taskDelay( 500 ),
      startTime( std::chrono::steady_clock::now() ) // only used for debugging
  {
  }

  void AckAwaitQueue::trigger(){
    // increase the time to process the queue to "now + x seconds"
    //std::cout << "trigger "<< running << "\n";
    if ( !running )
    {
      std::unique_lock<std::mutex> lock( mutex );
      timeout = std::chrono::steady_clock::now() + taskDelay;
      // launch a new asynchronous task which will process the queue
      task = std::async( std::launch::async, [this]{ processWork(); } );
      running = true;
    }
  }

  void AckAwaitQueue::append( short destAddr, short packetNo, const CCPACKET& packet )
  {
    int key = PKTKEY( destAddr, packetNo);
    //std::cout << "Q packet no "<< packetNo << " for " << destAddr << " key " << key << "\n";
    mapPacketAwaitAck[key] = CCPACKET(packet);
    //std::cout << " -----xxxx- " << CCPACKET(packet).to_string() << "\n";
    trigger();
  }

  void AckAwaitQueue::ack_packet( short destAddr, short packetNo )
  {
    int key = PKTKEY( destAddr, packetNo);
    //std::cout << "R Ack packet no "<< packetNo << " for " << destAddr << " key " << key << "\n";
    std::map<int, CCPACKET>::iterator it = mapPacketAwaitAck.find(key);
	  if (it != mapPacketAwaitAck.end()){
          CCPACKET& packet = it->second;
          packet.ack_ok = true;
          //TODO: as of now, we hold a single packet for each destination. 
          //If we implement a multi-packet queue, we'll have to find the right packet with the given packetNo 
          //and aknowledge only this one
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
      //std::cout << "processing queue at " << std::chrono::duration_cast< std::chrono::milliseconds >( std::chrono::steady_clock::now() - startTime ).count() << "ms\n";
      for (auto it = mapPacketAwaitAck.begin(), next_it = it; it != mapPacketAwaitAck.end(); it = next_it) /* no increment !!*/   
      {
        ++next_it;
        int key = it->first;
          CCPACKET* packet = &it->second;
          if (packet->ack_ok){
            //std::cout << "ACK OK for key " << key << ". Dequeueing\n";
            mapPacketAwaitAck.erase(it);
          }else{ //resend packet and increment the counter
            if (packet->retry >= MAX_RESEND_RETRY){
              verbprintf(5, "Max resend reached for key %i. Dropping \n" , key );
              //mapPacketAwaitAck.erase(lastkey);   
              mapPacketAwaitAck.erase(it);   
            }else{
              packet->incr_retry_cnt();
              verbprintf(5, "%i resend for key %i \n" , packet->retry , key );
              resend_packet(packet);
            }
        } 
      }
    }
    else
    {
      fprintf(stderr, "Aborting queue processing. %i remaining items\n", mapPacketAwaitAck.size() );
    }
    running = false;
  };
