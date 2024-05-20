#include "queue.h"
#include "../radio/radio.h"
#include "../../util.h"

#define PKTKEY(a,b) a<<8|b
#define ADDR_OF_KEY(a) a>>8
#define PKTNO_OF_KEY(a) a&0x00FF

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
              verbprintf(4, "Max resend reached for key %i. Dropping. \n" , key );
              //mapPacketAwaitAck.erase(lastkey);   
              mapPacketAwaitAck.erase(it);
              registrar.incrErrCnt(ADDR_OF_KEY(key));
            }else{
              packet->incr_retry_cnt();
              verbprintf(4, "%i resend for addr %i pktno %i \n" , packet->retry , ADDR_OF_KEY(key), PKTNO_OF_KEY(key));
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
