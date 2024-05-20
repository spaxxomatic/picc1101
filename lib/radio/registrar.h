#ifndef _REGISTRAR_H_
#define _REGISTRAR_H_

#define CNT_ERROR_DEREGISTER 2

class RADIOLINK {
    public:
    RADIOLINK():address(0),channel(0),lqi(0){};
    RADIOLINK(uint8_t address, uint8_t channel, uint8_t lqi):address(address),channel(channel),lqi(lqi){};
    RADIOLINK(const RADIOLINK& source);
    uint8_t address;
    uint8_t channel;    
    uint8_t lqi;
    int error_count=0;
    int packets_sent=0;
};

class Registrar {
    public:
    void send_heartbeat(uint8_t address);
    void send_nextheartbeat();
    void registerLink(uint8_t address, uint8_t lqi);
    RADIOLINK* getLink(uint8_t address);
    void deregisterLink(uint8_t address);
    void incrErrCnt(uint8_t address);
    void incrSentCnt(uint8_t address);
    private:
    std::map<int, RADIOLINK> mapRadioLinks; // List of registered radio links
    int nextHeartbeatAddr=0;
};

#endif