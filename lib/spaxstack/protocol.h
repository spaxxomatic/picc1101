
#ifndef _PROTOCOL_H
#define _PROTOCOL_H

#define MAX_RETRY_SEND_DATA 10
#define MAX_WAIT_RESPONSE 12 //multiples of 100ms
#define MAX_RETRY_SEND_ACK 10

#define ACK_PACKET 0XF0F0
#define CMD_SET_RADIO_ADDRESS 0xDD
#define CMD_SET_ACTOR 0x01
#define CMD_READ_INPUT 0x02

//comm stack errors
enum STACK_ERRORCODES 
{
    STACKERR_ACK_WITHOUT_SEND = 0, //we received a ack for a packet that we did not sent
    STACKERR_WRONG_DEST_ADDR //a packet with an address different from this radio address has reached the stack. This should not happend when addr check is enabled in the radio
};
#endif