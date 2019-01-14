
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
/*
enum STACK_ERRORCODES 
{
    STACKERR_OK = 0, 
    STACKERR_REGISTER_NOT_FOUND, //Not a valid register
    STACKERR_UNKNOWN_FUNCTION, //not a valid function
    STACKERR_ACK_WITHOUT_SEND, //we received a ack for a packet that we did not sent
    STACKERR_WRONG_DEST_ADDR, //a packet with an address different from this radio address has reached the stack. This should not happend when addr check is enabled in the radio
    STACKERR_INVALID_PARM_LENGTH // wrong param length for this register
};
*/
#define FOREACH_ELEM(ERRCODE) \
        ERRCODE(STACKERR_OK)   \
        ERRCODE(STACKERR_REGISTER_NOT_FOUND)  \
        ERRCODE(STACKERR_UNKNOWN_FUNCTION)   \
        ERRCODE(STACKERR_ACK_WITHOUT_SEND)  \
        ERRCODE(STACKERR_WRONG_DEST_ADDR)  \
        ERRCODE(STACKERR_INVALID_PARM_LENGTH)  \
        ERRCODE(ERR_UNKNOWN_COMMAND) \
        ERRCODE(ERR_REGISTER_HAS_NO_SETTER) \

#define GENERATE_ENUM(ENUM) ENUM,
#define GENERATE_STRING(STRING) #STRING,

enum STACK_ERRORCODES {
    FOREACH_ELEM(GENERATE_ENUM)
    LASTELEM = ERR_REGISTER_HAS_NO_SETTER
};

static const char *STACK_ERRORCODES_STRINGS[] = {
    FOREACH_ELEM(GENERATE_STRING)
};

static const char* getStackErrorText(uint8_t errorCode){
    if (errorCode <= LASTELEM)
        return STACK_ERRORCODES_STRINGS[errorCode];
    else  return "STACKERR_UNKNOWN" ;
}

static const char* getAlarmText(uint8_t alarmCode){
    //TODO: this should be read from a device definition file
    return "TIMEOUT_REACHING_DOOR_STATE" ;
}

int registerNewNode();

enum CUSTOM_REGINDEX                    
{                                       
  REGI_PRODUCTCODE = 0,                 
  REGI_HWVERSION,                     
  REGI_FWVERSION,                  
  REGI_FREQCHANNEL,                     
  REGI_NETWORKID,                       
  REGI_DEVADDRESS,                      
  REGI_TXINTERVAL,
};

#endif