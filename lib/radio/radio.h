/******************************************************************************/
/* PiCC1101  - Radio serial link using CC1101 module and Raspberry-Pi         */
/*                                                                            */
/* Radio link definitions                                                     */
/*                                                                            */
/*                      (c) Edouard Griffiths, F4EXB, 2015                    */
/*                                                                            */
/******************************************************************************/

#ifndef _RADIO_H_
#define _RADIO_H_

#include "pi_cc_spi.h"
#include "pi_cc_cc1101.h"
#include "../spaxstack/protocol.h"
#include "../spaxstack/ccpacket.h"
#include "../spaxstack/swpacket.h"
#include "../spaxstack/spaxstack.h"

#define WPI_GDO0 5 // For Wiring Pi, 5 is GPIO_24 connected to GDO0
#define WPI_GDO2 6 // For Wiring Pi, 6 is GPIO_25 connected to GDO2

#define TX_FIFO_REFILL 60 // With the default FIFO thresholds selected this is the number of bytes to refill the Tx FIFO
#define RX_FIFO_UNLOAD 59 // With the default FIFO thresholds selected this is the number of bytes to unload from the Rx FIFO

#define RADIO_BUFSIZE (1<<16)   // 256 max radio block size times a maximum of 256 radio blocs

typedef enum sync_word_e
{
    NO_SYNC = 0,              // No preamble/sync
    SYNC_15_OVER_16,          // 15/16 sync word bits detected
    SYNC_16_OVER_16,          // 16/16 sync word bits detected
    SYNC_30_over_32,          // 30/32 sync word bits detected
    SYNC_CARRIER,             // No preamble/sync, carrier-sense above threshold
    SYNC_15_OVER_16_CARRIER,  // 15/16 + carrier-sense above threshold
    SYNC_16_OVER_16_CARRIER,  // 16/16 + carrier-sense above threshold
    SYNC_30_over_32_CARRIER   // 30/32 + carrier-sense above threshold
} sync_word_t;


typedef enum radio_modulation_e {
    RADIO_MOD_OOK,
    RADIO_MOD_FSK2,
    RADIO_MOD_FSK4,
    RADIO_MOD_MSK,
    RADIO_MOD_GFSK,
    NUM_RADIO_MOD
} radio_modulation_t;

typedef struct radio_parms_s
{
    uint32_t           f_xtal;        // Crystal frequency (Hz)
    uint32_t           f_if;          // IF frequency (Hz)
    radio_modulation_t modulation;    // Type of modulation
    uint8_t            fec;           // FEC is in use
    sync_word_t        sync_ctl;      // Sync word control
    float              deviat_factor; // FSK-2 deviation is +/- data rate divised by this factor
    uint32_t           freq_word;     // Frequency 24 bit word FREQ[23..0]
    uint8_t            chanspc_m;     // Channel spacing mantissa 
    uint8_t            chanspc_e;     // Channel spacing exponent
    uint8_t            if_word;       // Intermediate frequency 5 bit word FREQ_IF[4:0] 
    uint8_t            drate_m;       // Data rate mantissa
    uint8_t            drate_e;       // Data rate exponent
    uint8_t            chanbw_m;      // Channel bandwidth mantissa
    uint8_t            chanbw_e;      // Channel bandwidth exponent
    uint8_t            deviat_m;      // Deviation mantissa
    uint8_t            deviat_e;      // Deviation exponent
} radio_parms_t;

typedef enum radio_int_scheme_e 
{
    RADIOINT_NONE = 0,   // Do not use interrupts
    RADIOINT_SIMPLE,     // Interrupts for packets fitting in FIFO
    RADIOINT_COMPOSITE,  // Interrupts for any packet length up to 255
    NUM_RADIOINT
} radio_int_scheme_t;


typedef enum radio_errors_e {
    RADIO_PACKET_OK = 0,
    RADIOERR_PACKET_TOO_LONG ,
    RADIOERR_PACKET_CRC_ERR
} radio_errors_t;

typedef struct
{
	uint8_t code; 	
    char* description;  
} CODE_TO_DESCR;	

typedef enum radio_mode_e
{
    RADIOMODE_NONE = 0,
    RADIOMODE_RX,
    RADIOMODE_TX
} radio_mode_t;
 
#define BUFF_SIZE 64

typedef volatile struct radio_int_data_s 
{
    spi_parms_t  *spi_parms;             // SPI link parameters
    radio_mode_t mode;                   // Radio mode (essentially Rx or Tx)
    uint8_t      tx_count;               // Number of bytes in Tx buffer
    uint8_t      rx_count;               // Number of bytes in Rx buffer
    uint8_t      tx_buff_idx;        //Index of the last packet tramsitted in the tx buffer
    uint8_t      tx_buff_idx_ins;        //Index of the last packet inserted in the tx buffer
    uint8_t      rx_buff_idx;             // Index of the next packet to insert in buffer
    uint8_t      rx_buff_read_idx;        // Read side - Index of the next packet to read from the buffer
    uint8_t      packet_send;            // Indicates transmission of a packet is in progress
    uint32_t     wait_us;                // Unit wait time of approximately 4 2-FSK symbols
    uint8_t      last_error;
} radio_int_data_t;


static radio_int_data_t *p_radio_int_data = 0;
static radio_int_data_t radio_int_data;

volatile static int packets_sent = 0;
volatile static int packets_received = 0;

extern const char     *state_names[];
extern float    chanbw_limits[];

void     init_radio_parms(radio_parms_t *radio_parms, arguments_t *arguments);
int      init_radio(radio_parms_t *radio_parms,  spi_parms_t *spi_parms, arguments_t *arguments);
void     init_radio_int(spi_parms_t *spi_parms, arguments_t *arguments);
void     radio_init_rx(spi_parms_t *spi_parms);
void     radio_flush_fifos(spi_parms_t *spi_parms);

void     radio_turn_idle(spi_parms_t *spi_parms);
void     radio_turn_rx(spi_parms_t *spi_parms);

void     print_radio_parms(radio_parms_t *radio_parms);
int      print_radio_status(spi_parms_t *spi_parms);

int      radio_set_packet_length(spi_parms_t *spi_parms, uint8_t pkt_len);
uint8_t  radio_get_packet_length(spi_parms_t *spi_parms);
float    radio_get_rate(radio_parms_t *radio_parms);
float    radio_get_byte_time(radio_parms_t *radio_parms);
void     radio_wait_a_bit(uint32_t amount);
void     radio_wait_free();

static bool     tx_handler(spi_parms_t *spi_parms);
static uint8_t  radio_process_receive(uint8_t *block, uint32_t *size, uint8_t *crc);

//void     radio_send_packet(spi_parms_t *spi_parms, arguments_t *arguments, uint8_t *packet, uint32_t size);
//uint32_t radio_receive_packet(spi_parms_t *spi_parms, arguments_t *arguments, uint8_t *packet);

#endif
