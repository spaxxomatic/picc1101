/******************************************************************************/
/* PiCC1101  - Radio serial link using CC1101 module and Raspberry-Pi         */
/*                                                                            */
/* Radio link interface                                                       */
/*                                                                            */
/*                      (c) Lucian Nutiu, Edouard Griffiths, F4EXB, 2015                    */
/*                                                                            */
/******************************************************************************/

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <pthread.h>
#include <map>
#include <iterator>
#include <sstream>
//when compiling in mocked mode, include the mock and disable the global include
#ifdef _MOCKED
#include "../../mocks/wiringPi.h"
#else 
#include <wiringPi.h>
#endif

#include "params.h"
#include "../../mqtt.h"
#include "../../util.h"
#include "radio.h"
#include "pi_cc_spi.h"
#include "cc1101_defvals.h"
#include "pi_cc_cc1101.h"

#include "../inih/inireader.h"
extern INIReader* inireader ;
AckAwaitQueue ackAwaitQueue;
Registrar registrar;

sem_t sem_radio_irq; //semaphore for thread sync
//static radio_int_data_t *p_radio_int_data = 0;
extern int registerNewNode();

#define RADIO_WATCHDOG_TIMEOUT 1000 // ms to wait for a send/receive finish
radio_parms_t radio_parameters;

const char *state_names[] = {
	"SLEEP",            // 00
	"IDLE",             // 01
	"XOFF",             // 02
	"VCOON_MC",         // 03
	"REGON_MC",         // 04
	"MANCAL",           // 05
	"VCOON",            // 06
	"REGON",            // 07
	"STARTCAL",         // 08
	"BWBOOST",          // 09
	"FS_LOCK",          // 10 0xA
	"IFADCON",          // 11 0xB
	"ENDCAL",           // 12 0xC
	"RX",               // 13 0xD
	"RX_END",           // 14 0xE
	"RX_RST",           // 15 0xF
	"TXRX_SWITCH",      // 16 0x10
	"RXFIFO_OVERFLOW",  // 17 0x11
	"FSTXON",           // 18 0x12
	"TX",               // 19 0x13
	"TX_END",           // 20 0x14
	"RXTX_SWITCH",      // 21 0x15
	"TXFIFO_UNDERFLOW", // 22 0x16
	"undefined",        // 23 0x17
	"undefined",        // 24 0x18
	"undefined",        // 25 0x19
	"undefined",        // 26 0x20
	"undefined",        // 27 0x21
	"undefined",        // 28 0x22
	"undefined",        // 29 0x23
	"undefined",        // 30 0x24
	"undefined"         // 31 0x25
};


CODE_TO_DESCR ERR_DESCRIPTIONS[] =
{
	{RADIO_PACKET_OK, "Packet OK"},
	{RADIOERR_PACKET_TOO_LONG, "Packet too long"},
	{RADIOERR_PACKET_CRC_ERR, "CRC ERR"}
};

#define circular_incr(index) (index<BUFF_SIZE-1)?(index++):(index=0)

CCPACKET  tx_ccpacket_buf[BUFF_SIZE]; // Tx buffer
CCPACKET  rx_ccpacket_buf[BUFF_SIZE]; // Rx buffer

// === Static functions declarations ==============================================================

static float    rssi_dbm(uint8_t rssi_dec);
static bool     wait_for_state(ccxxx0_state_t state, uint32_t timeout);
static uint8_t  crc_check(uint8_t *block);

// === Interupt handlers ==========================================================================

void irq_handle_packet(void) {
	// the irq handler will read data from the radio chip and transfer it to the p_radio_int_data->rx_buff beginning from the pos 1
	// The pos 0 (first byte) in the buffer will contain the error code. 
	// Errors will be stored inside the packet. If the packet is too long no data will be copied to the buff

	uint8_t rxBytes;
	uint8_t rxDataLength;
	uint8_t rxAddr;
	static CCPACKET packet;
	packet.errorCode = 0;
	uint8_t int_line = digitalRead(WPI_GDO0); // Sense interrupt line to determine if it was a raising or falling edge
	if (radio_int_data.mode == RADIOMODE_RX) {
		if (int_line)
		{ 
			fprintf(stderr," R->"); 
			radio_int_data.packet_receive = 1;
		}else{
			fprintf(stderr,"<-R\n"); fflush(stderr);
			//verbprintf(3, "RX F IRQ\n");
			PI_CC_SPIReadStatus(PI_CCxxx0_RXBYTES, &rxBytes);
			
			// Any byte waiting to be read and no overflow?			
			if (rxBytes & 0x80){ //overflow
				verbprintf(3, "RX fifo ovrflw: \n"); //rx fifo is polluted by some spurious reception
			} else if (rxBytes & 0x7F) { //data in buffer
				// Read data length
				PI_CC_SPIReadReg(PI_CCxxx0_RXFIFO, &rxDataLength); //first byte contains the data len
				if (rxDataLength != rxBytes-3){
					verbprintf(3, "   ???datalen %i vs %i \n", rxBytes, rxDataLength); //rx fifo is now polluted, data not usable, empty it
				}
				if (rxDataLength > CC1101_DATA_LEN) {
					verbprintf(3, "Pkt too long: %i \n", rxDataLength); //rx fifo is now polluted, data not usable, empty it
				}
				else {
					//read the net data
					packet.length = rxDataLength ;
					PI_CC_SPIReadBurstReg(PI_CCxxx0_RXFIFO, packet.data, packet.length ); 
					
					//packet.printAsHex();
					//the last 2 bytes are appended by the CC1101. First one is the RSSI and the second is LQI/CRC
					// Read RSSI
					PI_CC_SPIReadReg(PI_CCxxx0_RXFIFO, &packet.rssi);
					// Read LQI and CRC_OK
					PI_CC_SPIReadReg(PI_CCxxx0_RXFIFO, &packet.lqi);
					bool crc_ok = bitRead(packet.lqi, 7);
					packet.lqi = packet.lqi & 0x7F;
					if (!crc_ok) {
						verbprintf(1, "CRC err \n");
						packet.errorCode = RADIOERR_PACKET_CRC_ERR;
					}else{
						//increment the buff counter
						circular_incr(radio_int_data.rx_buff_idx);
						rx_ccpacket_buf[radio_int_data.rx_buff_idx].errorCode = packet.errorCode;
						//the copy function will copy the data only if there are no errors
						//cout << "RCIRQ" << packet.to_string();
						rx_ccpacket_buf[radio_int_data.rx_buff_idx].copy(&packet);
						//increment semaphore to notify packet reception
						sem_post(&sem_radio_irq);
					}
				}
			} else {
				verbprintf(3, "RX fifo unhandled condition %.2X:\n", rxBytes); 
				radio_int_data.mode = RADIOMODE_UNDEF_CONDITION; //this will trigger a radio reset
				radio_int_data.packet_receive = 0; //receive done
				sem_post(&sem_radio_irq);
				return;
			}
			PI_CC_SPIStrobe(PI_CCxxx0_SIDLE);// Enter IDLE state
			PI_CC_SPIStrobe(PI_CCxxx0_SFRX); // Flush Rx FIFO
			PI_CC_SPIStrobe(PI_CCxxx0_SRX); //turn on Rx again			
			radio_int_data.packet_receive = 0; //receive done
		}
	}
	else if (radio_int_data.mode == RADIOMODE_TX) {
		if (int_line) {
			fprintf(stderr," T->"); 
			radio_int_data.packet_send = 1; // Assert packet transmission after sync has been sent
			//verbprintf(3, "TX R IRQ\n");
		}else {
			fprintf(stderr,"<-T\n"); fflush(stderr);
			//verbprintf(3, "TX F IRQ\n");
			if (radio_int_data.packet_send) // packet has been sent
			{
				radio_int_data.mode = RADIOMODE_TX_END; //the mode has to be set 
				radio_int_data.packet_send = 0; // De-assert packet transmission after packet has been sent
			}
		}
	}
}
// === Static functions ===========================================================================
// ------------------------------------------------------------------------------------------------
// Calculate RSSI in dBm from decimal RSSI read out of RSSI status register
float rssi_dbm(uint8_t rssi_dec)
// ------------------------------------------------------------------------------------------------
{
	if (rssi_dec >= 128)
	{
		return ((rssi_dec - 256) / 2.0) - 74.0;
	}
	else
	{
		return (rssi_dec / 2.0) - 74.0;
	}
}

// ------------------------------------------------------------------------------------------------
// Poll FSM state waiting for a post-TX state transition until timeout (approx ms)
bool wait_for_tx_end(uint32_t timeout)
// ------------------------------------------------------------------------------------------------
{
	uint8_t fsm_state;
	while (timeout)
	{
		usleep(500);
		PI_CC_SPIReadStatus(PI_CCxxx0_MARCSTATE, &fsm_state);
		fsm_state &= 0x1F; //state change timer
		if ((fsm_state != 0x13) && (fsm_state != 0x14) && (fsm_state != 0x15)) {
			break;
		}
		timeout--;
	}

	if (!timeout)
	{
		verbprintf(1, "wait_for_tx_end: timeout reached in state %s waiting for TX end\n", state_names[fsm_state]);
		return false;
	}
	return true;
}

// ------------------------------------------------------------------------------------------------
// Poll FSM state waiting for given state until timeout (approx ms)
bool wait_for_state(ccxxx0_state_t state, uint32_t timeout)
// ------------------------------------------------------------------------------------------------
{
	uint8_t fsm_state;

	while (timeout)
	{
		PI_CC_SPIReadStatus(PI_CCxxx0_MARCSTATE, &fsm_state);
		fsm_state &= 0x1F;
#ifdef _MOCKED //when mocked
		break;
#endif
		if (fsm_state == (uint8_t)state)
		{
			break;
		}

		usleep(1000);
		timeout--;
	}

	if (!timeout)
	{
		verbprintf(1, "wait_for_state: timeout reached in state %s waiting for state %s\n", state_names[fsm_state], state_names[state]);
		return false;
		if (fsm_state == CCxxx0_STATE_RXFIFO_OVERFLOW)
		{
			PI_CC_SPIStrobe(PI_CCxxx0_SFRX); // Flush Rx FIFO
			PI_CC_SPIStrobe(PI_CCxxx0_SFTX); // Flush Tx FIFO
		}
	}
	return true;
}

// === Public functions ===========================================================================

// ------------------------------------------------------------------------------------------------
// Initialize interrupt data and mechanism
void init_radio_int()
// ------------------------------------------------------------------------------------------------
{
	radio_int_data.rx_buff_idx = 0;
	radio_int_data.rx_buff_read_idx = 0;
	radio_int_data.tx_buff_idx_sent = 0;
	radio_int_data.tx_buff_idx_ins = 0;

	packets_sent = 0;
	packets_received = 0;
	if (wiringPiISR(WPI_GDO0, INT_EDGE_BOTH, &irq_handle_packet) != 0) {
		fprintf(stderr, "IRQ ISR failed\n");
	};       // set interrupt handler for packet interrupts
}

// ------------------------------------------------------------------------------------------------
// Inhibit operations by returning to IDLE state
void radio_turn_idle()
// ------------------------------------------------------------------------------------------------
{
	PI_CC_SPIStrobe(PI_CCxxx0_SIDLE);
}

// ------------------------------------------------------------------------------------------------
// Allow Rx operations by returning to Rx state
void radio_turn_rx()
// ------------------------------------------------------------------------------------------------
{
	PI_CC_SPIStrobe(PI_CCxxx0_SRX);
	wait_for_state(CCxxx0_STATE_RX, 10); // Wait max 10ms
}

// ------------------------------------------------------------------------------------------------
// Flush Rx and Tx FIFOs
void radio_flush_fifos()
// ------------------------------------------------------------------------------------------------
{
	PI_CC_SPIStrobe(PI_CCxxx0_SFRX); // Flush Rx FIFO
	PI_CC_SPIStrobe(PI_CCxxx0_SFTX); // Flush Tx FIFO
}

int setup_spi(arguments_t *arguments, const char* spi_device){
	
	wiringPiSetup(); // initialize Wiring Pi library and GDOx interrupt routines
	verbprintf(1, "wiringPiSetup done.\n");
	int ret = 0;
	// Switch main thread to real time
	if (arguments->real_time)
	{
		ret = piHiPri(1);
		if (ret == 0){
			fprintf(stderr, "RADIO: real time OK\n");
		}else{
			perror("RADIO: cannot set in real time");
		}
	}

	// open SPI link
	PI_CC_SPIParmsDefaults();
	ret = PI_CC_SPISetup(spi_device);
	if (ret != 0)
	{
		fprintf(stderr, "RADIO: cannot open SPI link, RC=%d\n", ret);
	}
	else
	{
		verbprintf(1, "SPI open.\n");
	}
	return ret;
}

int reset_radio(const char* reason){
	verbprintf(0, "\nRESET RADIO: (reason: %s)\n", reason);
	radio_parameters.f_xtal = 26000000;         // 26 MHz Xtal
	radio_parameters.f_if = 310000;           // 304.6875 kHz (lowest point below 310 kHz)
	radio_parameters.channel = inireader->GetInteger("RADIO","channel", 0);
	int ret = init_radio();
	if (ret == 0){
		radio_flush_fifos();
	  	radio_init_rx(); // init for new packet to receive Rx
	  	radio_turn_rx(); // Turn Rx on
	}
	return ret;
}

// ------------------------------------------------------------------------------------------------
// Initialize the radio link interface
int init_radio()
// ------------------------------------------------------------------------------------------------
{
	int ret = 0;
	uint8_t  reg_word;
	ret = PI_CC_PowerupResetCCxxxx(); // reset chip
	if (ret != 0)
	{
		fprintf(stderr, "RADIO: cannot reset CC1101 chip, RC=%d\n", ret);
		return ret;
	}
	else
	{
		verbprintf(1, "CC1101 chip has been reset.\n");
	}

	// Write register settings

	// IOCFG2 = 0x00: Set in Rx mode (0x02 for Tx mode)
	// o 0x00: Asserts when RX FIFO is filled at or above the RX FIFO threshold. 
	//         De-asserts when RX FIFO is drained below the same threshold.
	// o 0x02: Asserts when the TX FIFO is filled at or above the TX FIFO threshold.
	//         De-asserts when the TX FIFO is below the same threshold.
	PI_CC_SPIWriteReg(PI_CCxxx0_IOCFG2, PI_CCxxx0_IOCFG2); // GDO2 output pin config.

	// IOCFG0 = 0x06: Asserts when sync word has been sent / received, and de-asserts at the
	// end of the packet. In RX, the pin will de-assert when the optional address
	// check fails or the RX FIFO overflows. In TX the pin will de-assert if the TX
	// FIFO underflows:    
	//PI_CC_SPIWriteReg( PI_CCxxx0_IOCFG0,   0x06); // GDO0 output pin config.
	PI_CC_SPIWriteReg(PI_CCxxx0_IOCFG0, CC1101_DEFVAL_IOCFG0);
	// FIFO_THR = 14: 
	// o 5 bytes in TX FIFO (55 available spaces)
	// o 60 bytes in the RX FIFO
	PI_CC_SPIWriteReg(PI_CCxxx0_FIFOTHR, CC1101_DEFVAL_FIFOTHR); // FIFO threshold.

	// PKTLEN: packet length up to 255 bytes. 
	PI_CC_SPIWriteReg(PI_CCxxx0_PKTLEN, CC1101_DEFVAL_PKTLEN); // Packet length.

	// PKTCTRL0: Packet automation control #0
	// . bit  7:   unused
	// . bit  6:   0  -> whitening off
	// . bits 5:4: 00 -> normal mode use FIFOs for Rx and Tx
	// . bit  3:   unused
	// . bit  2:   1  -> CRC enabled
	// . bits 1:0: xx -> Packet length mode. Taken from radio config.
	//reg_word = (arguments->whitening<<6) + 0x04 + (int) radio_parms->packet_config;

	//reg_word = (arguments->whitening<<6) + 0x04 + (int) radio_parms->packet_config;
	PI_CC_SPIWriteReg(PI_CCxxx0_PKTCTRL0, CC1101_DEFVAL_PKTCTRL0); // Packet automation control.

	// PKTCTRL1: Packet automation control #1
	// . bits 7:5: 000 -> Preamble quality estimator threshold
	// . bit  4:   unused
	// . bit  3:   0   -> Automatic flush of Rx FIFO disabled (too many side constraints see doc)
	// . bit  2:   1   -> Append two status bytes to the payload (RSSI and LQI + CRC OK)
	// . bits 1:0: 00  -> No address check of received packets
	PI_CC_SPIWriteReg(PI_CCxxx0_PKTCTRL1, CC1101_DEFVAL_PKTCTRL1); // Packet automation control.

	//#define CC1101_DEFVAL_SYNC1      0xB5        // Synchronization word, high byte
	//#define CC1101_DEFVAL_SYNC0      0x47        // Synchronization word, low byte
	PI_CC_SPIWriteReg(PI_CCxxx0_SYNC1, CC1101_DEFVAL_SYNC1);
	PI_CC_SPIWriteReg(PI_CCxxx0_SYNC0, CC1101_DEFVAL_SYNC0);

	PI_CC_SPIWriteReg(PI_CCxxx0_ADDR, MASTER_ADDRESS);

	//PI_CC_SPIWriteReg(PI_CCxxx0_CHANNR, CC1101_DEFVAL_CHANNR); // Channel number
	PI_CC_SPIWriteReg(PI_CCxxx0_CHANNR, radio_parameters.channel);


	// FSCTRL0: Frequency offset added to the base frequency before being used by the
	// frequency synthesizer. (2s-complement). Multiplied by Fxtal/2^14
	PI_CC_SPIWriteReg(PI_CCxxx0_FSCTRL0, CC1101_DEFVAL_FSCTRL0); // Freq synthesizer control.
	PI_CC_SPIWriteReg(PI_CCxxx0_FSCTRL1, CC1101_DEFVAL_FSCTRL1);
	// FSCTRL1: The desired IF frequency to employ in RX. Subtracted from FS base frequency
	// in RX and controls the digital complex mixer in the demodulator. Multiplied by Fxtal/2^10
	// Here 0.3046875 MHz (lowest point below 310 kHz)
	
	//radio_parms->if_word = get_if_word(radio_parms->f_xtal, radio_parms->f_if);
	//PI_CC_SPIWriteReg( PI_CCxxx0_FSCTRL1, (radio_parms->if_word & 0x1F)); // Freq synthesizer control.

	// FREQ2..0: Base frequency for the frequency sythesizer
	// Fo = (Fxosc / 2^16) * FREQ[23..0]
	// FREQ2 is FREQ[23..16]
	// FREQ1 is FREQ[15..8]
	// FREQ0 is FREQ[7..0]
	// Fxtal = 26 MHz and FREQ = 0x10A762 => Fo = 432.99981689453125 MHz
	
	PI_CC_SPIWriteReg(PI_CCxxx0_FREQ2, CC1101_DEFVAL_FREQ2_433); // Freq control word, high byte
	PI_CC_SPIWriteReg(PI_CCxxx0_FREQ1, CC1101_DEFVAL_FREQ1_433); // Freq control word, mid byte.
	PI_CC_SPIWriteReg(PI_CCxxx0_FREQ0, CC1101_DEFVAL_FREQ0_433);  // Freq control word, low byte.

	// MODCFG4 Modem configuration - bandwidth and data rate exponent
	// High nibble: Sets the decimation ratio for the delta-sigma ADC input stream hence the channel bandwidth
	// . bits 7:6: 0  -> CHANBW_E: exponent parameter (see next)
	// . bits 5:4: 2  -> CHANBW_M: mantissa parameter as per:
	//      BW = Fxosc / 8(4+CHANBW_M).2^CHANBW_E => Here: BW = 26/48 MHz = 541.67 kHz
	//      Factory defaults: M=0, E=1 => BW = 26/128 ~ 203 kHz
	// Low nibble:
	// . bits 3:0: 13 -> DRATE_E: data rate base 2 exponent => here 13 (multiply by 8192)
	//reg_word = (radio_parms->chanbw_e << 6) + (radio_parms->chanbw_m << 4) + radio_parms->drate_e;
	PI_CC_SPIWriteReg(PI_CCxxx0_MDMCFG4, CC1101_DEFVAL_MDMCFG4); // Modem configuration.

	// MODCFG3 Modem configuration: DRATE_M data rate mantissa as per formula:
	//    Rate = (256 + DRATE_M).2^DRATE_E.Fxosc / 2^28 
	// Here DRATE_M = 59, DRATE_E = 13 => Rate = 250 kBaud
	PI_CC_SPIWriteReg(PI_CCxxx0_MDMCFG3, CC1101_DEFVAL_MDMCFG3); // Modem configuration.

	// MODCFG2 Modem configuration: DC block, modulation, Manchester, sync word
	// o bit 7:    0   -> Enable DC blocking (1: disable)
	// o bits 6:4: xxx -> (provided)
	// o bit 3:    0   -> Manchester disabled (1: enable)
	// o bits 2:0: 011 -> Sync word qualifier is 30/32 (static init in radio interface)
	PI_CC_SPIWriteReg(PI_CCxxx0_MDMCFG2, CC1101_DEFVAL_MDMCFG2); // Modem configuration.

	// MODCFG1 Modem configuration: FEC, Preamble, exponent for channel spacing
	// o bit 7:    0   -> FEC disabled (1: enable)
	// o bits 6:4: 2   -> number of preamble bytes (0:2, 1:3, 2:4, 3:6, 4:8, 5:12, 6:16, 7:24)
	// o bits 3:2: unused
	// o bits 1:0: CHANSPC_E: exponent of channel spacing (here: 2)
	PI_CC_SPIWriteReg(PI_CCxxx0_MDMCFG1, CC1101_DEFVAL_MDMCFG1); // Modem configuration.

	// MODCFG0 Modem configuration: CHANSPC_M: mantissa of channel spacing following this formula:
	//    Df = (Fxosc / 2^18) * (256 + CHANSPC_M) * 2^CHANSPC_E
	//    Here: (26 /  ) * 2016 = 0.199951171875 MHz (200 kHz)
	PI_CC_SPIWriteReg(PI_CCxxx0_MDMCFG0, CC1101_DEFVAL_MDMCFG0); // Modem configuration.

	// DEVIATN: Modem deviation
	// o bit 7:    0   -> not used
	// o bits 6:4: 0   -> DEVIATION_E: deviation exponent
	// o bit 3:    0   -> not used
	// o bits 2:0: 0   -> DEVIATION_M: deviation mantissa
	//
	//   Modulation  Formula
	//
	//   2-FSK    |  
	//   4-FSK    :  Df = (Fxosc / 2^17) * (8 + DEVIATION_M) * 2^DEVIATION_E : Here: 1.5869140625 kHz
	//   GFSK     |
	//
	//   MSK      :  Tx: not well documented, Rx: no effect
	//
	//   OOK      : No effect
	//    
	//reg_word = (radio_parms->deviat_e << 4) + (radio_parms->deviat_m);
	PI_CC_SPIWriteReg(PI_CCxxx0_DEVIATN, CC1101_DEFVAL_DEVIATN); // Modem dev (when FSK mod en)

	// MCSM2: Main Radio State Machine. See documentation.
	//xxnutiu PI_CC_SPIWriteReg(PI_CCxxx0_MCSM2, 0x00); //MainRadio Cntrl State Machine
	PI_CC_SPIWriteReg(PI_CCxxx0_MCSM2, CC1101_DEFVAL_MCSM2); //MainRadio Cntrl State Machine

	// MCSM1: Main Radio State Machine. 
	// o bits 7:6: not used
	// o bits 5:4: CCA_MODE: Clear Channel Indicator 
	//   0 (00): Always clear
	//   1 (01): Clear if RSSI below threshold
	//   2 (10): Always claar unless receiving a packet
	//   3 (11): Claar if RSSI below threshold unless receiving a packet
	// o bits 3:2: RXOFF_MODE: Select to what state it should go when a packet has been received
	//   0 (00): IDLE <== 
	//   1 (01): FSTXON
	//   2 (10): TX
	//   3 (11): RX (stay)
	// o bits 1:0: TXOFF_MODE: Select what should happen when a packet has been sent
	//   0 (00): IDLE <==
	//   1 (01): FSTXON
	//   2 (10): TX (stay)
	//   3 (11): RX 
	
	PI_CC_SPIWriteReg(PI_CCxxx0_MCSM1, CC1101_DEFVAL_MCSM1); //MainRadio Cntrl State Machine

	// MCSM0: Main Radio State Machine.
	// o bits 7:6: not used
	// o bits 5:4: FS_AUTOCAL: When to perform automatic calibration
	//   0 (00): Never i.e. manually via strobe command
	//   1 (01): When going from IDLE to RX or TX (or FSTXON)
	//   2 (10): When going from RX or TX back to IDLE automatically
	//   3 (11): Every 4th time when going from RX or TX to IDLE automatically
	// o bits 3:2: PO_TIMEOUT: 
	//   Value : Exp: Timeout after XOSC start
	//   0 (00):   1: Approx. 2.3 – 2.4 μs
	//   1 (01):  16: Approx. 37 – 39 μs
	//   2 (10):  64: Approx. 149 – 155 μs
	//   3 (11): 256: Approx. 597 – 620 μs
	// o bit 1: PIN_CTRL_EN:   Enables the pin radio control option
	// o bit 0: XOSC_FORCE_ON: Force the XOSC to stay on in the SLEEP state.
	
	PI_CC_SPIWriteReg(PI_CCxxx0_MCSM0, CC1101_DEFVAL_MCSM0); //MainRadio Cntrl State Machine


	// FOCCFG: Frequency Offset Compensation Configuration.
	// o bits 7:6: not used
	// o bit 5:    If set, the demodulator freezes the frequency offset compensation and clock
	//             recovery feedback loops until the CS signal goes high.
	// o bits 4:3: The frequency compensation loop gain to be used before a sync word is detected.
	//   0 (00): K
	//   1 (01): 2K
	//   2 (10): 3K
	//   3 (11): 4K
	// o bit 2: FOC_POST_K: The frequency compensation loop gain to be used after a sync word is detected.
	//   0: Same as FOC_PRE_K
	//   1: K/2
	// o bits 1:0: FOC_LIMIT: The saturation point for the frequency offset compensation algorithm:
	//   0 (00): ±0 (no frequency offset compensation)
	//   1 (01): ±BW CHAN /8
	//   2 (10): ±BW CHAN /4
	//   3 (11): ±BW CHAN /2
	//PI_CC_SPIWriteReg( PI_CCxxx0_FOCCFG,   0x1D); // Freq Offset Compens. Config
	PI_CC_SPIWriteReg(PI_CCxxx0_FOCCFG, CC1101_DEFVAL_FOCCFG); // Freq Offset Compens. Config

	// BSCFG:Bit Synchronization Configuration
	// o bits 7:6: BS_PRE_KI: Clock recovery loop integral gain before sync word
	//   0 (00): Ki
	//   1 (01): 2Ki
	//   2 (10): 3Ki
	//   3 (11): 4Ki
	// o bits 5:4: BS_PRE_KP: Clock recovery loop proportional gain before sync word
	//   0 (00): Kp
	//   1 (01): 2Kp
	//   2 (10): 3Kp
	//   3 (11): 4Kp
	// o bit 3: BS_POST_KI: Clock recovery loop integral gain after sync word
	//   0: Same as BS_PRE_KI
	//   1: Ki/2
	// o bit 2: BS_POST_KP: Clock recovery loop proportional gain after sync word
	//   0: Same as BS_PRE_KP
	//   1: Kp
	// o bits 1:0: BS_LIMIT: Data rate offset saturation (max data rate difference)
	//   0 (00): ±0 (No data rate offset compensation performed)
	//   1 (01): ±3.125 % data rate offset
	//   2 (10): ±6.25 % data rate offset
	//   3 (11): ±12.5 % data rate offset
	PI_CC_SPIWriteReg(PI_CCxxx0_BSCFG, CC1101_DEFVAL_BSCFG); //  Bit synchronization config.

	// AGCCTRL2: AGC Control
	// o bits 7:6: MAX_DVGA_GAIN. Allowable DVGA settings
	//   0 (00): All gain settings can be used
	//   1 (01): The highest gain setting can not be used
	//   2 (10): The 2 highest gain settings can not be used
	//   3 (11): The 3 highest gain settings can not be used
	// o bits 5:3: MAX_LNA_GAIN. Maximum allowable LNA + LNA 2 gain relative to the maximum possible gain.
	//   0 (000): Maximum possible LNA + LNA 2 gain
	//   1 (001): Approx. 2.6 dB below maximum possible gain
	//   2 (010): Approx. 6.1 dB below maximum possible gain
	//   3 (011): Approx. 7.4 dB below maximum possible gain
	//   4 (100): Approx. 9.2 dB below maximum possible gain
	//   5 (101): Approx. 11.5 dB below maximum possible gain
	//   6 (110): Approx. 14.6 dB below maximum possible gain
	//   7 (111): Approx. 17.1 dB below maximum possible gain
	// o bits 2:0: MAGN_TARGET: target value for the averaged amplitude from the digital channel filter (1 LSB = 0 dB).
	//   0 (000): 24 dB
	//   1 (001): 27 dB
	//   2 (010): 30 dB
	//   3 (011): 33 dB
	//   4 (100): 36 dB
	//   5 (101): 38 dB
	//   6 (110): 40 dB
	//   7 (111): 42 dB

	PI_CC_SPIWriteReg(PI_CCxxx0_AGCCTRL2, CC1101_DEFVAL_AGCCTRL2); // AGC control.

	// AGCCTRL1: AGC Control
	// o bit 7: not used
	// o bit 6: AGC_LNA_PRIORITY: Selects between two different strategies for LNA and LNA 2 gain
	//   0: the LNA 2 gain is decreased to minimum before decreasing LNA gain
	//   1: the LNA gain is decreased first.
	// o bits 5:4: CARRIER_SENSE_REL_THR: Sets the relative change threshold for asserting carrier sense
	//   0 (00): Relative carrier sense threshold disabled
	//   1 (01): 6 dB increase in RSSI value
	//   2 (10): 10 dB increase in RSSI value
	//   3 (11): 14 dB increase in RSSI value
	// o bits 3:0: CARRIER_SENSE_ABS_THR: Sets the absolute RSSI threshold for asserting carrier sense. 
	//   The 2-complement signed threshold is programmed in steps of 1 dB and is relative to the MAGN_TARGET setting.
	//   0 is at MAGN_TARGET setting.
	PI_CC_SPIWriteReg(PI_CCxxx0_AGCCTRL1, CC1101_DEFVAL_AGCCTRL1); // AGC control.

	// AGCCTRL0: AGC Control
	// o bits 7:6: HYST_LEVEL: Sets the level of hysteresis on the magnitude deviation
	//   0 (00): No hysteresis, small symmetric dead zone, high gain
	//   1 (01): Low hysteresis, small asymmetric dead zone, medium gain
	//   2 (10): Medium hysteresis, medium asymmetric dead zone, medium gain
	//   3 (11): Large hysteresis, large asymmetric dead zone, low gain
	// o bits 5:4: WAIT_TIME: Sets the number of channel filter samples from a gain adjustment has
	//   been made until the AGC algorithm starts accumulating new samples.
	//   0 (00):  8
	//   1 (01): 16
	//   2 (10): 24
	//   3 (11): 32
	// o bits 3:2: AGC_FREEZE: Control when the AGC gain should be frozen.
	//   0 (00): Normal operation. Always adjust gain when required.
	//   1 (01): The gain setting is frozen when a sync word has been found.
	//   2 (10): Manually freeze the analogue gain setting and continue to adjust the digital gain. 
	//   3 (11): Manually freezes both the analogue and the digital gain setting. Used for manually overriding the gain.
	// o bits 0:1: FILTER_LENGTH: 
	//   2-FSK, 4-FSK, MSK: Sets the averaging length for the amplitude from the channel filter.    |  
	//   ASK ,OOK: Sets the OOK/ASK decision boundary for OOK/ASK reception.
	//   Value : #samples: OOK/ASK decixion boundary
	//   0 (00):        8: 4 dB
	//   1 (01):       16: 8 dB
	//   2 (10):       32: 12 dB
	//   3 (11):       64: 16 dB  
	PI_CC_SPIWriteReg(PI_CCxxx0_AGCCTRL0, CC1101_DEFVAL_AGCCTRL0); // AGC control.

	// FREND1: Front End RX Configuration
	// o bits 7:6: LNA_CURRENT: Adjusts front-end LNA PTAT current output
	// o bits 5:4: LNA2MIX_CURRENT: Adjusts front-end PTAT outputs
	// o bits 3:2: LODIV_BUF_CURRENT_RX: Adjusts current in RX LO buffer (LO input to mixer)
	// o bits 1:0: MIX_CURRENT: Adjusts current in mixer
	//xnutiu PI_CC_SPIWriteReg(PI_CCxxx0_FREND1, 0xB6); // Front end RX configuration.
	PI_CC_SPIWriteReg(PI_CCxxx0_FREND1, CC1101_DEFVAL_FREND1); // Front end RX configuration.

	// FREND0: Front End TX Configuration
	// o bits 7:6: not used
	// o bits 5:4: LODIV_BUF_CURRENT_TX: Adjusts current TX LO buffer (input to PA). The value to use
	//   in this field is given by the SmartRF Studio software
	// o bit 3: not used
	// o bits 1:0: PA_POWER: Selects PA power setting. This value is an index to the PATABLE, 
	//   which can be programmed with up to 8 different PA settings. In OOK/ASK mode, this selects the PATABLE
	//   index to use when transmitting a ‘1’. PATABLE index zero is used in OOK/ASK when transmitting a ‘0’. 
	//   The PATABLE settings from index ‘0’ to the PA_POWER value are used for ASK TX shaping, 
	//   and for power ramp-up/ramp-down at the start/end of transmission in all TX modulation formats.
	PI_CC_SPIWriteReg(PI_CCxxx0_FREND0, CC1101_DEFVAL_FREND0); // Front end RX configuration.

	// FSCAL3: Frequency Synthesizer Calibration
	// o bits 7:6: The value to write in this field before calibration is given by the SmartRF
	//   Studio software.
	// o bits 5:4: CHP_CURR_CAL_EN: Disable charge pump calibration stage when 0.
	// o bits 3:0: FSCAL3: Frequency synthesizer calibration result register.

	PI_CC_SPIWriteReg(PI_CCxxx0_FSCAL3, CC1101_DEFVAL_FSCAL3); // Frequency synthesizer cal.

	// FSCAL2: Frequency Synthesizer Calibration
	PI_CC_SPIWriteReg(PI_CCxxx0_FSCAL2, CC1101_DEFVAL_FSCAL2); // Frequency synthesizer cal.
	PI_CC_SPIWriteReg(PI_CCxxx0_FSCAL1, CC1101_DEFVAL_FSCAL1); // Frequency synthesizer cal.
	PI_CC_SPIWriteReg(PI_CCxxx0_FSCAL0, CC1101_DEFVAL_FSCAL0); // Frequency synthesizer cal.

	PI_CC_SPIWriteReg(PI_CCxxx0_FSTEST, 0x59); // Frequency synthesizer cal.
	PI_CC_SPIWriteReg(PI_CCxxx0_PATABLE, PA_LongDistance);
	// TEST2: Various test settings. The value to write in this field is given by the SmartRF Studio software.
	//PI_CC_SPIWriteReg( PI_CCxxx0_TEST2,    0x88); // Various test settings.
	PI_CC_SPIWriteReg(PI_CCxxx0_TEST2, CC1101_DEFVAL_TEST2); // Various test settings.

	// TEST1: Various test settings. The value to write in this field is given by the SmartRF Studio software.
	//PI_CC_SPIWriteReg( PI_CCxxx0_TEST1,    0x31); // Various test settings.
	PI_CC_SPIWriteReg(PI_CCxxx0_TEST1, CC1101_DEFVAL_TEST1); // Various test settings.
	// TEST0: Various test settings. The value to write in this field is given by the SmartRF Studio software.
	//PI_CC_SPIWriteReg( PI_CCxxx0_TEST0,    0x09); // Various test settings.
	PI_CC_SPIWriteReg(PI_CCxxx0_TEST0, CC1101_DEFVAL_TEST0); // Various test settings.

	return 0;
}

// ------------------------------------------------------------------------------------------------
// Print status registers to stderr
int  print_radio_status()
// ------------------------------------------------------------------------------------------------
{
	uint8_t regs[14];
	uint8_t reg_index = PI_CCxxx0_PARTNUM;
	int ret = 0;

	memset(regs, 0, 14);

	while ((ret == 0) && (reg_index <= PI_CCxxx0_RXBYTES))
	{
		ret = PI_CC_SPIReadStatus(reg_index, &regs[reg_index - PI_CCxxx0_PARTNUM]);
		reg_index++;
	}

	if (ret != 0)
	{
		fprintf(stderr, "RADIO: read status register %02X failed\n", reg_index - 1);
		return ret;
	}

	fprintf(stderr, "Part number ...........: %d\n", regs[0]);
	fprintf(stderr, "Version ...............: %d\n", regs[1]);
	fprintf(stderr, "Freq offset estimate ..: %d\n", regs[2]);
	fprintf(stderr, "CRC OK ................: %d\n", ((regs[3] & 0x80) >> 7));
	fprintf(stderr, "LQI ...................: %d\n", regs[3] & 0x7F);
	fprintf(stderr, "RSSI ..................: %.1f dBm\n", rssi_dbm(regs[4]));
	fprintf(stderr, "Radio FSM state .......: %s\n", state_names[regs[5] & 0x1F]);
	fprintf(stderr, "WOR time ..............: %d\n", ((regs[6] << 8) + regs[7]));
	fprintf(stderr, "Carrier Sense .........: %d\n", ((regs[8] & 0x40) >> 6));
	fprintf(stderr, "Preamble Qual Reached .: %d\n", ((regs[8] & 0x20) >> 5));
	fprintf(stderr, "Clear channel .........: %d\n", ((regs[8] & 0x10) >> 4));
	fprintf(stderr, "Start of frame delim ..: %d\n", ((regs[8] & 0x08) >> 3));
	fprintf(stderr, "GDO2 ..................: %d\n", ((regs[8] & 0x04) >> 2));
	fprintf(stderr, "GDO0 ..................: %d\n", ((regs[8] & 0x01)));
	fprintf(stderr, "VCO VC DAC ............: %d\n", regs[9]);
	fprintf(stderr, "FIFO Tx underflow .....: %d\n", ((regs[10] & 0x80) >> 7));
	fprintf(stderr, "FIFO Tx bytes .........: %d\n", regs[10] & 0x7F);
	fprintf(stderr, "FIFO Rx overflow ......: %d\n", ((regs[11] & 0x80) >> 7));
	fprintf(stderr, "FIFO Rx bytes .........: %d\n", regs[11] & 0x7F);
	fprintf(stderr, "RC CRTL0 ..............: %d\n", (regs[12] & 0x7F));
	fprintf(stderr, "RC CRTL1 ..............: %d\n", (regs[13] & 0x7F));	
	//packet length
	fprintf(stderr, "PKT LEN ..............: %d\n", radio_get_packet_length());
	//Channel and freq
	fprintf(stderr, "Channel ..............: %d\n", radio_get_channel());
	
	return ret;
}


// ------------------------------------------------------------------------------------------------
// Set packet length
int radio_set_packet_length(uint8_t pkt_len)
// ------------------------------------------------------------------------------------------------
{
	return PI_CC_SPIWriteReg(PI_CCxxx0_PKTLEN, pkt_len); // Packet length.
}

// ------------------------------------------------------------------------------------------------
// Get packet length
uint8_t radio_get_packet_length()
// ------------------------------------------------------------------------------------------------
{
	uint8_t pkt_len;
	PI_CC_SPIReadReg(PI_CCxxx0_PKTLEN, &pkt_len); // Packet length.
	return pkt_len;
}

// ------------------------------------------------------------------------------------------------
// Set packet length
int radio_set_channel(uint8_t channel)
// ------------------------------------------------------------------------------------------------
{
	return PI_CC_SPIWriteReg(PI_CCxxx0_CHANNR, channel); // Packet length.
}

// ------------------------------------------------------------------------------------------------
// Get channel
uint8_t radio_get_channel()
// ------------------------------------------------------------------------------------------------
{
	uint8_t chan;
	PI_CC_SPIReadReg(PI_CCxxx0_CHANNR, &chan); // Packet length.
	return chan;
}
// ------------------------------------------------------------------------------------------------
// Wait for the reception or transmission to finish
void radio_wait_free()
// ------------------------------------------------------------------------------------------------
{
	uint8_t timeout = 0;
	while((radio_int_data.mode == RADIOMODE_TX) || (radio_int_data.packet_receive) || (radio_int_data.packet_send))
	{
		usleep(50);
		timeout ++;
		if (timeout > RADIO_WATCHDOG_TIMEOUT/50){
			//we should reset the radio - it must be hanging
			verbprintf(4,"TOUT MODE=%i packet_receive=%i packet_send=%i", radio_int_data.mode, radio_int_data.packet_receive, radio_int_data.packet_send);
			reset_radio("ERROR: Tx/Rx IRQ state change timer expired.");
			return;
		}
	}
}

// ------------------------------------------------------------------------------------------------
// Initialize for Rx mode
void radio_init_rx()
// ------------------------------------------------------------------------------------------------
{
	radio_int_data.mode = RADIOMODE_RX;
	PI_CC_SPIWriteReg(PI_CCxxx0_IOCFG2, CC1101_DEFVAL_IOCFG2); // GDO2 output pin config RX mode
}

/*
* radio_process_packet
* decode a received packet
*/
uint8_t radio_process_packet() {
	if (radio_int_data.mode == RADIOMODE_UNDEF_CONDITION){
		reset_radio("RADIO_UNDEFINED_STATE");
		return 0;
	};
	if (radio_int_data.rx_buff_idx == radio_int_data.rx_buff_read_idx)
		return 0; //nothing to do, no new packets in the receive buffer. This should actually not happend.
	uint8_t no_of_packets = 0;
	while (radio_int_data.rx_buff_idx != radio_int_data.rx_buff_read_idx) { //packets have been received and are waiting processing
		circular_incr(radio_int_data.rx_buff_read_idx);
		//verbprintf(3, "rx %i rxread %i\n", radio_int_data.rx_buff_idx, radio_int_data.rx_buff_read_idx);
		//rx_ccpacket_buf[radio_int_data.rx_buff_read_idx].printAsHex();
		SWPACKET swPacket = SWPACKET(&rx_ccpacket_buf[radio_int_data.rx_buff_read_idx]);
		verbprintf(4,"ADDR %i RSSI %.1f dB \n", swPacket.srcAddr ,rssi_dbm(swPacket.rssi));
		char buff[512];
		verbprintf(4, "RCV: SW packet: %s", swPacket.as_string(buff));
		no_of_packets++;
		switch (swPacket.function)
		{
		case SWAPFUNCT_ALARM: //Alarm packet. Error reason is coded in the regAddr field
			{	
				//char valbuff[sizeof(CCPACKET::data)];
				//swPacket.val_to_string(valbuff);
				//verbprintf(1, "ALARM: src: %i %s \n", swPacket.srcAddr,  valbuff);
				//mqtt_send_alarm(swPacket.srcAddr, valbuff);
				verbprintf(1, "\nALARM: src: %i %i \n", swPacket.srcAddr,  swPacket.regAddr);
				mqtt_send_alarm(swPacket.srcAddr, getAlarmText(swPacket.regAddr));
			}
			break;			
		case SWAPFUNCT_ACK: //ack packet. Might contain an error in the regAddr field
			verbprintf(4, "ACK: ERR: %s \n", getStackErrorText(swPacket.regAddr));
			break;
		case SWAPFUNCT_STA: //status packet
			if (swPacket.destAddr == MASTER_ADDRESS || swPacket.destAddr == BROADCAST_ADDRESS) {
					//std::string val; 
					//val.insert(0, swPacket.value.data, swPacket.value.length);
					//val.c_str();
					/* a buffer for holding a string representation of the payload */
					char valbuff[sizeof(CCPACKET::data)];
					swPacket.val_to_string(valbuff);
					verbprintf(2,"\nReceived stat from ADDR %i REGID %i VAL %s\n", swPacket.srcAddr, swPacket.regId, valbuff);
					//not very elegant to directly send to mqtt, but for now let's leave it so
					//We should rather use a buffer and decouple packet decode from sending responses
					//but mqtt runs in another thread, so it's not a big deal
					//std::ostringstream s_msg;
					//s_msg << std::to_string(swPacket.regId) << ':' << valbuff;
					//const std::string msg = s_msg.str();
					mqtt_send_actor_state(swPacket.srcAddr, swPacket.regId, &swPacket.value);
			}
			break;
		case SWAPFUNCT_CMD:
			// Command not addressed to us?
			if (swPacket.destAddr != MASTER_ADDRESS)
				break;
			// TODO: implement me
		case SWAPFUNCT_QRY:
			// Query addressed to us?
			if (swPacket.destAddr != MASTER_ADDRESS)
				break;
			if (swPacket.destAddr != swPacket.regAddr)
				break;
			/*if (swPacket.regId == REGI_DEVADDRESS){
				printf("Address request ");
				int newNodeAddr = registerNewNode();
				//SWCOMMAND response = SWCOMMAND(0,0,REGI_DEVADDRESS,&newNodeAddr, 1);
			}*/
			break;
		default:
			verbprintf(1, "ERR: Unknown swapfunc %i", swPacket.function);
			break;
		}
		
		//link management
		if (swPacket.destAddr == MASTER_ADDRESS){ //packet addressed to us
			//Aknowledgement of a previously sent packet
			ackAwaitQueue.ack_packet(swPacket.srcAddr, swPacket.packetNo);
			//make sure this radio link is registered to trigger later the heartbeat
			registrar.registerLink(swPacket.srcAddr,swPacket.lqi);
			
			if (swPacket.request_ack) {//ack requested
				SWACK ack = SWACK(swPacket.srcAddr,swPacket.packetNo,STACKERR_OK);
				verbprintf(1, "Sending ACK to %i", swPacket.srcAddr);
				enque_tx_packet(&ack, false);
			}
		}		
	}
	return no_of_packets;
}

bool transmit_packets() { //sends a radio packet 
	//add_to_tx_queue(p_packet);
	//look if there is anything to be sent in the tx buff
	if (radio_int_data.tx_buff_idx_sent == radio_int_data.tx_buff_idx_ins) { //nothing added to the buff, return
		return true;
	}else {
		while (radio_int_data.tx_buff_idx_sent != radio_int_data.tx_buff_idx_ins) {
			//send all packets from the buffer
			circular_incr(radio_int_data.tx_buff_idx_sent);
			//cout << "-----" << (int) radio_int_data.tx_buff_idx_ins << " **  " << (int) radio_int_data.tx_buff_idx_sent <<"\n"<< std::flush;
			CCPACKET *packet = &tx_ccpacket_buf[radio_int_data.tx_buff_idx_sent];
			//verbprintf(3, "Read from buff at %i %s\n", radio_int_data.tx_buff_idx_sent, packet->to_string().c_str() );
			radio_wait_free();
			//PI_CC_SPIWriteReg(PI_CCxxx0_IOCFG2, 0x02); // GDO2 output pin config TX mode
			PI_CC_SPIStrobe(PI_CCxxx0_SIDLE);// Enter IDLE state
			PI_CC_SPIStrobe(PI_CCxxx0_SFTX); //flush the TX fifo buffer
			PI_CC_SPIStrobe(PI_CCxxx0_SFRX); //flush the RX fifo buffer
			PI_CC_SPIWriteReg(PI_CCxxx0_PATABLE, PA_LongDistance); //set transmit power
			//set freq (should not be neccessary, but we had some strange freq shifts)
			PI_CC_SPIWriteReg(PI_CCxxx0_FREQ2, CC1101_DEFVAL_FREQ2_433); // Freq control word, high byte
			PI_CC_SPIWriteReg(PI_CCxxx0_FREQ1, CC1101_DEFVAL_FREQ1_433); // Freq control word, mid byte.
			PI_CC_SPIWriteReg(PI_CCxxx0_FREQ0, CC1101_DEFVAL_FREQ0_433);  // Freq control word, low byte.
			PI_CC_SPIWriteReg(PI_CCxxx0_CHANNR, radio_parameters.channel); // Channel number
			
			PI_CC_SPIWriteBurstReg(PI_CCxxx0_TXFIFO, &packet->length, 1); //must write burst or the radio hangs
			PI_CC_SPIWriteBurstReg(PI_CCxxx0_TXFIFO, (uint8_t *)packet->data, packet->length); //write the payload data
			
			radio_int_data.mode = RADIOMODE_TX; //incoming interrupt must be handled as TX 
			
			PI_CC_SPIStrobe(PI_CCxxx0_STX); // Kick-off Tx
			if (!wait_for_tx_end(20)) {
				//timeout
				verbprintf(0, "ERROR: transmit failed");
			}; // Wait max 10ms
			//radio_wait_free();
			//PI_CC_SPIStrobe(PI_CCxxx0_SFTX); // Flush Tx FIFO
			verbprintf(4, "TX: %s\n", packet->to_string().c_str() );
			//packet must be aknowledged, add it to the ack buffer			
			if (!wait_for_state(CCxxx0_STATE_RX, 100)) {
				//timeout
				reset_radio("ERROR: UNDEFINED STATE AFTER TX"); //should not happend, but if, at least try to leave the radio in a defined state
				//return false;
			};
			// Declare to be in Rx state
			radio_int_data.mode = RADIOMODE_RX;			
		}
		return true;
	}

}

//reentrant, thread-safe function for inserting the packets to be sent in the transmit buffer
void enque_tx_packet(SWPACKET* p_packet, bool awaitAck){ 
	circular_incr(radio_int_data.tx_buff_idx_ins);	
	p_packet->prepare(&tx_ccpacket_buf[radio_int_data.tx_buff_idx_ins]);
	//printf(" ----EN---- to insert AT %i : %s ", radio_int_data.tx_buff_idx_ins , p_packet->to_string().c_str());
	//printf(" ----EN---- inserted AT %i : %s \n", radio_int_data.tx_buff_idx_ins, tx_ccpacket_buf[radio_int_data.tx_buff_idx_ins].to_string().c_str());
	transmit_packets();
	if (awaitAck)
		ackAwaitQueue.append(p_packet->destAddr, p_packet->packetNo, tx_ccpacket_buf[radio_int_data.tx_buff_idx_ins]);
	//printf(" ----EN---- awaitq AT %i : %s \n", radio_int_data.tx_buff_idx_ins, tx_ccpacket_buf[radio_int_data.tx_buff_idx_ins].to_string().c_str());
}

void resend_packet(const CCPACKET* p_packet){ 
	circular_incr(radio_int_data.tx_buff_idx_ins);		
	//printf(" ----RE---- to insert AT %i : %s \n", radio_int_data.tx_buff_idx_ins , p_packet->to_string().c_str());
	tx_ccpacket_buf[radio_int_data.tx_buff_idx_ins].copy(p_packet); //copy the packet to the transmit buffer
	//printf(" ----RE--- inserted AT %i : %s \n", radio_int_data.tx_buff_idx_ins, tx_ccpacket_buf[radio_int_data.tx_buff_idx_ins].to_string().c_str());
	transmit_packets();
}
