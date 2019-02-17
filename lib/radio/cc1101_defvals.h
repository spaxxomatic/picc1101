
/**
 * CC1101 configuration registers - Default values extracted from SmartRF Studio
 *
 * Configuration:
 *
 * Deviation = 20.629883 
 * Base frequency = 867.999939 
 * Carrier frequency = 867.999939 
 * Channel number = 0 
 * Carrier frequency = 867.999939 
 * Modulated = true 
 * Modulation format = GFSK 
 * Manchester enable = false
 * Data whitening = off
 * Sync word qualifier mode = 30/32 sync word bits detected 
 * Preamble count = 4 
 * Channel spacing = 199.951172 
 * Data rate = 38.3835 Kbps
 * RX filter BW = 101.562500 
 * Data format = Normal mode 
 * Length config = Variable packet length mode. Packet length configured by the first byte after sync word 
 * CRC enable = true 
 * Packet length = 255 
 * Device address = 1 
 * Address config = Enable address check
 * Append status = Append two status bytes to the payload of the packet. The status bytes contain RSSI and
 * LQI values, as well as CRC OK
 * CRC autoflush = false 
 * PA ramping = false 
 * TX power = C0
 * GDO0 mode = Asserts when sync word has been sent / received, and de-asserts at the end of the packet.
 * In RX, the pin will also de-assert when a packet is discarded due to address or maximum length filtering
 * or when the radio enters RXFIFO_OVERFLOW state. In TX the pin will de-assert if the TX FIFO underflows
 * Settings optimized for low current consumption
 */
#define CC1101_DEFVAL_IOCFG2     0x29        // GDO2 Output Pin Configuration
#define CC1101_DEFVAL_IOCFG1     0x2E        // GDO1 Output Pin Configuration
#define CC1101_DEFVAL_IOCFG0     0x06        // GDO0 Output Pin Configuration
#define CC1101_DEFVAL_FIFOTHR    0x07        // RX FIFO and TX FIFO Thresholds
#define CC1101_DEFVAL_SYNC1      0xB5        // Synchronization word, high byte
#define CC1101_DEFVAL_SYNC0      0x47        // Synchronization word, low byte
#define CC1101_DEFVAL_PKTLEN     0x3D        // Packet Length
// def pana #define CC1101_DEFVAL_PKTCTRL1   0x06        // Packet Automation Control
// def pana #define CC1101_DEFVAL_PKTCTRL0   0x05        // Packet Automation Control
//#define CC1101_DEFVAL_PKTLEN     0xFF        // Packet Length
#define CC1101_DEFVAL_PKTCTRL1   0x06        // Packet Automation Control
#define CC1101_DEFVAL_PKTCTRL0   0x05        // Packet Automation Control
#define CC1101_DEFVAL_ADDR       0xFF        // Device Address
#define CC1101_DEFVAL_CHANNR     0x04        // Channel Number
#define CC1101_DEFVAL_FSCTRL1    0x08        // Frequency Synthesizer Control
#define CC1101_DEFVAL_FSCTRL0    0x00        // Frequency Synthesizer Control
// Carrier frequency = 868 MHz
#define CC1101_DEFVAL_FREQ2_868  0x21        // Frequency Control Word, High Byte
#define CC1101_DEFVAL_FREQ1_868  0x62        // Frequency Control Word, Middle Byte
#define CC1101_DEFVAL_FREQ0_868  0x76        // Frequency Control Word, Low Byte
// Carrier frequency = 902 MHz
#define CC1101_DEFVAL_FREQ2_915  0x22        // Frequency Control Word, High Byte
#define CC1101_DEFVAL_FREQ1_915  0xB1        // Frequency Control Word, Middle Byte
#define CC1101_DEFVAL_FREQ0_915  0x3B        // Frequency Control Word, Low Byte
// Carrier frequency = 433 MHz
#define CC1101_DEFVAL_FREQ2_433  0x10        // Frequency Control Word, High Byte
#define CC1101_DEFVAL_FREQ1_433  0xB1        // Frequency Control Word, Middle Byte
#define CC1101_DEFVAL_FREQ0_433  0x86        // Frequency Control Word, Low Byte

//#define CC1101_DEFVAL_MDMCFG4    0xCA        // Modem Configuration
#define CC1101_DEFVAL_MDMCFG4    0x8A        // Modem Configuration 200 khz bw
#define CC1101_DEFVAL_MDMCFG3    0x83        // Modem Configuration
//#define CC1101_DEFVAL_MDMCFG2    0x93        // Modem Configuration
#define CC1101_DEFVAL_MDMCFG2    0x13        // Modem Configuration bit 7 = 0 (enable digital dc block)
#define CC1101_DEFVAL_MDMCFG1    0x22        // Modem Configuration
#define CC1101_DEFVAL_MDMCFG0    0xF8        // Modem Configuration
#define CC1101_DEFVAL_DEVIATN    0x35        // Modem Deviation Setting

#define CC1101_DEFVAL_MCSM2      0x07        // Main Radio Control State Machine Configuration
//#define CC1101_DEFVAL_MCSM1      0x30        // Main Radio Control State Machine Configuration - this value enables the CCA_MODE - clear channel indication
//#define CC1101_DEFVAL_MCSM1      0x20        // Main Radio Control State Machine Configuration - going to IDLE after TX
#define CC1101_DEFVAL_MCSM1      0b101111      // Main Radio Control State Machine Configuration - going to RX after RX, going to RX after TX

#define CC1101_DEFVAL_MCSM0      0b11000        // Main Radio Control State Machine Configuration
#define CC1101_DEFVAL_FOCCFG     0x16        // Frequency Offset Compensation Configuration
#define CC1101_DEFVAL_BSCFG      0x6C        // Bit Synchronization Configuration
#define CC1101_DEFVAL_AGCCTRL2   0x43        // AGC Control

#define CC1101_DEFVAL_AGCCTRL1   0x40        // AGC Control
#define CC1101_DEFVAL_AGCCTRL0   0x91        // AGC Control
#define CC1101_DEFVAL_WOREVT1    0x87        // High Byte Event0 Timeout
#define CC1101_DEFVAL_WOREVT0    0x6B        // Low Byte Event0 Timeout
#define CC1101_DEFVAL_WORCTRL    0xFB        // Wake On Radio Control
#define CC1101_DEFVAL_FREND1     0x56        // Front End RX Configuration
#define CC1101_DEFVAL_FREND0     0x10        // Front End TX Configuration
#define CC1101_DEFVAL_FSCAL3     0xE9        // Frequency Synthesizer Calibration
#define CC1101_DEFVAL_FSCAL2     0x2A        // Frequency Synthesizer Calibration
#define CC1101_DEFVAL_FSCAL1     0x00        // Frequency Synthesizer Calibration
#define CC1101_DEFVAL_FSCAL0     0x1F        // Frequency Synthesizer Calibration
#define CC1101_DEFVAL_RCCTRL1    0x41        // RC Oscillator Configuration
#define CC1101_DEFVAL_RCCTRL0    0x00        // RC Oscillator Configuration
#define CC1101_DEFVAL_FSTEST     0x59        // Frequency Synthesizer Calibration Control
#define CC1101_DEFVAL_PTEST      0x7F        // Production Test
#define CC1101_DEFVAL_AGCTEST    0x3F        // AGC Test
#define CC1101_DEFVAL_TEST2      0x81        // Various Test Settings
#define CC1101_DEFVAL_TEST1      0x35        // Various Test Settings
#define CC1101_DEFVAL_TEST0      0x09        // Various Test Settings
// PATABLE values
#define PA_LowPower               0x60
#define PA_LongDistance           0xC0
//#define PA_LongDistance           0xC8
