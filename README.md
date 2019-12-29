spaxxserver
========

Connects an Orange Pi or Raspberry Pi to CC1101 RF module and implements the spaxstack IoT protocol

- [picc1101](#picc1101)
- [Introduction](#introduction)
- [Disclaimer](#disclaimer)
- [Installation and basic usage](#installation-and-basic-usage)
  - [Prerequisites](#prerequisites)
  - [Obtain the code](#obtain-the-code)
  - [Compilation](#compilation)
  - [Run test programs](#run-test-programs)
  - [Process priority](#process-priority)
    - [Specify a higher priority at startup](#specify-a-higher-priority-at-startup)
    - [Engage the "real time" priority](#engage-the-real-time-priority)
  - [Program options](#program-options)
  - [Detailed options](#detailed-options)
    - [Verbosity level (-v)](#verbosity-level--v)
    - [Test routines (-t)](#test-routines--t)

# Introduction
The RF module based on the Texas Instruments (Chipcon) chip CC1101  is a OOK/2-FSK/4-FSK/MSK/GFSK low power (~10dBm) digital transceiver working in the 315, 433 and 868 MHz ISM bands. It is used as a radio interface for conecting various sensors/actors in a home automation scenario. 

These RF modules are available from a variety of sellers on eBay. Search for words `CC1101` and `433 MHz`.

The CC1101 chip has all the necessary features to cover the OSI layer 1 (physical). In the context of this project, it is used in packet mode. 

The CC1101 chip is interfaced using a SPI bus that is implemented natively on the Orange/Raspberry PI and can be accessed through the `spidev` library. In addition two GPIOs must be used to support the handling of the CC1101 Rx and Tx FIFOs. For convenience GPIO-24 and GPIO-25 close to the SPI bus on the Orange-Pi are chosen to be connected to the GDO0 and GDO2 lines of the CC1101 respectively. The WiringPi library is used to support the GPIO interrupt handling.

The CC1101 data sheet is available [here](www.ti.com/lit/ds/symlink/cc1101.pdf).

# Disclaimer
This software is provided "as is" and "with all faults.". I give no warranty of any kind and cannot guarantee the safety, suitability, lack of viruses, inaccuracies, typographical errors, or other harmful components inside the SW. You are solely responsible for the protection of your equipment and backup of your data, and I am not liable for any damages you may suffer in connection with using, modifying, or distributing this software.

You are supposed to use the CC1101 radio modules according to your local radio spectrum regulations. 

# Installation and basic usage
## Prerequisites
This has been tested successfully on a Raspberry Pi version 1 B with kernel 3.12.36 and on Orange Pi with an Armbian with mainline kernel 4.14.y

SPI drivers for Raspberry Pi:
For best performance you will need the DMA based SPI driver for BCM2708 found [here](https://github.com/notro/spi-bcm2708.git) After successful compilation you will obtain a kernel module that is to be stored as `/lib/modules/$(uname -r)/kernel/drivers/spi/spi-bcm2708.ko` 

SPI Drivers for Orange Pi:
The DMA based SPI driver for Orange PI can be found [here](https://github.com/notro/spi-bcm2708.git) 
After successful compilation you will obtain a kernel module that is to be stored as `/lib/modules/$(uname -r)/kernel/drivers/spi/spi-bcm2708.ko` 
// TODO: check info about SPI on OPi

You will have to download and install the WiringPi library found [here](http://wiringpi.com/) 

The process relies heavily on interrupts that must be served in a timely manner. You are advised to reduce the interrupts activity by removing USB connected devices as much as possible.

## Obtain the code
Just clone this repository in a local folder of your choice on the Raspberry/Orange Pi

## Compilation
You can compile on the Raspberry Pi v.1 as it doesn't take too much time even on the single core BCM2735. You are advised to activate the -O3 optimization:
  - `CFLAGS=-O3 make`

The result is the `spaxxserver` executable in the /out directory

## Process priority
You may experience better behaviour (less timeouts) depending on the speed of the link when raising the prioriry of the process. Interrupts are already served with high priority (-56) with the WiringPi library. The main process may need a little boost as well though

### Specify a higher priority at startup
You can use the `nice` utility: `sudo nice -n -20 ./picc1101 options...` 
This will set the priority to 0 and is the minimum you can obtain with the `nice` commmand. The lower the priority figure the higher the actual priority. 

### Engage the "real time" priority
You can use option -T of the program to get an even lower priority of -2 for a so called "real time" scheduling. This is not real time actually but will push the priority figure into the negative numbers. It has been implemented with the WiringPi piHiPri method and -2 is the practical lowest figure possible before entering into bad behaviour that might make a cold reboot necessary. Note that this is the same priority as the watchdog.

## Program options
 <pre><code>
  -i, --ini=INIFILE   INI file, (default : ./spaxxserver.ini)
  -H, --long-help            Print a long help and exit
  -s, --radio-status         Print radio status and exit
  -v, --verbose=VERBOSITY_LEVEL   Verbosity level: 0 quiet else verbose level
                             (default : quiet)
  -?, --help                 Give this help list
      --usage                Give a short usage message
      --version              Print program version
</code></pre>

Note: variable length blocks are not implemented yet.

## Detailed options
### Verbosity level (-v)
It ranges from 0 to 4:
  - 0: nothing at all
  - 1: Errors and some warnings and one line summary for each block sent or received
  - 2: Adds details on received blocks like RSSI and LQI
  - 3: Adds details on interrupt calls
  - 4: Adds full hex dump of sent and received blocks

Be aware that printing out to console might cause problems with transfer speeds and interfere with real time operations.

## Relay the KST chat
As a sidenote this is the way you can relay the KST chat (that is port 23000 of a specific server) through this radio link. 

On one end that has a connection to the Internet (say 10.0.1.3) do the port forwarding:
  - `sudo /sbin/iptables -t nat -A PREROUTING -p tcp -i ax0 --dport 23000 -j DNAT --to-destination 188.165.198.144:23000`
  - `sudo /sbin/iptables -t nat -A POSTROUTING -p tcp --dport 23000 -j MASQUERADE`

On the other end (say 10.0.1.7) use a telnet chat client such as the [modified colrdx for KST](https://github.com/f4exb/colrdx) and connect using the one end's IP address and port 23000:
  - `colrdx -c (callsign) -k 10.0.1.3 23000`

# Details of the design
## Multiple block handling
The CC1101 can transmit blocks up to 255 bytes. There is a so called "Infinite block" option but we don't want to use it here. In order to transmit larger blocks which is necessary for concatenated KISS frames or effective MTUs larger than 255 bytes we simply use a block countdown scheme. Each block has a header of two bytes:
  - Byte 0 is the length of the actual block of data inside the fixed size block 
  - Byte 1 is a block countdown counter that is decremented at each successive block belonging to the same larger block to transmit. Single blocks are simply transmitted with a countdown of 0 and so is the last block of a multiple block group.

When transmitting a sequence of blocks the first block is set to the result of the Euclidean division of the greater block size by the radio block size and it is decremented at each successive radio block to send until it reaches zero for the last block.

At the reception end the radio block countdown is checked and if it is not zero it will expect a next block with a countdown counter decremented by one until it receives a block with a countdown of zero.

This allows the transmission of greater blocks of up to 2^16 = 64k = 65536 bytes.

If any block is corrupted (bad CRC) or if its countdown counter is out of sequence then the whole greater block is discarded. This effectively puts a limit on the acceptable fragmentation depending on the quality of the link.
