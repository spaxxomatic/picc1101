spaxmatic
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
  -l, --packet-delay=DELAY_UNITS   Delay between successive radio blocks when
                             transmitting a larger block. In 2-FSK byte
                             duration units. (default 30)
  -s, --radio-status         Print radio status and exit
  -t, --test-mode=TEST_SCHEME   Test scheme, See long help (-H) option fpr
                             details (default : 0 no test)
      --tnc-switchover-delay=SWITCHOVER_DELAY_US
                             FUTUR USE: TNC switchover delay in microseconds
                             (default: 0 inactive)
  -T, --real-time            Engage so called "real time" scheduling (defalut
                             0: no)
  -v, --verbose=VERBOSITY_LEVEL   Verbosity level: 0 quiet else verbose level
                             (default : quiet)
  -W, --whitening            Activate whitening (default off)
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

Be aware that printing out to console takes time and might cause problems when transfer speeds and interactivity increase.

### Radio interface speeds (-R)
 <pre><code>
Value: Rate (Baud):
 0     50 (experimental)
 1     110 (experimental)
 2     300 (experimental)
 3     600
 4     1200
 5     2400
 6     4800
 7     9600
 8     14400
 9     19200
10     28800
11     38400
12     57600
13     76800
14     115200
15     250000
16     500000 (300000 for 4-FSK)
</code></pre>

### Modulations (-M)
 <pre><code>
Value: Scheme:
0      OOK
1      2-FSK
2      4-FSK
3      MSK
4      GFSK
</code></pre>

Note: MSK does not seem to work too well at least with the default radio options.

### Test routines (-t)
 <pre><code>
Value: Scheme:
0      No test (KISS virtual TNC)
1      Simple Tx with polling. Packet smaller than 64 bytes
2      Simple Tx with packet interrupt handling. Packet up to 255 bytes
3      Simple Rx with polling. Packet smaller than 64 bytes
4      Simple Rx with packet interrupt handling. Packet up to 255 bytes
5      Simple echo test starting with Tx
6      Simple echo test starting with Rx
</code></pre>

# AX.25/KISS operation
## Set up the AX.25/KISS environment
### Kernel modules
You will need to activate the proper options in the `make menuconfig` of your kernel compilation in order to get the `ax25` and `mkiss` modules. It comes by default in most pre-compiled kernels.

Load the modules with `modprobe` command:
  - `sudo modprobe ax25`
  - `sudo modprobe mkiss`

Alternatively you can specify these modules to be loaded at boot time by adding their names in the `/etc/modules` file

### Install AX.25 and KISS software
  - `sudo apt-get install ax25-apps ax25-node ax25-tools libax25`

</code></pre>

### Create a virtual serial link
 - `socat d -d pty,link=/var/ax25/axp1,raw,echo=0 pty,link=/var/ax25/axp2,raw,echo=0 &`

Note the `&` at the end that allows the command to run in background.

This creates two serial devices at the end of a virtual serial cable. 
They are accessible via the symlinks specified in the command:
  - /var/ax25/axp1
  - /var/ax25/axp2

AX.25/KISS engine will be attached to the `axp1` end and the program to `axp2`.

### Create the network device using kissattach
  - `sudo kissattach /var/ax25/axp1 radio0 10.0.1.7`
  - `sudo ifconfig ax0 netmask 255.255.255.0`

This will create the `ax0` network device as shown by the `/sbin/ifconfig` command:
 <pre><code>
ax0       Link encap:AMPR AX.25  HWaddr F4EXB-15  
          inet addr:10.0.1.7  Bcast:10.0.1.255  Mask:255.255.255.0
          UP BROADCAST RUNNING  MTU:224  Metric:1
          RX packets:3033 errors:24 dropped:0 overruns:0 frame:0
          TX packets:3427 errors:0 dropped:0 overruns:0 carrier:0
          collisions:0 txqueuelen:10 
          RX bytes:483956 (472.6 KiB)  TX bytes:446797 (436.3 KiB)
</code></pre>

### Scripts that will run these commands
In the `scripts` directory you will find:
  - `kissdown.sh`: kills all processes and removes the `ax0` network interface from the system
  - `kissup.sh <IP> <Netmask>`: brings up the `ax0` network interface with IP address <IP> and net mask <Netmask>

Examples:
  - `./kissdown.sh`
  - `./kissup.sh 10.0.1.3 255.255.255.0`

## Run the program
This example will set the CC1101 at 9600 Baud with GFSK modulation. We raise the priority of the process (lower the priority number down to 0) with the `nice` command:

  - `sudo nice -n -20 ./picc1101 -v1 -B 9600 -P 252 -R7 -M4 -W -l15`

Other options are:
  - verbosity level (-v) of 1 will only display basic execution messages, errors and warnings
  - radio block size (-P) is fixed at 252 bytes
  - data whitening is in use (-W)
  - inter-block pause when sending multiple blocks (see next) is set for a 15 bytes transmission time approximately (-l) 

Note that you have to be super user to execute the program.

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

## Mitigate AX.25/KISS spurious packet retransmissions
In the latest versions an effort has been made to try to mitigate unnecessary packet retransmissions. These are generally caused by fragmenting packet chains too early. In return the ACK from the other end is received too early and synchronization is broken. Because of its robust handshake mechanism TCP/IP eventually recovers but some time is wasted.

To mitigate this effect when a packet is received on the serial link if another packet is received before some delay expires it is concatenated to the previous packet(s). The packets are sent over the air after this delay or if a radio packet has been received. This delay is called the TNC serial window.

The same mechanism exists on the radio side to possibly concatenate radio packets before they are sent on the serial line. The corresponding delay is called the TNC radio window.

These delays can be entered on the command line with the following long options with arguments in microseconds. The defaults have proved satisfactory on a 9600 Baud 2-FSK with 252 byte packets transmission. You may want to play with them or tweak them for different transmission characteristics:
  - `--tnc-serial-window`: defaults to 40ms. 
  - `--tnc-radio-window`: defaults to 0 that is no delay. Once the packet is received it will be immediately transfered to the serial link. At 9600 Baud 2-FSK with 250 byte packets the transmission time is already 208ms.
  
