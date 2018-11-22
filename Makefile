ECHO := echo
MD := mkdir
RM := rm

EXTRA_CFLAGS := -DMAX_VERBOSE_LEVEL=4

all: prepare spaxxserver 

mock: EXTRA_CFLAGS+=-D_MOCKED
mock: prepare spaxxserver_mock

SRC =  main.cpp lib/spaxstack/register.cpp lib/spaxstack/spaxstack.cpp lib/spaxstack/ccpacket.cpp lib/radio/pi_cc_spi.cpp lib/radio/radio.cpp server.cpp test.cpp util.c lib/inih/inireader.cpp
OBJ = $(SRC:.c=.o) 
clean:
	rm -rf out 
	 
prepare:
	$(MD) -p out

spaxxserver_mock: main.o wiringpi_mock.o pi_cc_spi_mock.o radio.o server.o util.o swpacket.o ccpacket.o test.o register.o inireader.o spaxstack.o
	cd out; $(CCPREFIX)g++ $(LDFLAGS) -s -o spaxxserver swpacket.o ini.o util.o inireader.o main.o wiringpi.o pi_cc_spi.o radio.o server.o test.o spaxstack.o ccpacket.o \
	-lm -lmosquitto -lrt -lpthread
	cp out/spaxxserver .

spaxxserver: main.o pi_cc_spi.o radio.o server.o util.o test.o register.o ccpacket.o swpacket.o inireader.o spaxstack.o
	cd out; $(CCPREFIX)g++ $(LDFLAGS) -s -o spaxxserver swpacket.o ini.o inireader.o main.o pi_cc_spi.o radio.o server.o test.o spaxstack.o util.o ccpacket.o \
	-lm -lmosquitto -lrt -lwiringPi -lwiringPiDev 
	cp out/spaxxserver .

main.o: lib/radio/params.h main.cpp
	$(CCPREFIX)g++ $(CFLAGS) $(EXTRA_CFLAGS) -c -o out/main.o main.cpp

register.o: lib/spaxstack/register.h lib/spaxstack/register.cpp
	$(CCPREFIX)g++ $(CFLAGS) $(EXTRA_CFLAGS) -c -o out/register.o lib/spaxstack/register.cpp

spaxstack.o: lib/spaxstack/spaxstack.h lib/spaxstack/spaxstack.cpp
	$(CCPREFIX)g++ $(CFLAGS) $(EXTRA_CFLAGS) -c -o out/spaxstack.o lib/spaxstack/spaxstack.cpp

ccpacket.o: lib/spaxstack/ccpacket.h lib/spaxstack/ccpacket.cpp
	$(CCPREFIX)g++ $(CFLAGS) $(EXTRA_CFLAGS) -c -o out/ccpacket.o lib/spaxstack/ccpacket.cpp

swpacket.o: lib/spaxstack/swpacket.h lib/spaxstack/swpacket.cpp
	$(CCPREFIX)g++ $(CFLAGS) $(EXTRA_CFLAGS) -c -o out/swpacket.o lib/spaxstack/swpacket.cpp

pi_cc_spi.o: lib/radio/params.h lib/radio/pi_cc_spi.h lib/radio/pi_cc_spi.cpp
	$(CCPREFIX)g++ $(CFLAGS) $(EXTRA_CFLAGS) -c -o out/pi_cc_spi.o lib/radio/pi_cc_spi.cpp

radio.o: lib/radio/params.h lib/radio/radio.h lib/radio/radio.cpp
	$(CCPREFIX)g++ $(CFLAGS) $(EXTRA_CFLAGS) -c -o out/radio.o lib/radio/radio.cpp

server.o: lib/radio/params.h server.h server.cpp
	$(CCPREFIX)g++ $(CFLAGS) $(EXTRA_CFLAGS) -c -o out/server.o server.cpp

test.o: test.h test.cpp
	$(CCPREFIX)g++ $(CFLAGS) $(EXTRA_CFLAGS) -c -o out/test.o test.cpp

util.o: util.h util.c
	$(CCPREFIX)g++ $(CFLAGS) $(EXTRA_CFLAGS) -c -o out/util.o util.c

inireader.o: lib/inih/inireader.h lib/inih/inireader.cpp
	$(CCPREFIX)g++ $(CFLAGS) $(EXTRA_CFLAGS) -c -o out/inireader.o lib/inih/inireader.cpp
	$(CCPREFIX)g++ $(CFLAGS) $(EXTRA_CFLAGS) -c -o out/ini.o lib/inih/ini.c

###Mocks:
wiringpi_mock.o: mocks/wiringpi.c	
	$(CCPREFIX)g++ $(CFLAGS) $(EXTRA_CFLAGS) -c -o out/wiringpi.o mocks/wiringpi.c

pi_cc_spi_mock.o: lib/radio/params.h lib/radio/pi_cc_spi.h mocks/pi_cc_spi.cpp
	$(CCPREFIX)g++ $(CFLAGS) $(EXTRA_CFLAGS) -c -o out/pi_cc_spi.o mocks/pi_cc_spi.cpp

