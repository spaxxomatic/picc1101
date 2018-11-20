ECHO := echo
MD := mkdir
RM := rm

EXTRA_CFLAGS := -DMAX_VERBOSE_LEVEL=4

all: prepare spaxxserver 

mock: EXTRA_CFLAGS+=-D_MOCKED
mock: prepare spaxxserver_mock

clean:
	rm -rf out rfreceiver
	 
prepare:
	$(MD) -p out; echo $(EXTRA_CFLAGS)

spaxxserver_mock: main.o wiringpi_mock.o pi_cc_spi.o radio.o server.o util.o test.o register.o spaxstack.o
	cd out; $(CCPREFIX)g++ $(LDFLAGS) -s -lm util.o main.o wiringpi_mock.o pi_cc_spi.o radio.o server.o test.o spaxstack.o -o spaxxserver 

spaxxserver: main.o pi_cc_spi.o radio.o server.o util.o test.o register.o spaxstack.o
	cd out; $(CCPREFIX)g++ $(LDFLAGS) -s -lm -lwiringPi -o spaxxserver main.o pi_cc_spi.o radio.o server.o test.o spaxstack.o util.o

main.o: lib/radio/params.h main.cpp
	$(CCPREFIX)g++ $(CFLAGS) $(EXTRA_CFLAGS) -c -o out/main.o main.cpp

register.o: lib/spaxstack/register.h lib/spaxstack/register.cpp
	$(CCPREFIX)g++ $(CFLAGS) $(EXTRA_CFLAGS) -c -o out/register.o lib/spaxstack/register.cpp

spaxstack.o: lib/spaxstack/spaxstack.h lib/spaxstack/spaxstack.cpp
	$(CCPREFIX)g++ $(CFLAGS) $(EXTRA_CFLAGS) -c -o out/spaxstack.o lib/spaxstack/spaxstack.cpp

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

###Mocks:
wiringpi_mock.o: mocks/wiringpi_mock.c	
	$(CCPREFIX)g++ $(CFLAGS) $(EXTRA_CFLAGS) -c -o out/wiringpi_mock.o mocks/wiringpi_mock.c
