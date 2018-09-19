MD := mkdir
RM := rm

EXTRA_CFLAGS := -DMAX_VERBOSE_LEVEL=4

all: prepare rfreceiver 

clean:
	rm -rf out rfreceiver
	 
prepare:
	$(MD) -p out

rfreceiver: main.o serial.o pi_cc_spi.o radio.o server.o util.o test.o register.o spaxstack.o
	cd out; $(CCPREFIX)gcc $(LDFLAGS) -s -lm -lwiringPi -o out/rfreceiver main.o serial.o pi_cc_spi.o radio.o server.o util.o test.o spaxstack.o

main.o: lib/radio/params.h main.cpp
	$(CCPREFIX)gcc $(CFLAGS) $(EXTRA_CFLAGS) -c -o out/main.o main.cpp

register.o: lib/spaxstack/register.h lib/spaxstack/register.cpp
	$(CCPREFIX)gcc $(CFLAGS) $(EXTRA_CFLAGS) -c -o out/register.o lib/spaxstack/register.cpp

#spaxstack.o: lib/spaxstack/spaxstack.h lib/spaxstack/spaxstack.cpp
#	$(CCPREFIX)gcc $(CFLAGS) $(EXTRA_CFLAGS) -c -o out/spaxstack.o lib/spaxstack/spaxstack.cpp

serial.o: lib/radio/params.h serial.h serial.c
	$(CCPREFIX)gcc $(CFLAGS) $(EXTRA_CFLAGS) -c -o out/serial.o serial.c

pi_cc_spi.o: lib/radio/params.h lib/radio/pi_cc_spi.h lib/radio/pi_cc_spi.cpp
	$(CCPREFIX)gcc $(CFLAGS) $(EXTRA_CFLAGS) -c -o out/pi_cc_spi.o lib/radio/pi_cc_spi.cpp

radio.o: lib/radio/params.h lib/radio/radio.h lib/radio/radio.cpp
	$(CCPREFIX)gcc $(CFLAGS) $(EXTRA_CFLAGS) -c -o out/radio.o lib/radio/radio.cpp

server.o: lib/radio/params.h server.h server.cpp
	$(CCPREFIX)gcc $(CFLAGS) $(EXTRA_CFLAGS) -c -o out/server.o server.cpp

test.o: test.h test.cpp
	$(CCPREFIX)gcc $(CFLAGS) $(EXTRA_CFLAGS) -c -o out/test.o test.cpp

util.o: util.h util.c
	$(CCPREFIX)gcc $(CFLAGS) $(EXTRA_CFLAGS) -c -o out/util.o util.c
