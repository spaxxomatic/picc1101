MD := mkdir
RM := rm

EXTRA_CFLAGS := -DMAX_VERBOSE_LEVEL=4

all: prepare picc1101 

clean:
	rm -f *.o picc1101
	 
prepare:
	$(MD) -p out

picc1101: main.o serial.o pi_cc_spi.o radio.o server.o util.o test.o register.o spaxstack.o
	cd out; $(CCPREFIX)gcc $(LDFLAGS) -s -lm -lwiringPi -o out/picc1101 main.o serial.o pi_cc_spi.o radio.o server.o util.o test.o spaxstack.o

main.o: main.h main.c
	$(CCPREFIX)gcc $(CFLAGS) $(EXTRA_CFLAGS) -c -o out/main.o main.c

register.o: lib/spaxstack/register.h lib/spaxstack/register.cpp
	$(CCPREFIX)gcc $(CFLAGS) $(EXTRA_CFLAGS) -c -o out/register.o lib/spaxstack/register.cpp

spaxstack.o: lib/spaxstack/spaxstack.h lib/spaxstack/spaxstack.cpp
	$(CCPREFIX)gcc $(CFLAGS) $(EXTRA_CFLAGS) -c -o out/spaxstack.o lib/spaxstack/spaxstack.cpp

serial.o: main.h serial.h serial.c
	$(CCPREFIX)gcc $(CFLAGS) $(EXTRA_CFLAGS) -c -o out/serial.o serial.c

pi_cc_spi.o: main.h pi_cc_spi.h pi_cc_spi.c
	$(CCPREFIX)gcc $(CFLAGS) $(EXTRA_CFLAGS) -c -o out/pi_cc_spi.o pi_cc_spi.c

radio.o: main.h radio.h radio.c
	$(CCPREFIX)gcc $(CFLAGS) $(EXTRA_CFLAGS) -c -o out/radio.o radio.c

server.o: main.h server.h server.c
	$(CCPREFIX)gcc $(CFLAGS) $(EXTRA_CFLAGS) -c -o out/server.o server.c

test.o: test.h test.c
	$(CCPREFIX)gcc $(CFLAGS) $(EXTRA_CFLAGS) -c -o out/test.o test.c

util.o: util.h util.c
	$(CCPREFIX)gcc $(CFLAGS) $(EXTRA_CFLAGS) -c -o out/util.o util.c
