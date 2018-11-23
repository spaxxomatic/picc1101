ECHO := echo
MD := mkdir
RM := rm -f

EXTRA_CFLAGS := -DMAX_VERBOSE_LEVEL=4 -MMD

all: build 

SRC =  main.cpp lib/inih/ini.c mqtt.cpp lib/spaxstack/register.cpp lib/spaxstack/spaxstack.cpp \
 lib/spaxstack/swstatus.cpp lib/spaxstack/swpacket.cpp lib/spaxstack/ccpacket.cpp lib/radio/pi_cc_spi.cpp lib/radio/radio.cpp \
 server.cpp test.cpp util.c lib/inih/inireader.cpp 

mock: EXTRA_CFLAGS+=-D_MOCKED
mock: MOCK_SRC = mocks/wiringpi.c

mock: list_obj build

#OBJ_DIR := out

#OBJ := $(patsubst $(SRC)/%.cpp,$(OBJ_DIR)/%.o,$(SRC))
#OBJ += $(patsubst $(SRC_DIR)/%.c,$(OBJ_DIR)/%.o,$(SRC))
list_obj: 
	@echo $(OBJ) 

OBJ_C = $(SRC:.c=.o) 
OBJ = $(OBJ_C:.cpp=.o)
DEP = $(OBJ:.o=.d)
#OBJ += $(SRC:.cpp=.o) 
CC := g++
clean:
	$(RM) $(OBJ) $(DEP)
	$(RM) spaxxserver 
	$(RM) $(MOCK_SRC:.c=.o)
	 
prepare:
	$(MD) -p out

build: gccversion prepare $(SRC) $(MOCK_SRC) $(OBJ) link

link:
	$(CC) $(LDFLAGS) $(EXTRA_CFLAGS) -s -o spaxxserver $(OBJ) -lm -lmosquitto -lrt -lpthread

gccversion : 
	$(CC) --version
	
%.o : %.c
	echo "Compiling $< with extra flags $(EXTRA_CFLAGS)" 
	$(CC) -c $(CFLAGS) $(EXTRA_CFLAGS) -c $< -o $@ 

%.o : %.cpp
	@echo "Compiling $<"
	$(CC) -c $(CFLAGS) $(EXTRA_CFLAGS) -c $< -o $@ 

#spaxxserver_mock: main.o mqtt.o wiringpi_mock.o pi_cc_spi_mock.o radio.o server.o util.o swpacket.o ccpacket.o test.o register.o inireader.o spaxstack.o
#	cd out; $(CC) $(LDFLAGS) -s -o spaxxserver swpacket.o mqtt.o ini.o util.o inireader.o main.o wiringpi.o pi_cc_spi.o radio.o server.o test.o spaxstack.o ccpacket.o \
#	-lm -lmosquitto -lrt -lpthread
#	cp out/spaxxserver .
