ECHO := echo
MD := mkdir
RM := rm -f


EXTRA_CFLAGS := -DMAX_VERBOSE_LEVEL=4 -MMD


SRC :=  main.cpp lib/inih/ini.c mqtt.cpp lib/spaxstack/register.cpp lib/spaxstack/spaxstack.cpp \
 lib/spaxstack/swstatus.cpp lib/spaxstack/swpacket.cpp lib/spaxstack/ccpacket.cpp lib/radio/pi_cc_spi.cpp lib/radio/radio.cpp \
 server.cpp test.cpp util.c lib/inih/inireader.cpp 

TARGET_DIR := out
MOCK_FILES := $(wildcard mocks/*.cpp)
MOCK_FILES += $(wildcard mocks/*.c)
MOCK_OBJ := $(MOCK_FILES:.c=.o) 
MOCK_OBJ := $(MOCK_OBJ:.cpp=.o) 

VPATH = $(dir $(SRC)) 
 
mock: EXTRA_CFLAGS+=-D_MOCKED
mock: SRC := $(patsubst lib/radio/pi_cc_spi.cpp,,$(SRC)) 
mock: SRC += $(MOCK_FILES)
mock: VPATH = $(dir $(MOCK_FILES)) $(dir $(SRC)) 

FNAMES = $(notdir $(SRC)) 

o1 = $(patsubst %.cpp,%.o,$(FNAMES)) 
objnames = $(patsubst %.c,%.o,$(o1)) 
objects = $(addprefix $(TARGET_DIR)/,$(objnames))

list_objects:
	@echo $(FNAMES)
	@echo $(objects)

#OBJ_DIR := out

#OBJ := $(patsubst $(SRC)/%.cpp,$(OBJ_DIR)/%.o,$(SRC))
#OBJ += $(patsubst $(SRC_DIR)/%.c,$(OBJ_DIR)/%.o,$(SRC))
list_obj: 
	@echo $(OBJ) 

OBJ_C := $(SRC:.c=.o) 
OBJ := $(OBJ_C:.cpp=.o)

DEP = $(objects:.o=.d)
#OBJ += $(SRC:.cpp=.o) 
CC := g++

.PHONY: prepare list_objects gccversion clean

clean:
	$(RM)r $(TARGET_DIR)
	$(RM) spaxxserver 
	 
prepare:
	$(MD) -p $(TARGET_DIR)

build: $(SRC) $(objects)  

build_mock: gccversion prepare $(SRC) $(MOCK_SRC) $(objects) 

link:
	$(CC) $(LDFLAGS) $(EXTRA_CFLAGS) -s -o spaxxserver $(objects) -lm -lmosquitto -lrt -lpthread -lwiringpi .lwiringpiDev

link_with_mock:
	$(CC) $(LDFLAGS) $(EXTRA_CFLAGS) -s -o spaxxserver $(objects) -lm -lmosquitto -lrt -lpthread

gccversion: 
	$(CC) --version
	
$(TARGET_DIR)/%.o : %.c
	echo "Compiling $< with extra flags $(EXTRA_CFLAGS)" 
	$(CC) -c $(CFLAGS) $(EXTRA_CFLAGS) -c $< -o $@ 

$(TARGET_DIR)/%.o : %.cpp
	@echo "Compiling with extra flags $(EXTRA_CFLAGS)$<"
	$(CC) -c $(CFLAGS) $(EXTRA_CFLAGS) -c $< -o $@ 

mock: gccversion prepare build link_with_mock

all: gccversion prepare build link
