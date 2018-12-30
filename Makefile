.DEFAULT_GOAL := all
ECHO := echo
MD := mkdir
RM := rm -f


EXTRA_CFLAGS :=  -std=c++11 -DMAX_VERBOSE_LEVEL=4 -MMD


SRC :=  main.cpp lib/inih/ini.c mqtt.cpp  $(wildcard lib/spaxstack/*.cpp)  lib/radio/pi_cc_spi.cpp lib/radio/radio.cpp \
 server.cpp test.cpp util.c lib/inih/inireader.cpp 

TARGET_DIR := out
VPATH = $(dir $(SRC)) 
MOCK_FILES := $(wildcard mocks/*.cpp)
MOCK_FILES += $(wildcard mocks/*.c)
MOCK_OBJ := $(patsubst %.c,%.o, $(notdir $(MOCK_FILES))) 
MOCK_OBJ := $(MOCK_OBJ:.cpp=.o) 
MOCK_OBJ := $(addprefix $(TARGET_DIR)/, $(MOCK_OBJ))

FNAMES = $(notdir $(SRC)) 

o1 = $(patsubst %.cpp,%.o,$(FNAMES)) 
objnames = $(patsubst %.c,%.o,$(o1)) 

OBJ = $(addprefix $(TARGET_DIR)/,$(objnames))

list_objects:
	@echo $(FNAMES)
	@echo $(OBJ)

list_obj: 
	@echo $(OBJ) 

DEP = $(OBJ:.o=.d)

CC := g++ -g3

.PHONY: prepare list_objects prepare_mock gccversion clean 

clean:
	$(RM)r $(TARGET_DIR)
	$(RM) spaxxserver 
	 
prepare:
	$(MD) -p $(TARGET_DIR)

VPATH = $(dir $(MOCK_FILES)) $(dir $(SRC))
build: $(SRC) $(OBJ) 

link:
	$(CC) $(LDFLAGS) $(EXTRA_CFLAGS)  -o spaxxserver $(OBJ) $(LIB_LIST)

link_mock: 
	$(CC) $(LDFLAGS) $(EXTRA_CFLAGS)  -o spaxxserver  $(sort $(OBJ) $(MOCK_OBJ)) $(LIB_LIST)

build_mock: SRC := $(patsubst lib/radio/pi_cc_spi.cpp,,$(SRC)) #remove orig pic_cc_spi, since it is mocked
build_mock: OBJ := $(patsubst $(TARGET_DIR)/pi_cc_spi.o,,$(OBJ))

LIB_LIST = -lm -lmosquitto -lrt -lpthread -lwiringPi -lwiringPiDev
mock: VPATH = $(dir $(MOCK_FILES)) $(dir $(SRC)) 
mock: LIB_LIST = -lm -lmosquitto -lrt -lpthread

build_mock: 
	@echo Source files: $(SRC)
	@echo Source obj: $(OBJ)
	@echo Mock files: $(MOCK_FILES)
	@echo Mock obj: $(MOCK_OBJ)
build_mock: $(MOCK_FILES) $(MOCK_OBJ)  $(SRC) $(OBJ) 
#build_mock:
#	$(CC) $(LDFLAGS) $(EXTRA_CFLAGS) -s -o spaxxserver $(OBJ) $(MOCK_OBJ) -lm -lmosquitto -lrt -lpthread

mock: EXTRA_CFLAGS := $(EXTRA_CFLAGS) -D_MOCKED
#mock: $(SRC) += $(MOCK_FILES) 	
#mock: $(OBJ) += $(MOCK_OBJ) 

gccversion: 
	$(CC) --version
	
$(TARGET_DIR)/%.o : %.c
	echo "Compiling $< with extra flags $(EXTRA_CFLAGS) " 
	$(CC) -c $(CFLAGS) $(EXTRA_CFLAGS) -c $< -o $@ 

$(TARGET_DIR)/%.o : %.cpp
	@echo "Compiling $< with extra flags $(EXTRA_CFLAGS)"
	$(CC) -c $(CFLAGS) $(EXTRA_CFLAGS) -c $< -o $@ 

mock: gccversion prepare prepare_mock build_mock link_mock

all: gccversion prepare build link
