
ALT_DEVICE_FAMILY ?= soc_cv_av
SOCEDS_ROOT 	  ?= $(SOCEDS_DEST_ROOT)
HWLIBS_ROOT 	   = $(SOCEDS_ROOT)/ip/altera/hps/altera_hps/hwlib
CROSS_COMPILE 	   = arm-linux-gnueabihf-
CC 		   = $(CROSS_COMPILE)g++
ARCH		   = arm

CFLAGS = -g -Wall -std=c++11  -D$(ALT_DEVICE_FAMILY) \
	-I$(HWLIBS_ROOT)/include/$(ALT_DEVICE_FAMILY) \
	-I$(HWLIBS_ROOT)/include/ \
	-I./lib/
LDFLAGS =  -g -Wall -lpthread

TARGET = balance_car
SRC = $(wildcard  ./lib/*.cpp *.cpp)
all:
	$(CC) $(CFLAGS) $(LDFLAGS) $(SRC) -o $(TARGET)

clean:
	rm $(TARGET)