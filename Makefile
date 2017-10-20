all: pwm_dma_eti

pwm_dma_eti: mailbox.o hdb3encodechain.hpp uncached_memblock.h zmqreader.h
CFLAGS+=$(shell pkg-config --cflags libzmq)
LDLIBS+=$(shell pkg-config --libs libzmq) -lpthread
CFLAGS += -Wall -O3 -g
CXXFLAGS += -Wall -O3 -g
