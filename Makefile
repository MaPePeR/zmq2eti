all: pwm_dma_eti hdb3_test zmq_speedtest test_dma_write_multiple hdb3_test2

pwm_dma_eti: mailbox.o hdb3encodechain.hpp uncached_memblock.h
CFLAGS+=$(shell pkg-config --cflags libzmq)
LDLIBS+=$(shell pkg-config --libs libzmq) -lpthread
CFLAGS += -Wall -O0 -g
CXXFLAGS += -Wall -O0 -g
test_dma_write_multiple: mailbox.o

hdb3_test2: hdb3encodechain.hpp