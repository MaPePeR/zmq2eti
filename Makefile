all: pwm_dma_eti hdb3_test zmq_speedtest

pwm_dma_eti: mailbox.o
CFLAGS+=$(shell pkg-config --cflags libzmq)
LDLIBS+=$(shell pkg-config --libs libzmq)
CFLAGS += -Wall -O3
CXXFLAGS += -Wall -O3