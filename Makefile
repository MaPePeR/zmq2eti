all: pwm_dma_eti hdb3_test zmq_speedtest

pwm_dma_eti: mailbox.o
CFLAGS+=$(shell pkg-config --cflags libzmq)
LDLIBS+=$(shell pkg-config --libs libzmq) -lpthread
CFLAGS += -Wall -O3 -g
CXXFLAGS += -Wall -O3 -g