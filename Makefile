all: pwm_dma_eti hdb3_test

pwm_dma_eti: mailbox.o
CFLAGS+=$(shell pkg-config --cflags libzmq)
LDLIBS+=$(shell pkg-config --libs libzmq)
CFLAGS += -Wall
CXXFLAGS += -Wall