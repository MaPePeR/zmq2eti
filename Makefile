all: pwm_dma_eti hdb3_test

pwm_dma_eti: mailbox.o
CFLAGS+=$(shell pkg-config —cflags libzmq)
LIBS+=$(shell pkg-config —libs libzmq)