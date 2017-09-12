all: pwm_dma_eti

pwm_dma_eti: pwm_dma_eti.o mailbox.o
	$(CC) pwm_dma_eti.o mailbox.o -o pwm_dma_eti