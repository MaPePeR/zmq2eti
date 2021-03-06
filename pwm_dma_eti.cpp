
//Inspired by https://github.com/Wallacoloo/Raspberry-Pi-DMA-Example/blob/master/dma-gpio.c
//and http://www.icrobotics.co.uk/wiki/index.php/Turning_the_Raspberry_Pi_Into_an_FM_Transmitter
//and https://github.com/hzeller/rpi-gpio-dma-demo/blob/master/gpio-dma-test.c

//  0 1
//0 0 X
//1 X X

//Option 1:
//  0 1
//0 0 +
//1 ? -

//Option 2:
//  0 1
//0 0 -
//1 ? +

//Option 2:
//  0 1
//0 0 +
//1 - ?
#include <sys/time.h>
//#include <sys/mman.h> //for mmap
#include <stdio.h>
#include <signal.h> //for sigaction
#include <stdlib.h> //for exit, valloc
#include <unistd.h> //for NULL, lseek
#include <fcntl.h> //for file opening
#include <string.h> //for memset
#include <errno.h> //for errno
#include <pthread.h> //for pthread_setschedparam
#include <assert.h>
#include <zmq.h>
#include <deque>

#define RPI_V2
#include "hw-addresses.h"
#include "pwm_dma_eti_sys.h"
#include "odr-dabOutput.h"

#include "hdb3encodechain.hpp"
#include "uncached_memblock.h"
#include "zmqreader.h"
/************/
/* Settings */
/************/

#define SCHED_PRIORITY 30 //Linux scheduler priority. Higher = more realtime

#define DMA_CHANNEL (6)
#define CLOCK_DIVI (122)
#define CLOCK_DIVF (288)

#define STATUS_GPIO (27)


//map a physical address into our virtual address space. memfd is the file descriptor for /dev/mem
volatile uint32_t* mapPeripheral(int memfd, int addr) {
	///dev/mem behaves as a file. We need to map that file into memory:
	//NULL = virtual address of mapping is chosen by kernel.
	//PAGE_SIZE = map 1 page.
	//PROT_READ|PROT_WRITE means give us read and write priveliges to the memory
	//MAP_SHARED means updates to the mapped memory should be written back to the file & shared with other processes
	//memfd = /dev/mem file descriptor
	//addr = offset in file to map
	void *mapped = mmap(NULL, PAGE_SIZE, PROT_READ|PROT_WRITE, MAP_SHARED, memfd, addr);
	//now, *mapped = memory at physical address of addr.
	if (mapped == MAP_FAILED) {
		printf("failed to map memory (did you remember to run as root?)\n");
		exit(1);
	} else {
		printf("mapped: %p\n", mapped);
	}
	return (volatile uint32_t*)mapped;
}


volatile uint32_t *gpio;
struct DmaChannelHeader *dmaHeader; //must be global for cleanup()
struct PwmHeader *pwmHeader;
struct ClockManagerHeader *cmHeader;

void setSchedPriority(int priority) {
	//In order to get the best timing at a decent queue size, we want the kernel to avoid interrupting us for long durations.
	//This is done by giving our process a high priority. Note, must run as super-user for this to work.
	struct sched_param sp; 
	sp.sched_priority=priority; 
	int ret;
	if ((ret = pthread_setschedparam(pthread_self(), SCHED_FIFO, &sp))) {
		printf("Warning: pthread_setschedparam (increase thread priority) returned non-zero: %i\n", ret);
	}
}

void writeBitmasked(volatile uint32_t *dest, uint32_t mask, uint32_t value) {
	//set bits designated by (mask) at the address (dest) to (value), without affecting the other bits
	//eg if x = 0b11001100
	//  writeBitmasked(&x, 0b00000110, 0b11110011),
	//  then x now = 0b11001110
	uint32_t cur = *dest;
	uint32_t new_dest = (cur & (~mask)) | (value & mask);
	*dest = new_dest;
	*dest = new_dest; //best to be safe when crossing memory boundaries
}

void logDmaChannelHeader(struct DmaChannelHeader *h) {
	#if 0
	printf("Dma Ch Header:\n CS: 0x%08x\n CONBLK_AD: 0x%08x\n TI: 0x%08x\n SOURCE_AD: 0x%08x\n DEST_AD: 0x%08x\n TXFR_LEN: %u\n STRIDE: 0x%08x\n NEXTCONBK: 0x%08x\n DEBUG: 0x%08x\n", h->CS, h->CONBLK_AD, h->TI, h->SOURCE_AD, h->DEST_AD, h->TXFR_LEN, h->STRIDE, h->NEXTCONBK, h->DEBUG);
	#else
	uint32_t CS = h->CS;
	printf("Dma Ch Header:\n CS: 0x%08x\n", CS);

	printf("   %1d RESET\n", (CS >> 31) & 1);
	printf("   %1d ABORT\n", (CS >> 30) & 1);
	printf("   %1d DISDEBUG\n", (CS >> 29) & 1);
	printf("   %1d WAIT_FOR_OUTSTANDING_WRITES\n", (CS >> 28) & 1);
	printf("   PANIC_PRIORITY: %d\n", (CS >> 20) & 0xF);
	printf("   PRIORITY: %d\n",  (CS >> 16) & 0xF);
	printf("   %1d ERROR\n", (CS >> 8) & 1);
	printf("   %1d WAITING_FOR_OUTSTANDING_WRITES\n", (CS >> 6) & 1);
	printf("   %1d DREQ_STOPS_DMA\n", (CS >> 5) & 1);
	printf("   %1d PAUSED\n", (CS >> 4) & 1);
	printf("   %1d DREQ\n", (CS >> 3) & 1);
	printf("   %1d ACTIVE\n", (CS >> 0) & 1);

	printf(" CONBLK_AD: 0x%08x\n TI: 0x%08x\n SOURCE_AD: 0x%08x\n DEST_AD: 0x%08x\n TXFR_LEN: %u\n STRIDE: 0x%08x\n NEXTCONBK: 0x%08x\n", h->CONBLK_AD, h->TI, h->SOURCE_AD, h->DEST_AD, h->TXFR_LEN, h->STRIDE, h->NEXTCONBK);
	uint32_t DEBUG = h->DEBUG;
	printf(" DEBUG: 0x%08x\n", DEBUG);
	printf("   %1d LITE\n", (DEBUG >> 28) & 1);
	printf("   VERSION: %d\n",  (DEBUG >> 25) & 0x7);
	printf("   DMA_STATE: %d\n",  (DEBUG >> 16) & 0x1FF);
	printf("   DMA_ID: %d\n",  (DEBUG >> 8) & 0xFF);
	printf("   OUTSTANDING_WRITES: %d\n",  (DEBUG >> 4) & 0xF);
	printf("   %1d READ_ERROR\n", (DEBUG >> 2) & 1);
	printf("   %1d FIFO_ERROR\n", (DEBUG >> 1) & 1);
	printf("   %1d READ_LAST_NOT_SET_ERROR\n", (DEBUG >> 0) & 1);
	#endif
}

void logDmaControlBlock(struct DmaControlBlock *b) {
	printf("Dma Control Block:\n TI: 0x%08x\n SOURCE_AD: 0x%08x\n DEST_AD: 0x%08x\n TXFR_LEN: 0x%08x\n STRIDE: 0x%08x\n NEXTCONBK: 0x%08x\n unused: 0x%08x %08x\n", b->TI, b->SOURCE_AD, b->DEST_AD, b->TXFR_LEN, b->STRIDE, b->NEXTCONBK, b->_reserved[0], b->_reserved[1]);
}

volatile int stop_program = 0;

void *zmq_ctx;

ZMQReader *zmqreader;

void logPWMState(uint32_t pwmState) {
	printf("PWM-STA:");
	if (pwmState & (1 <<  0)) printf(" FULL1");
	if (pwmState & (1 <<  1)) printf(" EMPT1");
	if (pwmState & (1 <<  2)) printf(" WERR1");
	if (pwmState & (1 <<  3)) printf(" RERR1");
	if (pwmState & (1 <<  4)) printf(" GAPO1");
	if (pwmState & (1 <<  5)) printf(" GAPO2");
	if (pwmState & (1 <<  6)) printf(" GAPO3");
	if (pwmState & (1 <<  7)) printf(" GAPO4");
	if (pwmState & (1 <<  8)) printf(" BERR");
	if (pwmState & (1 <<  9)) printf(" STA1");
	if (pwmState & (1 << 10)) printf(" STA2");
	if (pwmState & (1 << 11)) printf(" STA3");
	if (pwmState & (1 << 12)) printf(" STA4");
	printf("\n");
}

class EncodedHDB3WordConsumer {
protected:
	uint64_t frame_processed = 0;
	uint64_t frame_send = 0;
	uint64_t gap_counter = 0;
	int previous_cb;
	bool hadPaddingBefore = false;
	int current_frame = -1;
	int current_frame_index;
	static constexpr int BUFFER_COUNT = 40;
	static constexpr int ETI_FRAME_SIZE = 6144;

	int last_frame = 0;
	struct timeval start_time;
	//We encode the bits at double the frequency and have 2 channels, so each ETI-Frame is encoded to 4 times its size in bytes.
	//So each 2 bytes in the eti frame will be encoded to 2 uint32_t
	static constexpr size_t BUFFER_BYTES_PER_FRAME = ETI_FRAME_SIZE * 2 * 2;
	static constexpr size_t BUFFER_WORDS_PER_FRAME = ETI_FRAME_SIZE;

	UncachedMemBlock srcPage;
	uint32_t *srcArray;
	UncachedMemBlock cbPage;
	struct DmaControlBlock *cbArr;
	struct DmaControlBlock *cbArrPadding;


	struct ClockManagerClockHeader *pwmClock;
	void setupClock() {
		pwmClock = &cmHeader->PWM;

		printf("setting up clock for pwm\n");
		//Setup Clock for PWM
		pwmClock->CTL = CM_CTL_PASSWD | ((pwmClock->CTL)&(~CM_CTL_ENAB)); //disable clock
		do {} while (pwmClock->CTL & CM_CTL_BUSY); //wait for clock to deactivate
		pwmClock->DIV = CM_DIV_PASSWD | CM_DIV_DIVI(CLOCK_DIVI) | CM_DIV_DIVF(CLOCK_DIVF); //configure clock divider (running at 500MHz undivided)
		pwmClock->CTL = CM_CTL_PASSWD | CM_CTL_SRC_PLLD | CM_CTL_MASH(1); //source 500MHz base clock, MASH1.
		pwmClock->CTL = CM_CTL_PASSWD | CM_CTL_SRC_PLLD | CM_CTL_ENAB | CM_CTL_MASH(1); //enable clock
		
		do {} while ((pwmClock->CTL & CM_CTL_BUSY) == 0); //wait for clock to activate
	}
	void setupPWM() {
		printf("Shutting down PWM\n");

		pwmHeader->DMAC = 0; //disable DMA
		pwmHeader->CTL = PWM_CTL_CLRFIFO; //clear pwm
		usleep(100);
		while(pwmHeader->STA & (0x3<<9)) { printf("PWM still running..."); usleep(100);}
		
		pwmHeader->STA = PWM_STA_ERRS; //clear PWM errors
		usleep(100);
		printf("Setting up PWM\n");
		
		pwmHeader->DMAC = PWM_DMAC_EN | PWM_DMAC_DREQ(4) | PWM_DMAC_PANIC(4); //DREQ is activated at queue < PWM_FIFO_SIZE
		pwmHeader->RNG1 = 32; 
		pwmHeader->RNG2 = 32; 
		pwmHeader->CTL =  PWM_CTL_ENABLE1 | PWM_CTL_USEFIFO1 | PWM_CTL_SERIAL1 |
						PWM_CTL_ENABLE2 | PWM_CTL_USEFIFO2 | PWM_CTL_SERIAL2 ;

		printf("Setting up GPIO18 + 13 for PWM0 and PWM1\n");
		SET_GPIO_ALT(18,5); //GPIO18 = PIN12
		SET_GPIO_ALT(13,0); //GPIO13 = PIN33
	}
	void setupDMA() {
		srcArray = (uint32_t*)srcPage.mem;
		memset(srcPage.mem, 0, srcPage.size); 

		cbArr = (struct DmaControlBlock*)cbPage.mem;
		cbArrPadding = ((struct DmaControlBlock*)cbPage.mem) + BUFFER_COUNT;

		for (int i = 0; i < BUFFER_COUNT; i++) {
			cbArr[i].TI = DMA_CB_TI_PERMAP_PWM | DMA_CB_TI_DEST_DREQ | DMA_CB_TI_SRC_INC | DMA_CB_TI_WAIT_RESP;
			cbArr[i].SOURCE_AD = srcPage.to_physical(((uint8_t*)srcArray) + i * BUFFER_BYTES_PER_FRAME);
			cbArr[i].DEST_AD = PWM_BASE_BUS + PWM_FIF1; //write to the FIFO
			cbArr[i].TXFR_LEN = BUFFER_BYTES_PER_FRAME;
			cbArr[i].STRIDE = 0;
			cbArr[i].NEXTCONBK = cbPage.to_physical(cbArr+((i+1) % BUFFER_COUNT));

			cbArrPadding[i].TI = DMA_CB_TI_PERMAP_PWM | DMA_CB_TI_DEST_DREQ | DMA_CB_TI_WAIT_RESP;
			//cbArr[i].SOURCE_AD will be filled in separately
			cbArrPadding[i].DEST_AD = PWM_BASE_BUS + PWM_FIF1;
			//cbArr[i].TXFR_LEN = BUFFER_BYTES_PER_FRAME;
			cbArr[i].STRIDE = 0;
			//cbArr[i].NEXTCONBK = cbPage.to_physical(cbArr+((i+1) % BUFFER_COUNT));
		}

		printf("Previous DMA header:\n");
		logDmaChannelHeader(dmaHeader);

		printf("Stopping DMA\n");
		dmaHeader->NEXTCONBK = 0;
		dmaHeader->CS |= DMA_CS_ABORT; //make sure to disable dma first.
		usleep(100); //give time for the abort command to be handled.

		dmaHeader->CS = DMA_CS_RESET;
		usleep(1100);

		writeBitmasked(&dmaHeader->CS, DMA_CS_END, DMA_CS_END); //clear the end flag
		usleep(100);

		dmaHeader->DEBUG = DMA_DEBUG_READ_ERROR | DMA_DEBUG_FIFO_ERROR | DMA_DEBUG_READ_LAST_NOT_SET_ERROR; // clear debug error flags
		uint32_t firstAddr = cbPage.to_physical(cbArr);
		printf("starting DMA @ CONBLK_AD=0x%08x\n", firstAddr);
		dmaHeader->CONBLK_AD = firstAddr; //(uint32_t)physCbPage + ((void*)cbArr - virtCbPage); //we have to point it to the PHYSICAL address of the control block (cb1)
		dmaHeader->CS = DMA_CS_PRIORITY(7) | DMA_CS_PANIC_PRIORITY(15) | DMA_CS_WAIT_FOR_OUTSTANDING_WRITES; //high priority (max is 7)
		dmaHeader->CS = DMA_CS_PRIORITY(7) | DMA_CS_PANIC_PRIORITY(15) | DMA_CS_WAIT_FOR_OUTSTANDING_WRITES | DMA_CS_ACTIVE; //activate DMA. 
	}
	inline void advanceCommandBlock(bool didPadding) {
		if(hadPaddingBefore) {
			assert(cbArrPadding[previous_cb].NEXTCONBK == 0);
			cbArrPadding[previous_cb].NEXTCONBK = cbPage.to_physical(&cbArr[current_frame]);;
		} else {
			assert(cbArr[previous_cb].NEXTCONBK == 0);
			cbArr[previous_cb].NEXTCONBK = cbPage.to_physical(&cbArr[current_frame]);;
		}
		previous_cb = current_frame;
		current_frame = (current_frame + 1) % BUFFER_COUNT;
		current_frame_index = 0;
		frame_processed += 1;
		hadPaddingBefore = didPadding;
	}
	inline void waitForCurrentFrameToEnd() {
		//usleep(dmaHeader->TXFR_LEN * 24 * 1000 / BUFFER_BYTES_PER_FRAME);
		usleep(24 * 1000 * 2);
	}
	inline int getCurrentDMAFrame() {
		uint32_t dma_pos = (dmaHeader->CONBLK_AD - cbPage.bus_addr) / sizeof(struct DmaControlBlock);
		assert(dma_pos < 2 * BUFFER_COUNT && dma_pos >= 0);
		return (dma_pos % BUFFER_COUNT);
	}
public:
	EncodedHDB3WordConsumer() : srcPage(BUFFER_COUNT * BUFFER_BYTES_PER_FRAME), cbPage(BUFFER_COUNT * 2 * sizeof(struct DmaControlBlock)) {
		setupClock();
		setupPWM();
		setupDMA();
	}
	void consumeEncodedHdb3(uint32_t out_p, uint32_t out_m) {
		static struct timeval last_info;
		if (current_frame < 0) {
			assert(!hadPaddingBefore);

			current_frame = getCurrentDMAFrame();
			printf("Starting... %d\n", current_frame);
			previous_cb = (BUFFER_COUNT + current_frame - 1) % BUFFER_COUNT;
			cbArr[previous_cb].NEXTCONBK = 0;
			waitForCurrentFrameToEnd();
			printf("Frame ended...\n");
			while(current_frame == getCurrentDMAFrame()) { printf("Need to wait a little longer %d\n", current_frame);}
			gettimeofday(&start_time, NULL);
			last_frame = getCurrentDMAFrame();
			current_frame_index = 0;
			cbArr[current_frame].NEXTCONBK = 0;
		} else if (current_frame == getCurrentDMAFrame()) {
			//Need to wait until we can write into this DMA-Block
			assert(current_frame_index == 0);
			zmqreader->tryReceive();
			waitForCurrentFrameToEnd();
			if (current_frame == getCurrentDMAFrame()) {
				logDmaChannelHeader(dmaHeader);
				printf("Need to wait a little longer2: %d\n", current_frame);
				while(current_frame == getCurrentDMAFrame()) {usleep(100);}
			}
			current_frame_index = 0;
			cbArr[current_frame].NEXTCONBK = 0;
		}
		int dma_frame = getCurrentDMAFrame();
		if (last_frame != dma_frame) {
			frame_send += (BUFFER_COUNT + dma_frame - last_frame) % BUFFER_COUNT;
			last_frame = dma_frame;
		}
		srcArray[current_frame * BUFFER_WORDS_PER_FRAME + current_frame_index] = out_p;
		srcArray[current_frame * BUFFER_WORDS_PER_FRAME + current_frame_index + 1] = out_m;
		current_frame_index += 2;
		if (current_frame_index >= 0 && (unsigned)current_frame_index >= BUFFER_WORDS_PER_FRAME) {
			cbArr[current_frame].NEXTCONBK = 0;
			//Full size frame without padding
			advanceCommandBlock(false);
		}
		struct timeval now;
		gettimeofday(&now, NULL);
		//time_t now = time(0);
		if ((pwmHeader->STA & (0xF0)) > 0) {
			//printf("GAP!!! %d\n", (pwmHeader->STA & (0xF0)) >> 4);
			pwmHeader->STA = 0xF0;
			gap_counter++;
			//last_gap = now;
		}

		if (last_info.tv_sec != now.tv_sec) {
			double diff = (now.tv_sec - start_time.tv_sec) + (now.tv_usec - start_time.tv_usec) / 1000000.0;
			last_info = now;
			printf("Runtime: %7lds E: %10.3lfB/s S: %10.3lf  gaps: %llu buffer: %dms zmq: %dx4\n", 
				(long)diff, 
				frame_processed * 6144.0/ (diff), 
				frame_send * 6144.0 / (diff),
				gap_counter,
				((BUFFER_COUNT + current_frame - getCurrentDMAFrame()) % BUFFER_COUNT ) * 24 + dmaHeader->TXFR_LEN * 24 / BUFFER_BYTES_PER_FRAME,
				zmqreader->bufferSize()
				);
			if (dmaHeader->CONBLK_AD == 0 || dmaHeader->NEXTCONBK == 0) {
				printf("MAX-GAP!\n");
				stop_program = 1;
			}
		}
	}

	void consumePadding(int repeats, uint32_t padding_out_p, uint32_t padding_out_m) {
		if (repeats >= 2) {
			assert(current_frame >= 0);
			//Each repeat of the padding byte is 2 Bytes.
			assert(6144 - current_frame_index == repeats * 2);
			assert(current_frame_index >= 0 && (unsigned)current_frame_index + 4 < BUFFER_WORDS_PER_FRAME);
			//Prepare for 128-bit-read.
			srcArray[current_frame * BUFFER_WORDS_PER_FRAME + current_frame_index] = padding_out_p;
			srcArray[current_frame * BUFFER_WORDS_PER_FRAME + current_frame_index + 1] = padding_out_m;
			srcArray[current_frame * BUFFER_WORDS_PER_FRAME + current_frame_index + 2] = padding_out_p;
			srcArray[current_frame * BUFFER_WORDS_PER_FRAME + current_frame_index + 3] = padding_out_m;
			cbArrPadding[current_frame].SOURCE_AD = 
				srcPage.to_physical( 
					((uint8_t*)srcArray) + current_frame * BUFFER_BYTES_PER_FRAME + sizeof(uint32_t) * current_frame_index
			);

			cbArrPadding[current_frame].NEXTCONBK = 0;
			//Previous-CB --> [Previous CB Padding] --> This CB --> This CB Padding
			cbArrPadding[current_frame].TXFR_LEN = repeats * sizeof(uint32_t) * 2;
			//This CB is not the full length
			cbArr[current_frame].TXFR_LEN = current_frame_index * sizeof(uint32_t);
			assert(cbArr[current_frame].TXFR_LEN + cbArrPadding[current_frame].TXFR_LEN == 6144 * 4);

			cbArr[current_frame].NEXTCONBK = cbPage.to_physical(&cbArrPadding[current_frame]);
			advanceCommandBlock(true);
		} else if (repeats == 1) {
			consumeEncodedHdb3(padding_out_p, padding_out_m);
		}
	}

	~EncodedHDB3WordConsumer() {
		for(int i = 0; i < BUFFER_COUNT; i++) {
			cbArr[i].NEXTCONBK = 0;
			cbArrPadding[i].NEXTCONBK = 0;
		}
		dmaHeader->NEXTCONBK = 0;
		dmaHeader->CS |= DMA_CS_ABORT;
		usleep(100); //give time for the abort command to be handled.

		dmaHeader->CS = DMA_CS_RESET;

		pwmHeader->CTL = 0;
		pwmClock->CTL = CM_CTL_PASSWD | ((pwmClock->CTL)&(~CM_CTL_ENAB)); //disable clock
		do {} while (pwmClock->CTL & CM_CTL_BUSY); //wait for clock to deactivate
	}
};

typedef ETIFrameConsumer<ETIFrameBitConsumer<HDB3TertiaryConsumer<EncodedHDB3WordConsumer>>> EncodingChain;


EncodingChain *chain;

void cleanup() {
	printf("Cleanup\n");
	stop_program = 1;
	GPIO_CLR = 1<<STATUS_GPIO;

	zmqreader->keep_running = false;
	delete zmqreader;

	if (zmq_ctx) {
		printf("Terminating ZMQ-Context\n");
		zmq_term(zmq_ctx);
	}
	delete chain;
	//disable DMA. Otherwise, it will continue to run in the background, potentially overwriting future user data.
	if(dmaHeader) {
		printf("Stopping DMA\n");
		writeBitmasked(&dmaHeader->CS, DMA_CS_ACTIVE, 0);
		usleep(100);
		writeBitmasked(&dmaHeader->CS, DMA_CS_RESET, DMA_CS_RESET);
	}
	printf("Stopping PWM\n");
	pwmHeader->CTL = 0;
}

void cleanupAndExit(int sig) {
	cleanup();
	printf("Exiting with error; caught signal: %i\n", sig);
	exit(1);
}


int main(int argc, const char *argv[]) {
	if (argc != 2) {
		fprintf(stderr, "Missing Argument.\nUsage: %s [ZMQ-Endpoint]\n", argv[0]);
		return 1;
	}
	//Setup peripherals
	volatile uint32_t *dmaBaseMem, *pwmBaseMem, *clockBaseMem;
	//emergency clean-up:
	for (int i = 0; i < 64; i++) { //catch all signals (like ctrl+c, ctrl+z, ...) to ensure DMA is disabled
		struct sigaction sa;
		memset(&sa, 0, sizeof(sa));
		sa.sa_handler = cleanupAndExit;
		sigaction(i, &sa, NULL);
	}
	setSchedPriority(SCHED_PRIORITY);
	assert(argc == 2);

	zmq_ctx = zmq_init(1);
	if (!zmq_ctx) {
		fprintf(stderr, "%s\n", "Error initializing zmq_ctx.");
		exit(1);
	}

	//First, open the linux device, /dev/mem
	//dev/mem provides access to the physical memory of the entire processor+ram
	//This is needed because Linux uses virtual memory, thus the process's memory at 0x00000000 will NOT have the same contents as the physical memory at 0x00000000
	int memfd = open("/dev/mem", O_RDWR | O_SYNC);
	if (memfd < 0) {
		printf("Failed to open /dev/mem (did you remember to run as root?)\n");
		exit(1);
	}
	//now map /dev/mem into memory, but only map specific peripheral sections:
	gpio = mapPeripheral(memfd, GPIO_BASE);
	dmaBaseMem = mapPeripheral(memfd, DMA_BASE);
	dmaHeader = (struct DmaChannelHeader*)(dmaBaseMem + DMACH(DMA_CHANNEL)/4); //must divide by 4, as dmaBaseMem is uint32_t*
	pwmBaseMem = mapPeripheral(memfd, PWM_BASE);
	pwmHeader = (struct PwmHeader*)(pwmBaseMem);
	//timerBaseMem = mapPeripheral(memfd, TIMER_BASE);
	clockBaseMem = mapPeripheral(memfd, CLOCK_BASE);
	cmHeader = (struct ClockManagerHeader*)(clockBaseMem + 0x70/4);

	zmqreader = new ZMQReader(zmq_ctx, argv[1]);
	chain = new EncodingChain();

	INP_GPIO(STATUS_GPIO);
	OUT_GPIO(STATUS_GPIO);
	GPIO_CLR = 1<<STATUS_GPIO;

	int frame_count = 0;
	bool blink_status = 0;
	
	zmq_dab_message_t *dab_msg = zmqreader->getNextMessage();
	while(!stop_program) {
		int frame_offset = 0;
		for (int frame = 0; frame < NUM_FRAMES_PER_ZMQ_MESSAGE; frame++) {
			chain->consumeETIFrame(dab_msg->buf + frame_offset, dab_msg->buflen[frame]);
			frame_offset += dab_msg->buflen[frame];
			frame_count += 1;
			if (frame_count >= 42) {
				frame_count = 0;
				if (blink_status) {
					GPIO_CLR = 1<<STATUS_GPIO;
					blink_status = false;
				} else {
					GPIO_SET = 1<<STATUS_GPIO;
					blink_status = true;
				}
			}
		}
		dab_msg = zmqreader->getNextMessage();

	}

	GPIO_CLR = 1<<STATUS_GPIO;


	printf("Cleanup...\n");
	pwmHeader->CTL = 0;

	cleanup();

	return 0;
}