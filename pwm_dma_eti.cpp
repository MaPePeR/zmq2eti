
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
#include <time.h>
#include <sys/mman.h> //for mmap
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
#include "hdb3.h"
#include "odr-dabOutput.h"
/************/
/* Settings */
/************/

#define SCHED_PRIORITY 30 //Linux scheduler priority. Higher = more realtime

#define DMA_CHANNEL (5)
#define CLOCK_DIVI (122)
#define CLOCK_DIVF (72)
#define BUFFER_SECONDS (15)
//We output 2 * 2048 Bits per second onto 2 channels at 32 bit per command
#define BUFFER_COMMANDS (BUFFER_SECONDS * 2048 * 2 * 2 / 32)
//One ETI-Frame lasts for 24*10**-3 Seconds, so this is approx the amount of ZNQ-Messages for a full buffer of BUFFER_SECONDS-seconds.
#define ZMQ_MESSAGE_BUFFER_SIZE (BUFFER_SECONDS * 1000 / (24 * NUM_FRAMES_PER_ZMQ_MESSAGE))

#define SLEEP_TIME (BUFFER_SECONDS / 3)


/*************************/
extern "C"
{
	#include "mailbox.h"
}
// ---- Memory mappping defines
#define BUS_TO_PHYS(x) ((x)&~0xC0000000)

// ---- Memory allocating defines
// https://github.com/raspberrypi/firmware/wiki/Mailbox-property-interface
#define MEM_FLAG_DIRECT           (1 << 2)
#define MEM_FLAG_COHERENT         (2 << 2)
#define MEM_FLAG_L1_NONALLOCATING (MEM_FLAG_DIRECT | MEM_FLAG_COHERENT)

// A memory block that represents memory that is allocated in physical
// memory and locked there so that it is not swapped out.
// It is not backed by any L1 or L2 cache, so writing to it will directly
// modify the physical memory (and it is slower of course to do so).
// This is memory needed with DMA applications so that we can write through
// with the CPU and have the DMA controller 'see' the data.
// The UncachedMemBlock_{alloc,free,to_physical}
// functions are meant to operate on these.
struct UncachedMemBlock {
  void *mem;                  // User visible value: the memory to use.
  //-- Internal representation.
  uint32_t bus_addr;
  uint32_t mem_handle;
  size_t size;
};

static int mbox_fd = -1;   // used internally by the UncachedMemBlock-functions.

// Allocate a block of memory of the given size (which is rounded up to the next
// full page). The memory will be aligned on a page boundary and zeroed out.
static struct UncachedMemBlock UncachedMemBlock_alloc(size_t size) {
  if (mbox_fd < 0) {
	mbox_fd = mbox_open();
	assert(mbox_fd >= 0);  // Uh, /dev/vcio not there ?
  }
  // Round up to next full page.
  size = size % PAGE_SIZE == 0 ? size : (size + PAGE_SIZE) & ~(PAGE_SIZE - 1);

  struct UncachedMemBlock result;
  result.size = size;
  result.mem_handle = mem_alloc(mbox_fd, size, PAGE_SIZE,
								MEM_FLAG_L1_NONALLOCATING);
  result.bus_addr = mem_lock(mbox_fd, result.mem_handle);
  result.mem = mapmem(BUS_TO_PHYS(result.bus_addr), size);
  fprintf(stderr, "Alloc: %6d bytes;  %p (bus=0x%08x, phys=0x%08x)\n",
		  (int)size, result.mem, result.bus_addr, BUS_TO_PHYS(result.bus_addr));
  assert(result.bus_addr);  // otherwise: couldn't allocate contiguous block.
  memset(result.mem, 0x00, size);

  return result;
}

// Free block previously allocated with UncachedMemBlock_alloc()
static void UncachedMemBlock_free(struct UncachedMemBlock *block) {
  if (block->mem == NULL) return;
  assert(mbox_fd >= 0);  // someone should've initialized that on allocate.
  unmapmem(block->mem, block->size);
  mem_unlock(mbox_fd, block->mem_handle);
  mem_free(mbox_fd, block->mem_handle);
  block->mem = NULL;
}


// Given a pointer to memory that is in the allocated block, return the
// physical bus addresse needed by DMA operations.
static uintptr_t UncachedMemBlock_to_physical(const struct UncachedMemBlock *blk,
											  void *p) {
	uint32_t offset = (uint8_t*)p - (uint8_t*)blk->mem;
	assert(offset < blk->size);   // pointer not within our block.
	return blk->bus_addr + offset;
}

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


struct DmaChannelHeader *dmaHeader; //must be global for cleanup()
struct PwmHeader *pwmHeader;

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

void *zmq_ctx;
void *zmq_sock;
zmq_dab_message_t zmq_msg_buffer[ZMQ_MESSAGE_BUFFER_SIZE];
int zmq_msg_buffer_count = 0;
int zmq_msg_buffer_pos = 0;
volatile int stop_zmq_message_thread = 0;
const char *zmq_endpoint;

std::deque<zmq_dab_message_t *> empty_buffers;
std::deque<zmq_dab_message_t *> filled_buffers;
pthread_t zmq_read_thread;
pthread_mutex_t zmq_buffer_mutex;
pthread_cond_t zmq_buffer_done_with_message;
pthread_cond_t zmq_buffer_new_message_recieved;

zmq_dab_message_t * get_next_read_buffer(bool holdingBuffer) {
    pthread_mutex_lock(&zmq_buffer_mutex);

    if (holdingBuffer) {
    	empty_buffers.push_back(filled_buffers.front());
    	filled_buffers.pop_front();
    	pthread_cond_signal(&zmq_buffer_done_with_message);
    }

    while (filled_buffers.empty()) {
        pthread_cond_wait(&zmq_buffer_new_message_recieved, &zmq_buffer_mutex);
    }
    zmq_dab_message_t * output = filled_buffers.front();
    pthread_mutex_unlock(&zmq_buffer_mutex);
    return output;
}

void *read_zmq_messages_thread(void *arg) {
	pthread_mutex_lock(&zmq_buffer_mutex);
	for (int i = 0; i < ZMQ_MESSAGE_BUFFER_SIZE; i++) {
		empty_buffers.push_back(&zmq_msg_buffer[i]);
	}
	pthread_mutex_unlock(&zmq_buffer_mutex);

	zmq_ctx = zmq_init(1);
	if (!zmq_ctx) {
		fprintf(stderr, "%s\n", "Error initializing zmq_ctx.");
		exit(1);
	}
	zmq_sock = zmq_socket(zmq_ctx, ZMQ_SUB);
	if (!zmq_sock) {
		fprintf(stderr, "%s\n", "Error creating zmq_socket.");
		exit(1);
	}
	if (zmq_connect(zmq_sock, zmq_endpoint) < 0) {
		fprintf(stderr, "Failed to connect to zmq-endpoint %s\n", zmq_endpoint);
		exit(1);
	}
	int rc = zmq_setsockopt (zmq_sock, ZMQ_SUBSCRIBE, "", 0);
	assert (rc == 0);

	pthread_mutex_lock(&zmq_buffer_mutex);
	for (int i = 0; i < ZMQ_MESSAGE_BUFFER_SIZE / 2 && !stop_zmq_message_thread; i++) {
		zmq_dab_message_t * output = empty_buffers.front();
		empty_buffers.pop_front();
		int rc = zmq_recv(zmq_sock, output, sizeof(*zmq_msg_buffer), 0);
		assert(rc > 0 && rc <= sizeof(*zmq_msg_buffer));
		filled_buffers.push_back(output);
	}
	pthread_mutex_unlock(&zmq_buffer_mutex);


	while(!stop_zmq_message_thread) {
		zmq_dab_message_t * output;
		pthread_mutex_lock(&zmq_buffer_mutex);
		while(empty_buffers.empty()) {
			pthread_cond_wait(&zmq_buffer_done_with_message, &zmq_buffer_mutex);
		}
		output = empty_buffers.front();
		pthread_mutex_unlock(&zmq_buffer_mutex);
		if (stop_zmq_message_thread) break;

		int rc = zmq_recv(zmq_sock, output, sizeof(*zmq_msg_buffer), 0);
		assert(rc > 0 && rc <= sizeof(*zmq_msg_buffer));

		pthread_mutex_lock(&zmq_buffer_mutex);
			zmq_dab_message_t * output2 = empty_buffers.front();
			empty_buffers.pop_front();
			assert(output == output2);
			filled_buffers.push_back(output);
			pthread_cond_signal(&zmq_buffer_new_message_recieved);
		pthread_mutex_unlock(&zmq_buffer_mutex);
	}
	if (zmq_sock) {
		printf("Closing ZMQ-Socket\n");
		zmq_close(zmq_sock);
	}
	if (zmq_ctx) {
		printf("Terminating ZMQ-Context\n");
		zmq_term(zmq_ctx);
	}
	return NULL;
}

void cleanup() {
	stop_zmq_message_thread = 1;
	printf("Cleanup\n");
	//disable DMA. Otherwise, it will continue to run in the background, potentially overwriting future user data.
	if(dmaHeader) {
		printf("Stopping DMA\n");
		writeBitmasked(&dmaHeader->CS, DMA_CS_ACTIVE, 0);
		usleep(100);
		writeBitmasked(&dmaHeader->CS, DMA_CS_RESET, DMA_CS_RESET);
	}
	printf("Stopping PWM\n");
	pwmHeader->CTL = 0;

	pthread_mutex_lock(&zmq_buffer_mutex);
	pthread_cond_signal(&zmq_buffer_done_with_message);
	pthread_mutex_unlock(&zmq_buffer_mutex);
	pthread_join(zmq_read_thread, NULL); // will wait forever
	//could also disable PWM, but that's not imperative.
}

void cleanupAndExit(int sig) {
	cleanup();
	printf("Exiting with error; caught signal: %i\n", sig);
	exit(1);
}


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

int main(int argc, const char *argv[]) {
	//Setup peripherals
	volatile uint32_t *gpio, *dmaBaseMem, *pwmBaseMem, *clockBaseMem;
	//emergency clean-up:
	for (int i = 0; i < 64; i++) { //catch all signals (like ctrl+c, ctrl+z, ...) to ensure DMA is disabled
		struct sigaction sa;
		memset(&sa, 0, sizeof(sa));
		sa.sa_handler = cleanupAndExit;
		sigaction(i, &sa, NULL);
	}
	setSchedPriority(SCHED_PRIORITY);
	assert(argc == 2);
	zmq_endpoint = argv[1];

	printf("Starting read thread...");
	pthread_create(&zmq_read_thread, NULL, read_zmq_messages_thread, NULL);
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

	printf("setting up clock for pwm\n");
	//Setup Clock for PWM
	*(clockBaseMem + CM_PWMCTL/4) = CM_PWMCTL_PASSWD | ((*(clockBaseMem + CM_PWMCTL/4))&(~CM_PWMCTL_ENAB)); //disable clock
	do {} while (*(clockBaseMem + CM_PWMCTL/4) & CM_PWMCTL_BUSY); //wait for clock to deactivate
	*(clockBaseMem + CM_PWMDIV/4) = CM_PWMDIV_PASSWD | CM_PWMDIV_DIVI(CLOCK_DIVI) | CM_PWMDIV_DIVF(CLOCK_DIVF); //configure clock divider (running at 500MHz undivided)
	*(clockBaseMem + CM_PWMCTL/4) = CM_PWMCTL_PASSWD | CM_PWMCTL_SRC_PLLD | CM_PWMCTL_MASH(1); //source 500MHz base clock, MASH1.
	*(clockBaseMem + CM_PWMCTL/4) = CM_PWMCTL_PASSWD | CM_PWMCTL_SRC_PLLD | CM_PWMCTL_ENAB | CM_PWMCTL_MASH(1); //enable clock
	do {} while ((*(clockBaseMem + CM_PWMCTL/4) & CM_PWMCTL_BUSY) == 0); //wait for clock to activate

	printf("setting up PWM\n");
	//Setup PWM
	pwmHeader->DMAC = 0; //disable DMA
	pwmHeader->CTL = PWM_CTL_CLRFIFO; //clear pwm
	usleep(100);
	
	pwmHeader->STA = PWM_STA_ERRS; //clear PWM errors
	usleep(100);
	
	pwmHeader->DMAC = PWM_DMAC_EN | PWM_DMAC_DREQ(4) | PWM_DMAC_PANIC(4); //DREQ is activated at queue < PWM_FIFO_SIZE
	pwmHeader->RNG1 = 32; 
	pwmHeader->RNG2 = 32; 
	pwmHeader->CTL =  PWM_CTL_ENABLE1 | PWM_CTL_USEFIFO1 | PWM_CTL_SERIAL1 |
					PWM_CTL_ENABLE2 | PWM_CTL_USEFIFO2 | PWM_CTL_SERIAL2 ;

	printf("Setting up GPIO12 + 13 for PWM0 and PWM1\n");
	//Setup Pins to output PWM:
	//INP_GPIO(18);
	//OUT_GPIO(18);
	SET_GPIO_ALT(18,5); //GPIO12 = PIN32
	//INP_GPIO(13);
	//OUT_GPIO(13);
	SET_GPIO_ALT(13,0); //GPIO13 = PIN33

	//Setup DMA
	printf("Allocating locked memory\n");
	usleep(1000);

	size_t srcPageBytes = sizeof(uint32_t) * BUFFER_COMMANDS;

	struct UncachedMemBlock srcPage = UncachedMemBlock_alloc(srcPageBytes);
	
	uint32_t *srcArray = (uint32_t*)srcPage.mem; 

	memset(srcPage.mem, 0, srcPageBytes); //Just make sure its 0ed.

	//allocate memory for the control blocks
	size_t cbPageBytes = BUFFER_COMMANDS * sizeof(struct DmaControlBlock); //3 cbs for each source block
	struct UncachedMemBlock cbPage = UncachedMemBlock_alloc(cbPageBytes);
	struct DmaControlBlock *cbArr = (struct DmaControlBlock*)cbPage.mem;

	printf("Memory:\n  Data-Buffer-Size: %10zd Bytes\n  Command-Buffer-Size: %10zd Bytes\n", srcPageBytes, cbPageBytes);

	printf("Generating DMA-Commands...\n");
	usleep(1000);

	for (int i = 0; i < BUFFER_COMMANDS; i++) {
		cbArr[i].TI = DMA_CB_TI_PERMAP_PWM | DMA_CB_TI_DEST_DREQ | DMA_CB_TI_NO_WIDE_BURSTS;
		cbArr[i].SOURCE_AD = UncachedMemBlock_to_physical(&srcPage, srcArray + i);
		cbArr[i].DEST_AD = PWM_BASE_BUS + PWM_FIF1; //write to the FIFO
		cbArr[i].TXFR_LEN = DMA_CB_TXFR_LEN_XLENGTH(4);
		cbArr[i].STRIDE = 0;
		cbArr[i].NEXTCONBK = UncachedMemBlock_to_physical(&cbPage, cbArr+((i+1) % BUFFER_COMMANDS));
	}

	//for (int i = 0; i < BUFFER_COMMANDS; i++)
//		logDmaControlBlock(cbArr + i);

	//TODO: Start DMA
	printf("Stopping DMA\n");

	printf("Previous DMA header:\n");
	logDmaChannelHeader(dmaHeader);
	usleep(1000);

	dmaHeader->CS |= DMA_CS_ABORT; //make sure to disable dma first.
	usleep(100); //give time for the abort command to be handled.
	
	dmaHeader->CS = DMA_CS_RESET;
	usleep(100);
	
	writeBitmasked(&dmaHeader->CS, DMA_CS_END, DMA_CS_END); //clear the end flag
	dmaHeader->DEBUG = DMA_DEBUG_READ_ERROR | DMA_DEBUG_FIFO_ERROR | DMA_DEBUG_READ_LAST_NOT_SET_ERROR; // clear debug error flags
	uint32_t firstAddr = UncachedMemBlock_to_physical(&cbPage, cbArr);
	printf("starting DMA @ CONBLK_AD=0x%08x\n", firstAddr);
	dmaHeader->CONBLK_AD = firstAddr; //(uint32_t)physCbPage + ((void*)cbArr - virtCbPage); //we have to point it to the PHYSICAL address of the control block (cb1)
	dmaHeader->CS = DMA_CS_PRIORITY(7) | DMA_CS_PANIC_PRIORITY(7) | DMA_CS_DISDEBUG; //high priority (max is 7)
	dmaHeader->CS = DMA_CS_PRIORITY(7) | DMA_CS_PANIC_PRIORITY(7) | DMA_CS_DISDEBUG | DMA_CS_ACTIVE; //activate DMA. 

	time_t start_time = time(0);
	
	HDB3Context hdb3 = HDB3Context();
	int command_index = -1;

	int output_pos = 0;
	uint32_t out_p = 0;
	uint32_t out_m = 0;
	uint32_t last_version = 0;
	size_t bytes = filled_buffers.size() * NUM_FRAMES_PER_ZMQ_MESSAGE * 6144;
	while(1) {
		//For ZMQ-Message in Message-Queue:
			//For ETI-Frame in ZMQ-Message:
				//For Byte in ETI-Frame:
					//For Bit in Byte:
						//Calculate HDB3
						//For 32 Bit block of Output bytes
							//When waiting: Try to read next ZMQ-Message
		zmq_dab_message_t *dab_msg = get_next_read_buffer(command_index >= 0);

		if (command_index < 0) {
			//This is the first command, so we position ourself right at the DMA-Loop
			command_index = (dmaHeader->CONBLK_AD - cbPage.bus_addr) / sizeof(DmaControlBlock);
			command_index = (command_index + BUFFER_COMMANDS / 2) % BUFFER_COMMANDS;
			printf("Got first command. Starting at command=%d\n", command_index);
		}

		int frame_offset = 0;
		for (int frame = 0; frame < NUM_FRAMES_PER_ZMQ_MESSAGE; frame++) {
			for (int frame_byte = 0; frame_byte < 6144; frame_byte++) {
				uint8_t byte_data;
				if (frame_byte < dab_msg->buflen[frame]) {
					byte_data = dab_msg->buf[frame_offset + frame_byte];
				} else {
					byte_data = 0x55; //Frame-Padding for ETI
				}
				uint8_t byte_data_mask = 1<<7;
				while (byte_data_mask > 0) {
					bool bit_data = (byte_data & byte_data_mask) > 0;
					while(hdb3.hasOutput()) {
						int hdb3_out = hdb3.getOutput();
						out_p =(out_p << 2) | (hdb3_out > 0 ? 1 : 0);
						out_m =(out_m << 2) | (hdb3_out < 0 ? 1 : 0);
						output_pos += 2;
						if (output_pos == 32) {
							uint32_t last_next = UncachedMemBlock_to_physical(&cbPage, cbArr + command_index);
							while(dmaHeader->CONBLK_AD == last_next) {
							}
							srcArray[command_index] = out_p;
							last_next = UncachedMemBlock_to_physical(&cbPage, cbArr + (command_index + 1) % BUFFER_COMMANDS);

							while(dmaHeader->CONBLK_AD == last_next) {
							}
							srcArray[(command_index + 1) % BUFFER_COMMANDS] = out_m;
							command_index = (command_index + 2) % BUFFER_COMMANDS;
							output_pos = 0;
						}
					}
					hdb3.inputData(bit_data);

					byte_data_mask >>= 1;
				}

			}
			frame_offset += dab_msg->buflen[frame];
			bytes += 6144;
		}


		if (command_index < 2 || 1) {
			int dma_index = (dmaHeader->CONBLK_AD - cbPage.bus_addr) / sizeof(DmaControlBlock);
			printf("ZMQ-Buffer: %2d DMA-Buffer: %6.2f%% %10d %10d %4d\n", filled_buffers.size(), 
				((BUFFER_COMMANDS + dma_index - command_index) % BUFFER_COMMANDS) * 100.0f / BUFFER_COMMANDS,
				command_index, dma_index, dab_msg->version);
			logPWMState(pwmHeader->STA);

			if ((pwmHeader->STA & (0xF0)) > 0) {
				printf("GAP!!!\n");
				pwmHeader->STA = 0xF0;
			}
			printf("Avg-Speed: %20.3f\n", (bytes * 1.0/ (time(0) - start_time)));
			if ((last_version + 1) % 2048 != dab_msg->version) {
				printf("LOST!!!\n");
			}
			last_version = dab_msg->version;

		}
	}

	printf("Cleanup...\n");
	pwmHeader->CTL = 0;
	*(clockBaseMem + CM_PWMCTL/4) = CM_PWMCTL_PASSWD | ((*(clockBaseMem + CM_PWMCTL/4))&(~CM_PWMCTL_ENAB)); //disable clock
	do {} while (*(clockBaseMem + CM_PWMCTL/4) & CM_PWMCTL_BUSY); //wait for clock to deactivate
	

	cleanup();
	UncachedMemBlock_free(&srcPage);
	UncachedMemBlock_free(&cbPage);

	return 0;
}