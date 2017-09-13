
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
#define BUFFER_SECONDS (3)
//We output 2 * 2048 Bits per second onto 2 channels at 32 bit per command
#define BUFFER_COMMANDS (BUFFER_SECONDS * 2048 * 2 * 2 / 32)

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
	if (ret = pthread_setschedparam(pthread_self(), SCHED_FIFO, &sp)) {
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
	printf("Dma Ch Header:\n CS: 0x%08x\n CONBLK_AD: 0x%08x\n TI: 0x%08x\n SOURCE_AD: 0x%08x\n DEST_AD: 0x%08x\n TXFR_LEN: %u\n STRIDE: 0x%08x\n NEXTCONBK: 0x%08x\n DEBUG: 0x%08x\n", h->CS, h->CONBLK_AD, h->TI, h->SOURCE_AD, h->DEST_AD, h->TXFR_LEN, h->STRIDE, h->NEXTCONBK, h->DEBUG);
}

void logDmaControlBlock(struct DmaControlBlock *b) {
	printf("Dma Control Block:\n TI: 0x%08x\n SOURCE_AD: 0x%08x\n DEST_AD: 0x%08x\n TXFR_LEN: 0x%08x\n STRIDE: 0x%08x\n NEXTCONBK: 0x%08x\n unused: 0x%08x %08x\n", b->TI, b->SOURCE_AD, b->DEST_AD, b->TXFR_LEN, b->STRIDE, b->NEXTCONBK, b->_reserved[0], b->_reserved[1]);
}

#define ZMQ_MESSAGE_BUFFER_SIZE (3)
void *zmq_ctx;
void *zmq_sock;
zmq_dab_message_t zmq_msg_buffer[ZMQ_MESSAGE_BUFFER_SIZE];
int zmq_msg_buffer_count = 0;
int zmq_msg_buffer_pos = 0;

void cleanup() {
	printf("Cleanup\n");
	//disable DMA. Otherwise, it will continue to run in the background, potentially overwriting future user data.
	if(dmaHeader) {
		writeBitmasked(&dmaHeader->CS, DMA_CS_ACTIVE, 0);
		usleep(100);
		writeBitmasked(&dmaHeader->CS, DMA_CS_RESET, DMA_CS_RESET);
	}
	pwmHeader->CTL = 0;
	if (zmq_ctx) {
		zmq_term(zmq_ctx);
	}
	//could also disable PWM, but that's not imperative.
}

void cleanupAndExit(int sig) {
	cleanup();
	printf("Exiting with error; caught signal: %i\n", sig);
	exit(1);
}




int main(int argc, const char *argv[]) {
	//Setup peripherals
	volatile uint32_t *gpio, *dmaBaseMem, *pwmBaseMem, *timerBaseMem, *clockBaseMem;
	//emergency clean-up:
	for (int i = 0; i < 64; i++) { //catch all signals (like ctrl+c, ctrl+z, ...) to ensure DMA is disabled
		struct sigaction sa;
		memset(&sa, 0, sizeof(sa));
		sa.sa_handler = cleanupAndExit;
		sigaction(i, &sa, NULL);
	}
	setSchedPriority(SCHED_PRIORITY);
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
	if (zmq_connect(zmq_sock, argv[1]) < 0) {
		fprintf(stderr, "Failed to connect to zmq-endpoint %s\n", argv[1]);
		exit(1);
	}
	int rc = zmq_setsockopt (zmq_sock, ZMQ_SUBSCRIBE, "", 0);
	assert (rc == 0);

	//First, open the linux device, /dev/mem
	//dev/mem provides access to the physical memory of the entire processor+ram
	//This is needed because Linux uses virtual memory, thus the process's memory at 0x00000000 will NOT have the same contents as the physical memory at 0x00000000
	int memfd = open("/dev/mem", O_RDWR | O_SYNC);
	if (memfd < 0) {
		printf("Failed to open /dev/mem (did you remember to run as root?)\n");
		exit(1);
	}
	int pagemapfd = open("/proc/self/pagemap", O_RDONLY);
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
	do {} while (*(clockBaseMem + CM_PWMCTL/4) & CM_PWMCTL_BUSY == 0); //wait for clock to activate

	printf("setting up PWM\n");
	//Setup PWM
	pwmHeader->DMAC = 0; //disable DMA
	pwmHeader->CTL |= PWM_CTL_CLRFIFO; //clear pwm
	usleep(100);
	
	pwmHeader->STA = PWM_STA_ERRS; //clear PWM errors
	usleep(100);
	
	pwmHeader->DMAC = PWM_DMAC_EN | PWM_DMAC_DREQ(7) | PWM_DMAC_PANIC(7); //DREQ is activated at queue < PWM_FIFO_SIZE
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
	
	HDB3Context hdb3 = HDB3Context();
	int command_index = 0;

	int output_pos = 0;
	uint32_t out_p = 0;
	uint32_t out_m = 0;

	while(1) {
		//For ZMQ-Message in Message-Queue:
			//For ETI-Frame in ZMQ-Message:
				//For Byte in ETI-Frame:
					//For Bit in Byte:
						//Calculate HDB3
						//For 32 Bit block of Output bytes
							//When waiting: Try to read next ZMQ-Message
		if (zmq_msg_buffer_count == 0) {
			zmq_msg_buffer_pos = 0;
			rc = zmq_recv(zmq_sock, &zmq_msg_buffer[zmq_msg_buffer_pos], sizeof(*zmq_msg_buffer), 0);
			assert(rc <= sizeof(*zmq_msg_buffer));
		} else {
			zmq_msg_buffer_pos = (zmq_msg_buffer_pos + 1) % ZMQ_MESSAGE_BUFFER_SIZE;
		}
		zmq_dab_message_t *dab_msg = &zmq_msg_buffer[zmq_msg_buffer_pos];

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
								if (zmq_msg_buffer_count < ZMQ_MESSAGE_BUFFER_SIZE) {
									int next_free_buffer_index = (zmq_msg_buffer_pos + zmq_msg_buffer_count) % ZMQ_MESSAGE_BUFFER_SIZE;
									rc = zmq_recv(zmq_sock, &zmq_msg_buffer[next_free_buffer_index], sizeof(*zmq_msg_buffer), 0);
									if (rc > 0) {
										assert(rc <= sizeof(*zmq_msg_buffer));
										zmq_msg_buffer_count += 1;
									}
								}
								usleep(10000);
							}
							srcArray[command_index] = out_p;
							last_next = UncachedMemBlock_to_physical(&cbPage, cbArr + command_index + 1);

							while(dmaHeader->CONBLK_AD == last_next) {
							}
							srcArray[command_index + 1] = out_m;
							command_index = (command_index + 2) % BUFFER_COMMANDS;
							output_pos = 0;
						}
					}
					hdb3.inputData(bit_data);

					byte_data_mask >>= 1;
				}

			}
			frame_offset += dab_msg->buflen[frame];
		}


		zmq_msg_buffer_count--;
	}

	printf("Cleanup...\n");
	pwmHeader->CTL = 0;
	*(clockBaseMem + CM_PWMCTL/4) = CM_PWMCTL_PASSWD | ((*(clockBaseMem + CM_PWMCTL/4))&(~CM_PWMCTL_ENAB)); //disable clock
	do {} while (*(clockBaseMem + CM_PWMCTL/4) & CM_PWMCTL_BUSY); //wait for clock to deactivate
	

	cleanup();

	return 0;
}