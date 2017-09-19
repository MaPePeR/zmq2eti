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
	printf("Dma Ch Header:\n CS: 0x%08x\n CONBLK_AD: 0x%08x\n TI: 0x%08x\n SOURCE_AD: 0x%08x\n DEST_AD: 0x%08x\n TXFR_LEN: %u\n STRIDE: 0x%08x\n NEXTCONBK: 0x%08x\n DEBUG: 0x%08x\n", h->CS, h->CONBLK_AD, h->TI, h->SOURCE_AD, h->DEST_AD, h->TXFR_LEN, h->STRIDE, h->NEXTCONBK, h->DEBUG);
}

void logDmaControlBlock(struct DmaControlBlock *b) {
	printf("Dma Control Block:\n TI: 0x%08x\n SOURCE_AD: 0x%08x\n DEST_AD: 0x%08x\n TXFR_LEN: 0x%08x\n STRIDE: 0x%08x\n NEXTCONBK: 0x%08x\n unused: 0x%08x %08x\n", b->TI, b->SOURCE_AD, b->DEST_AD, b->TXFR_LEN, b->STRIDE, b->NEXTCONBK, b->_reserved[0], b->_reserved[1]);
}

void cleanup() {
	printf("Cleanup\n");
	//disable DMA. Otherwise, it will continue to run in the background, potentially overwriting future user data.
	if(dmaHeader) {
		printf("Stopping DMA\n");
		writeBitmasked(&dmaHeader->CS, DMA_CS_ACTIVE, 0);
		usleep(100);
		writeBitmasked(&dmaHeader->CS, DMA_CS_RESET, DMA_CS_RESET);
	}
}

void cleanupAndExit(int sig) {
	cleanup();
	printf("Exiting with error; caught signal: %i\n", sig);
	exit(1);
}

int main() {

	volatile uint32_t *dmaBaseMem;
	for (int i = 0; i < 64; i++) { //catch all signals (like ctrl+c, ctrl+z, ...) to ensure DMA is disabled
		struct sigaction sa;
		memset(&sa, 0, sizeof(sa));
		sa.sa_handler = cleanupAndExit;
		sigaction(i, &sa, NULL);
	}
	setSchedPriority(SCHED_PRIORITY);

		int memfd = open("/dev/mem", O_RDWR | O_SYNC);
	if (memfd < 0) {
		printf("Failed to open /dev/mem (did you remember to run as root?)\n");
		exit(1);
	}

	dmaBaseMem = mapPeripheral(memfd, DMA_BASE);
	dmaHeader = (struct DmaChannelHeader*)(dmaBaseMem + DMACH(DMA_CHANNEL)/4); //must divide by 4, as dmaBaseMem is uint32_t*



	//Setup DMA
	printf("Allocating locked memory\n");
	usleep(1000);

	size_t srcPageBytes = sizeof(uint32_t) * BUFFER_COMMANDS;

	struct UncachedMemBlock srcPage = UncachedMemBlock_alloc(srcPageBytes);
	
	uint32_t *srcArray = (uint32_t*)srcPage.mem; 
	for (int i = 0; i < srcPageBytes; i+=4) {
		srcArray[i/4] = i/4 + 23;
	}

	struct UncachedMemBlock destPage = UncachedMemBlock_alloc(srcPageBytes);
	
	uint32_t *destArray = (uint32_t*)destPage.mem; 

	memset(destPage.mem, 0, srcPageBytes); //Just make sure its 0ed.

	//allocate memory for the control blocks
	size_t cbPageBytes = BUFFER_COMMANDS * sizeof(struct DmaControlBlock); //3 cbs for each source block
	struct UncachedMemBlock cbPage = UncachedMemBlock_alloc(cbPageBytes);
	struct DmaControlBlock *cbArr = (struct DmaControlBlock*)cbPage.mem;

	printf("Memory:\n  Data-Buffer-Size: %10zd Bytes\n  Command-Buffer-Size: %10zd Bytes\n", srcPageBytes, cbPageBytes);

	printf("Generating DMA-Commands...\n");
	usleep(1000);

	cbArr[0].TI = DMA_CB_TI_NO_WIDE_BURSTS;
	cbArr[0].SOURCE_AD = UncachedMemBlock_to_physical(&srcPage, srcArray);
	cbArr[0].DEST_AD = UncachedMemBlock_to_physical(&destPage, destArray); //write to the FIFO
	cbArr[0].TXFR_LEN = DMA_CB_TXFR_LEN_XLENGTH(4*3);
	cbArr[0].STRIDE = 0;
	cbArr[0].NEXTCONBK = 0;

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

	while((dmaHeader->CS & DMA_CS_ACTIVE) != 0) {

	}

	for (int i = 0; i < 5; i++) {
		printf("dest[%d] = %d; src[%d] = %d\n", i, destArray[i], i, srcArray[i]);
	}

	cleanup();
	UncachedMemBlock_free(&srcPage);
	UncachedMemBlock_free(&cbPage);
	return 0;
}