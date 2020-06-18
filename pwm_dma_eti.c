
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

#define RPI_V2
#include "hw-addresses.h"
#include "pwm_dma_eti_sys.h"

#define SCHED_PRIORITY 30 //Linux scheduler priority. Higher = more realtime

#define DMA_CHANNEL (5)
#define CLOCK_DIVI (122)
#define CLOCK_DIVF (72)

size_t ceilToPage(size_t size) {
    //round up to nearest page-size multiple
    if (size & (PAGE_SIZE-1)) {
        size += PAGE_SIZE - (size & (PAGE_SIZE-1));
    }
    return size;
}

uintptr_t virtToPhys(void* virt, int pagemapfd) {
    uintptr_t pgNum = (uintptr_t)(virt)/PAGE_SIZE;
    int byteOffsetFromPage = (uintptr_t)(virt)%PAGE_SIZE;
    uint64_t physPage;
    ///proc/self/pagemap is a uint64_t array where the index represents the virtual page number and the value at that index represents the physical page number.
    //So if virtual address is 0x1000000, read the value at *array* index 0x1000000/PAGE_SIZE and multiply that by PAGE_SIZE to get the physical address.
    //because files are bytestreams, one must explicitly multiply each byte index by 8 to treat it as a uint64_t array.
    int err = lseek(pagemapfd, pgNum*8, SEEK_SET);
    if (err != pgNum*8) {
        printf("WARNING: virtToPhys %p failed to seek (expected %i got %i. errno: %i)\n", virt, pgNum*8, err, errno);
    }
    read(pagemapfd, &physPage, 8);
    if (!physPage & (1ull<<63)) { //bit 63 is set to 1 if the page is present in ram
        printf("WARNING: virtToPhys %p has no physical address\n", virt);
    }
    physPage = physPage & ~(0x1ffull << 55); //bits 55-63 are flags.
    uintptr_t mapped = (uintptr_t)(physPage*PAGE_SIZE + byteOffsetFromPage);
    return mapped;
}

uintptr_t virtToUncachedPhys(void *virt, int pagemapfd) {
	//0xc0000000 was changed from 0x40000000 for RPI_V2
    //return virtToPhys(virt, pagemapfd) | 0x40000000; //bus address of the ram is 0x40000000. With this binary-or, writes to the returned address will bypass the CPU (L1) cache, but not the L2 cache. 0xc0000000 should be the base address if L2 must also be bypassed. However, the DMA engine is aware of L2 cache - just not the L1 cache (source: http://en.wikibooks.org/wiki/Aros/Platforms/Arm_Raspberry_Pi_support#Framebuffer )
    return virtToPhys(virt, pagemapfd) &~0xC0000000; //bus address of the ram is 0x40000000. With this binary-or, writes to the returned address will bypass the CPU (L1) cache, but not the L2 cache. 0xc0000000 should be the base address if L2 must also be bypassed. However, the DMA engine is aware of L2 cache - just not the L1 cache (source: http://en.wikibooks.org/wiki/Aros/Platforms/Arm_Raspberry_Pi_support#Framebuffer )
}


//allocate some memory and lock it so that its physical address will never change
void* makeLockedMem(size_t size) {
    //void* mem = valloc(size); //memory returned by valloc is not zero'd
    size = ceilToPage(size);
    void *mem = mmap(
        NULL,   //let kernel place memory where it wants
        size,   //length
        PROT_WRITE | PROT_READ, //ask for read and write permissions to memory
        MAP_SHARED | 
        MAP_ANONYMOUS | //no underlying file; initialize to 0
        MAP_NORESERVE | //don't reserve swap space
        MAP_LOCKED, //lock into *virtual* ram. Physical ram may still change!
        -1,	// File descriptor
    0); //no offset into file (file doesn't exist).
    if (mem == MAP_FAILED) {
        printf("makeLockedMem failed\n");
        exit(1);
    }
    memset(mem, 0, size); //simultaneously zero the pages and force them into memory
    mlock(mem, size);
    return mem;
}

//free memory allocated with makeLockedMem
void freeLockedMem(void* mem, size_t size) {
    size = ceilToPage(size);
    munlock(mem, size);
    munmap(mem, size);
}

void* makeUncachedMemView(void* virtaddr, size_t bytes, int memfd, int pagemapfd) {
    //by default, writing to any virtual address will go through the CPU cache.
    //this function will return a pointer that behaves the same as virtaddr, but bypasses the CPU L1 cache (note that because of this, the returned pointer and original pointer should not be used in conjunction, else cache-related inconsistencies will arise)
    //Note: The original memory should not be unmapped during the lifetime of the uncached version, as then the OS won't know that our process still owns the physical memory.
    bytes = ceilToPage(bytes);
    //first, just allocate enough *virtual* memory for the operation. This is done so that we can do the later mapping to a contiguous range of virtual memory:
    void *mem = mmap(
        NULL,   //let kernel place memory where it wants
        bytes,   //length
        PROT_WRITE | PROT_READ, //ask for read and write permissions to memory
        MAP_SHARED | 
        MAP_ANONYMOUS | //no underlying file; initialize to 0
        MAP_NORESERVE | //don't reserve swap space
        MAP_LOCKED, //lock into *virtual* ram. Physical ram may still change!
        -1,	// File descriptor
    0); //no offset into file (file doesn't exist).
    //now, free the virtual memory and immediately remap it to the physical addresses used in virtaddr
    munmap(mem, bytes); //Might not be necessary; MAP_FIXED indicates it can map an already-used page
    for (int offset=0; offset<bytes; offset += PAGE_SIZE) {
        void *mappedPage = mmap(mem+offset, PAGE_SIZE, PROT_WRITE|PROT_READ, MAP_SHARED|MAP_FIXED|MAP_NORESERVE|MAP_LOCKED, memfd, virtToUncachedPhys(virtaddr+offset, pagemapfd));
        if (mappedPage != mem+offset) { //We need these mappings to be contiguous over virtual memory (in order to replicate the virtaddr array), so we must ensure that the address we requested from mmap was actually used.
            printf("Failed to create an uncached view of memory at addr %p+0x%08x\n", virtaddr, offset);
            exit(1);
        }
    }
    memset(mem, 0, bytes); //Although the cached version might have been reset, those writes might not have made it through.
    return mem;
}

//free memory allocated with makeLockedMem
void freeUncachedMemView(void* mem, size_t size) {
    size = ceilToPage(size);
    munmap(mem, size);
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
    uint32_t new = (cur & (~mask)) | (value & mask);
    *dest = new;
    *dest = new; //best to be safe when crossing memory boundaries
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
        writeBitmasked(&dmaHeader->CS, DMA_CS_ACTIVE, 0);
        usleep(100);
        writeBitmasked(&dmaHeader->CS, DMA_CS_RESET, DMA_CS_RESET);
    }
    pwmHeader->CTL = 0;
    //could also disable PWM, but that's not imperative.
}

void cleanupAndExit(int sig) {
    cleanup();
    printf("Exiting with error; caught signal: %i\n", sig);
    exit(1);
}


int main() {
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
    
    pwmHeader->DMAC = PWM_DMAC_EN | PWM_DMAC_DREQ(1) | PWM_DMAC_PANIC(1); //DREQ is activated at queue < PWM_FIFO_SIZE
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

    #define N_COMMANDS (8)

	size_t srcPageBytes = sizeof(uint32_t) * N_COMMANDS;
	void *virtSrcPageCached = makeLockedMem(srcPageBytes);
	printf("Made locked mem: %p\n", virtSrcPageCached);
    usleep(1000);
    void *virtSrcPage = makeUncachedMemView(virtSrcPageCached, srcPageBytes, memfd, pagemapfd);
    printf("%p mappedPhysSrcPage: %p\n", virtSrcPage, virtToPhys(virtSrcPage, pagemapfd));
    usleep(1000);


    
    //cast virtSrcPage to a GpioBufferFrame array:
    uint32_t *srcArray = (uint32_t*)virtSrcPage; //Note: calling virtToPhys on srcArray will return NULL. Use srcArrayCached for that.
    uint32_t *srcArrayCached = (uint32_t*)virtSrcPageCached;

    //Fill with data for now
    srcArray[0] = 0xFFCC5500;
    srcArray[2] = 0x00000000;
    srcArray[4] = 0x00000000;
    srcArray[6] = 0x00000000;
    //srcArray[1] = 0x55555555;
    //srcArray[0] = 0xFFFFFFFF;
    //srcArray[1] = 0x00000000;
    srcArray[1] = 0xCCCCCCCC;
    srcArray[3] = 0x00000000;
    srcArray[5] = 0x00000000;
    srcArray[7] = 0x00000000;


    //pwmHeader->FIF1 = 16;
    //pwmHeader->FIF1 = 16;

	//allocate memory for the control blocks
    size_t cbPageBytes = N_COMMANDS * sizeof(struct DmaControlBlock); //3 cbs for each source block
    void *virtCbPageCached = makeLockedMem(cbPageBytes);
    void *virtCbPage = makeUncachedMemView(virtCbPageCached, cbPageBytes, memfd, pagemapfd);
    //fill the control blocks:
    struct DmaControlBlock *cbArrCached = (struct DmaControlBlock*)virtCbPageCached;
    struct DmaControlBlock *cbArr = (struct DmaControlBlock*)virtCbPage;

    printf("Generating DMA-Commands...\n");
    usleep(1000);

    for (int i = 0; i < N_COMMANDS; i++) {
	    cbArr[i].TI = DMA_CB_TI_PERMAP_PWM | DMA_CB_TI_DEST_DREQ | DMA_CB_TI_NO_WIDE_BURSTS;
	    cbArr[i].SOURCE_AD = virtToUncachedPhys(srcArrayCached + i, pagemapfd); //The data written doesn't matter, but using the GPIO source will hopefully bring it into L2 for more deterministic timing of the next control block.
	    cbArr[i].DEST_AD = PWM_BASE_BUS + PWM_FIF1; //write to the FIFO
	    cbArr[i].TXFR_LEN = DMA_CB_TXFR_LEN_XLENGTH(4);
	    cbArr[i].STRIDE = 0;
	    cbArr[i].NEXTCONBK = virtToUncachedPhys(cbArrCached+((i+1) % N_COMMANDS), pagemapfd); //have to use the cached version because the uncached version isn't listed in pagemap(?)
	}

    /*cbArr[1].TI = DMA_CB_TI_PERMAP_PWM | DMA_CB_TI_DEST_DREQ | DMA_CB_TI_NO_WIDE_BURSTS;
    cbArr[1].SOURCE_AD = virtToUncachedPhys(srcArrayCached + 1, pagemapfd); //The data written doesn't matter, but using the GPIO source will hopefully bring it into L2 for more deterministic timing of the next control block.
    cbArr[1].DEST_AD = PWM_BASE_BUS + PWM_FIF1; //write to the FIFO
    cbArr[1].TXFR_LEN = DMA_CB_TXFR_LEN_XLENGTH(4);
    cbArr[1].STRIDE = 0;
    cbArr[1].NEXTCONBK = virtToUncachedPhys(cbArrCached, pagemapfd); */
	for (int i = 0; i < N_COMMANDS; i++)
		logDmaControlBlock(cbArr + i);

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
    uint32_t firstAddr = virtToUncachedPhys(cbArrCached, pagemapfd);
    printf("starting DMA @ CONBLK_AD=0x%08x\n", firstAddr);
    dmaHeader->CONBLK_AD = firstAddr; //(uint32_t)physCbPage + ((void*)cbArr - virtCbPage); //we have to point it to the PHYSICAL address of the control block (cb1)
    dmaHeader->CS = DMA_CS_PRIORITY(7) | DMA_CS_PANIC_PRIORITY(7) | DMA_CS_DISDEBUG; //high priority (max is 7)
    dmaHeader->CS = DMA_CS_PRIORITY(7) | DMA_CS_PANIC_PRIORITY(7) | DMA_CS_DISDEBUG | DMA_CS_ACTIVE; //activate DMA. 
    
    uint32_t last_next = 0;
    printf("Sleeping...\n");
   	for (int i = 0; i < 32 * 100; i++) {
    	usleep(10000);
    	uint32_t current = dmaHeader->NEXTCONBK;
    	if (last_next != current) {
    		printf("Pos: %p\n", current);
    		last_next = current;
    	}
    }

    printf("Cleanup...\n");
    pwmHeader->CTL = 0;
    *(clockBaseMem + CM_PWMCTL/4) = CM_PWMCTL_PASSWD | ((*(clockBaseMem + CM_PWMCTL/4))&(~CM_PWMCTL_ENAB)); //disable clock
    do {} while (*(clockBaseMem + CM_PWMCTL/4) & CM_PWMCTL_BUSY); //wait for clock to deactivate
    

    cleanup();

	return 0;
}