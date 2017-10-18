#include "pwm_dma_eti_sys.h" //for uint32_t & PAGE_SIZE
#include <stdlib.h> //for exit, valloc
#include <assert.h>
#include <string.h> //for memset
#include <stdio.h>
#include <sys/mman.h> //for mmap
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


//WARNING!!! CANNOT MOVE OBJECTS OF THIS CLASS! Destructor will be called and ruin your day!

static int mbox_fd = -1;   // used internally by the UncachedMemBlock-functions.

// A memory block that represents memory that is allocated in physical
// memory and locked there so that it is not swapped out.
// It is not backed by any L1 or L2 cache, so writing to it will directly
// modify the physical memory (and it is slower of course to do so).
// This is memory needed with DMA applications so that we can write through
// with the CPU and have the DMA controller 'see' the data.
// The UncachedMemBlock_{alloc,free,to_physical}
// functions are meant to operate on these.
class UncachedMemBlock {
public:


    void *mem = 0;  // User visible value: the memory to use.
    //-- Internal representation.
    uint32_t bus_addr = 0;
    uint32_t mem_handle = 0;
    size_t size = 0;
    UncachedMemBlock() {
        mem = 0;
        bus_addr = 0;
        mem_handle = 0;
    }
    // Allocate a block of memory of the given size (which is rounded up to the next
    // full page). The memory will be aligned on a page boundary and zeroed out.
    UncachedMemBlock(size_t size_) {
        if (size_ == 0) {
            mem = 0;
            bus_addr = 0;
            mem_handle = 0;
            return;
        }
        if (mbox_fd < 0) {
            mbox_fd = mbox_open();
            assert(mbox_fd >= 0);  // Uh, /dev/vcio not there ?
        }
        // Round up to next full page.
        size = size_ % PAGE_SIZE == 0 ? size_ : (size_ + PAGE_SIZE) & ~(PAGE_SIZE - 1);

        mem_handle = mem_alloc(mbox_fd, size, PAGE_SIZE, MEM_FLAG_L1_NONALLOCATING);
        bus_addr = mem_lock(mbox_fd, mem_handle);
        mem = mapmem(BUS_TO_PHYS(bus_addr), size);
        fprintf(stderr, "Alloc: %6d bytes;  %p (bus=0x%08x, phys=0x%08x)\n",
          (int)size, mem, bus_addr, BUS_TO_PHYS(bus_addr));
        assert(bus_addr);  // otherwise: couldn't allocate contiguous block.
        memset(mem, 0x00, size);
        fprintf(stderr, "Success....\n");
    }

    uintptr_t to_physical(void *p) {
        uint32_t offset = (uint8_t*)p - (uint8_t*)mem;
        assert(offset < size);   // pointer not within our block.
        return bus_addr + offset;
    }

    ~UncachedMemBlock() {
        if (mem == NULL) return;
        fprintf(stderr, "Destructor called on: %p\n", mem);
        assert(mbox_fd >= 0);  // someone should've initialized that on allocate.
        unmapmem(mem, size);
        mem_unlock(mbox_fd, mem_handle);
        mem_free(mbox_fd, mem_handle);
        mem = NULL;
    }
};