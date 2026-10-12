// test linux HPS to FPGA bridge register access
//gcc sensor_read_test.c -o sensor_read_test
//sudo ./sensor_read_test

#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h> //file, /dev/mem
#include <sys/mman.h> //mem, virtual addy
#include <unistd.h> //sleep

//from address_map_arm.h
#define LW_BRIDGE_BASE 0xFF200000 //fixed mem space
#define LW_BRIDGE_SPAN 0x00005000 //

#define SENSOR_SLAVE_BASE 0x00000100   // base address from Platform Designer
#define DIST1_OFFSET 0x00000000   // word 0 -> sensor 1 (byte offset 0x0)
#define DIST2_OFFSET 0x00000004   // word 1 -> sensor 2 (byte offset 0x4)

int main() { //try open phys mem
    int fd = open("/dev/mem", O_RDWR | O_SYNC);
    if (fd < 0) {
        perror("open /dev/mem failed");
        return 1;
    }

    // map HW  address range to virtuail mem
    // mmap(address, length, protection, flags, file_descriptor, offset)
    void *virtual_base = mmap(NULL, LW_BRIDGE_SPAN, PROT_READ | PROT_WRITE,
                               MAP_SHARED, fd, LW_BRIDGE_BASE);
    if (virtual_base == MAP_FAILED) {
        perror("mmap failed");
        close(fd);
        return 1;
    }

    // reg addr = map virtual base + periph offset + reg offset
    volatile unsigned long *dist1_addr = (volatile unsigned long *)  
        ((char *)virtual_base + SENSOR_SLAVE_BASE + DIST1_OFFSET);
    volatile unsigned long *dist2_addr = (volatile unsigned long *)
        ((char *)virtual_base + SENSOR_SLAVE_BASE + DIST2_OFFSET);

    printf("Reading sensor registers at base 0x%08x (offsets 0x%x, 0x%x).\n",
           LW_BRIDGE_BASE + SENSOR_SLAVE_BASE, DIST1_OFFSET, DIST2_OFFSET);
    printf("Wave a hand to test. Ctrl+C to stop.\n\n");

    while (1) {
        unsigned long d1 = *dist1_addr;
        unsigned long d2 = *dist2_addr;
        printf("sensor1 = %4lu cm    sensor2 = %4lu cm\n", d1, d2);
        sleep(1);
    }

    munmap(virtual_base, LW_BRIDGE_SPAN);
    close(fd);
    return 0;
}