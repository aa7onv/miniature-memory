// mmap_test.c
// confirms mmap()/dev-mem access to the HPS-to-FPGA bridge
// works on linux image, 

// writing to the LED PIO register and checking
// if onboard LEDs (LEDR) respond.
// address values from address_map_arm.h

// compile gcc mmap_test.c -o mmap_test
// run      sudo ./mmap_test 

#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>

#define LW_BRIDGE_BASE   0xFF200000   // lightweight HPS-to-FPGA bridge base
#define LW_BRIDGE_SPAN   0x00005000   // bridge span
#define LED_PIO_OFFSET   0x00000000   // LEDR_BASE

int main() {
    int fd;
    void *virtual_base;
    volatile unsigned long *led_pio_addr;

    fd = open("/dev/mem", O_RDWR | O_SYNC);
    if (fd < 0) {
        perror("open /dev/mem failed");
        return 1;
    }

    virtual_base = mmap(NULL, LW_BRIDGE_SPAN, PROT_READ | PROT_WRITE,
                         MAP_SHARED, fd, LW_BRIDGE_BASE);
    if (virtual_base == MAP_FAILED) {
        perror("mmap failed");
        close(fd);
        return 1;
    }

    led_pio_addr = (volatile unsigned long *)
        ((char *)virtual_base + LED_PIO_OFFSET);

    printf("Reading current LED register value: 0x%08lx\n", *led_pio_addr);

    printf("Writing 0x3FF (all 10 LEDs on)...\n");
    *led_pio_addr = 0x3FF;
    sleep(2);

    printf("Writing 0x155 (alternating LEDs)...\n");
    *led_pio_addr = 0x155;
    sleep(2);

    printf("Writing 0x000 (all off)...\n");
    *led_pio_addr = 0x000;

    munmap(virtual_base, LW_BRIDGE_SPAN);
    close(fd);
    printf("Done.\n");
    return 0;
}