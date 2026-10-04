// sensor_read_test.c
// Reads both HC-SR04 distance registers (sensor_avalon_slave component)
// over the lightweight HPS-to-FPGA bridge, once per second, so you can
// wave a hand near each sensor and watch the printed values change live.
//
// This is the end-to-end proof that Verilog FSM -> Avalon-MM slave ->
// lightweight bridge -> Linux userspace all actually works.
//
// Compile:  gcc sensor_read_test.c -o sensor_read_test
// Run:      sudo ./sensor_read_test
// Ctrl+C to stop.

#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>

#define LW_BRIDGE_BASE     0xFF200000
#define LW_BRIDGE_SPAN     0x00005000   // matches this system's actual valid span

#define SENSOR_SLAVE_BASE  0x00000100   // base address you set in Platform Designer
#define DIST1_OFFSET       0x00000000   // word 0 -> sensor 1 (byte offset 0x0)
#define DIST2_OFFSET       0x00000004   // word 1 -> sensor 2 (byte offset 0x4)

int main() {
    int fd = open("/dev/mem", O_RDWR | O_SYNC);
    if (fd < 0) {
        perror("open /dev/mem failed (are you running with sudo?)");
        return 1;
    }

    void *virtual_base = mmap(NULL, LW_BRIDGE_SPAN, PROT_READ | PROT_WRITE,
                               MAP_SHARED, fd, LW_BRIDGE_BASE);
    if (virtual_base == MAP_FAILED) {
        perror("mmap failed");
        close(fd);
        return 1;
    }

    volatile unsigned long *dist1_addr = (volatile unsigned long *)
        ((char *)virtual_base + SENSOR_SLAVE_BASE + DIST1_OFFSET);
    volatile unsigned long *dist2_addr = (volatile unsigned long *)
        ((char *)virtual_base + SENSOR_SLAVE_BASE + DIST2_OFFSET);

    printf("Reading sensor registers at base 0x%08x (offsets 0x%x, 0x%x).\n",
           LW_BRIDGE_BASE + SENSOR_SLAVE_BASE, DIST1_OFFSET, DIST2_OFFSET);
    printf("Wave a hand near each sensor and watch for changes. Ctrl+C to stop.\n\n");

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