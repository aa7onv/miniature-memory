
// Reads distance registers over the lightweight bridge and
// sends each as an OSC message (/sensor1, /sensor2, {float, cm}) over UDP to TouchDesigner, 
// ~10 times per second.
//
// build OSC packet format:
// address pattern string, type tag string, argument(s). each padded to a 4-byte boundary
//
// gcc osc_data.c -o osc_data
// sudo ./osc_data <TouchDesigner_IP> <port>
// sudo ./osc_data 192.168.1.50 7000


#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <sys/socket.h>
#include <arpa/inet.h> 
#include <unistd.h>

//from address_map_arm.h
#define LW_BRIDGE_BASE     0xFF200000
#define LW_BRIDGE_SPAN     0x00005000

#define SENSOR_SLAVE_BASE  0x00000100
#define DIST1_OFFSET       0x00000000
#define DIST2_OFFSET       0x00000004

// for osc, string fields bytes must be divisible by 4. 
static int round_up4(int n) {
    return (n + 3) & ~3;
}

// build minimal OSC message with ONE float32 argument into buf.
// address must start with '/'. Returns the total packet length.
static int build_osc_float_message(char *buf, const char *address, float value) {
    int pos = 0; //track buf position

    // ==== OSC addr  pattern: null-terminated, padded to 4 bytes ==
    int addr_len = strlen(address) + 1; // +1 for null terminator
    int addr_padded = round_up4(addr_len);
    memset(buf + pos, 0, addr_padded); //initialize region as 0 
    memcpy(buf + pos, address, strlen(address)); // copies bytes from one mem location to another 
    pos += addr_padded;

    // ==== OSC Type Tag String: ",f" (comma + one float tag), padded ====
    const char *type_tags = ",f";
    int tag_len = strlen(type_tags) + 1;
    int tag_padded = round_up4(tag_len);
    memset(buf + pos, 0, tag_padded);
    memcpy(buf + pos, type_tags, strlen(type_tags));
    pos += tag_padded;

    // osc requires 32-b it IEEE 754 bit pattern in big endian order. 
    // HPS/ARMS CPU stores in little endian. cant convert float to big endian directly, so convert to int first, then use htonl() to swap bytes.

    union { float f; unsigned int i; } conv;
    conv.f = value;
    unsigned int big_endian_bits = htonl(conv.i); // conversion
    memcpy(buf + pos, &big_endian_bits, 4);
    pos += 4;

    return pos;
}

int main(int argc, char *argv[]) {
    if (argc != 3) {
        fprintf(stderr, "Usage: %s <TouchDesigner_IP> <port>\n", argv[0]);
        fprintf(stderr, "Example: %s 192.168.1.50 10000\n", argv[0]);
        return 1;
    }
    const char *td_ip = argv[1];
    int td_port = atoi(argv[2]);

    // =========== set up the FPGA register access (same as sensor_read_test.c)===============
    int mem_fd = open("/dev/mem", O_RDWR | O_SYNC);
    if (mem_fd < 0) {
        perror("open /dev/mem failed");
        return 1;
    }

    void *virtual_base = mmap(NULL, LW_BRIDGE_SPAN, PROT_READ | PROT_WRITE,
                               MAP_SHARED, mem_fd, LW_BRIDGE_BASE);
    if (virtual_base == MAP_FAILED) {
        perror("mmap failed");
        close(mem_fd);
        return 1;
    }

    volatile unsigned long *dist1_addr = (volatile unsigned long *)
        ((char *)virtual_base + SENSOR_SLAVE_BASE + DIST1_OFFSET);
    volatile unsigned long *dist2_addr = (volatile unsigned long *)
        ((char *)virtual_base + SENSOR_SLAVE_BASE + DIST2_OFFSET);
    /////// ========================================

    // ---- set up the UDP socket to TouchDesigner ----
    int sock_fd = socket(AF_INET, SOCK_DGRAM, 0);
    if (sock_fd < 0) {
        perror("socket failed");
        munmap(virtual_base, LW_BRIDGE_SPAN);
        close(mem_fd);
        return 1;
    }

    struct sockaddr_in td_addr;
    memset(&td_addr, 0, sizeof(td_addr));
    td_addr.sin_family = AF_INET;
    td_addr.sin_port = htons(td_port);
    if (inet_pton(AF_INET, td_ip, &td_addr.sin_addr) != 1) {
        fprintf(stderr, "Invalid IP address: %s\n", td_ip);
        close(sock_fd);
        munmap(virtual_base, LW_BRIDGE_SPAN);
        close(mem_fd);
        return 1;
    }

    printf("Sending OSC to %s:%d  (/sensor1, /sensor2, each one float in cm)\n",
           td_ip, td_port);
    printf("Ctrl+C to stop.\n\n");

    char packet[64];

    while (1) {
        float d1 = (float)(*dist1_addr);
        float d2 = (float)(*dist2_addr);

        int len1 = build_osc_float_message(packet, "/sensor1", d1);
        sendto(sock_fd, packet, len1, 0,
               (struct sockaddr *)&td_addr, sizeof(td_addr));

        int len2 = build_osc_float_message(packet, "/sensor2", d2);
        sendto(sock_fd, packet, len2, 0,
               (struct sockaddr *)&td_addr, sizeof(td_addr));

        printf("sent: sensor1=%.0f cm  sensor2=%.0f cm\n", d1, d2);

        usleep(100000); // ~10 times per second
    }

    close(sock_fd);
    munmap(virtual_base, LW_BRIDGE_SPAN);
    close(mem_fd);
    return 0;
}