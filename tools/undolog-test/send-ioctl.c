#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <unistd.h>

#define DEVICE_PATH "/dev/bpf_rcuht_sync"
#define BPF_READY _IO('B', 1)
#define END_BENCHMARK _IO('B', 2)

int main(int argc, char **argv) {
    if(argc != 2) {
        printf("Usage ./send_ioctl Number:[READY:1,END:2]\n");
        return 1;
    }

    int ioctl_num = 0;
    switch(atoi(argv[1])) {
        case 1: ioctl_num = BPF_READY; break;
        case 2: ioctl_num = END_BENCHMARK; break;
        default: printf("Unkown ioctl number\n"); return 2;
    }

    int fd = open(DEVICE_PATH, O_RDWR);
    if (fd < 0) {
        perror("open");
        return 1;
    }
    
    if (ioctl(fd, ioctl_num) < 0) {
        perror("ioctl BPF_READY");
        close(fd);
        return 1;
    }
    
    printf("BPF ready signal sent! Module kthread unblocked.\n");
    close(fd);
    return 0;
}

